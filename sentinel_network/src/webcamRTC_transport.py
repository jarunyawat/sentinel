#!/usr/bin/env python3

import cv2
import asyncio
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

pcs = set()


# ─── Camera Track ─────────────────────────────────────────────────────────────
class CameraTrack(VideoStreamTrack):
    """
    Reads from /dev/video0 OR from a ROS2 Image topic.
    Mode is selected at construction time.
    """

    def __init__(self, ros_node=None, use_ros_topic=False):
        super().__init__()
        self.ros_node = ros_node
        self.use_ros_topic = use_ros_topic
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._bridge = CvBridge()

        if use_ros_topic:
            # Subscribe to ROS2 camera topic
            self.ros_node.create_subscription(
                CompressedImage,
                '/camera/image_raw/compressed',
                self._ros_image_callback,
                10
            )
            self.ros_node.get_logger().info("CameraTrack: using ROS topic /camera/image_raw")
        else:
            # Direct V4L2 capture
            self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.ros_node.get_logger().info("CameraTrack: using /dev/video0 directly")

    def _ros_image_callback(self, msg: CompressedImage):
        """
        CompressedImage.data is a JPEG/PNG byte array.
        np.frombuffer + imdecode handles it without cv_bridge.
        """
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # returns BGR
        if frame is not None:
            with self._frame_lock:
                self._latest_frame = frame

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        if self.use_ros_topic:
            # Get latest frame from ROS topic
            with self._frame_lock:
                frame = self._latest_frame
            if frame is None:
                # No frame yet — return blank
                import numpy as np
                frame = np.zeros((480, 640, 3), dtype='uint8')
        else:
            ret, frame = self.cap.read()
            if not ret:
                return
            frame = cv2.resize(frame, (640, 480))

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        if self.ros_node:
            self.ros_node.get_logger().info("Releasing camera...")
        if not self.use_ros_topic and hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().stop()


# ─── ROS2 Node ────────────────────────────────────────────────────────────────
class WebRTCServerNode(Node):

    def __init__(self):
        super().__init__('webrtc_server')

        # ROS2 parameters — set from launch file or CLI
        self.declare_parameter('port', 8080)
        self.declare_parameter('use_ros_topic', True)

        self.port = self.get_parameter('port').value
        self.use_ros_topic = self.get_parameter('use_ros_topic').value

        # Publisher: broadcasts connection events to ROS2 ecosystem
        self.status_pub = self.create_publisher(String, '/webrtc/status', 10)

        self.get_logger().info(f"WebRTC server node initialized (port={self.port})")

        # Start aiohttp in a background thread with its own event loop
        self._loop = asyncio.new_event_loop()
        self._server_thread = threading.Thread(
            target=self._run_server,
            daemon=True
        )
        self._server_thread.start()

    def _run_server(self):
        """Runs in a background thread — owns the asyncio event loop."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._start_aiohttp())

    async def _start_aiohttp(self):
        app = web.Application()
        app.router.add_post("/offer", self._offer_handler)
        app.on_shutdown.append(self._on_shutdown)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', self.port)
        await site.start()

        self.get_logger().info(f"WebRTC HTTP server listening on port {self.port}")

        # Keep running until node shuts down
        while rclpy.ok():
            await asyncio.sleep(0.5)

        await runner.cleanup()

    async def _offer_handler(self, request):
        try:
            self.get_logger().info("Offer received")
            params = await request.json()

            pc = RTCPeerConnection()
            pcs.add(pc)

            camera_track = CameraTrack(
                ros_node=self,
                use_ros_topic=self.use_ros_topic
            )
            pc.addTrack(camera_track)

            self._publish_status("client_connected")

            @pc.on("iceconnectionstatechange")
            async def on_ice_state_change():
                self.get_logger().info(f"ICE state: {pc.iceConnectionState}")
                if pc.iceConnectionState in ["failed", "disconnected", "closed"]:
                    self.get_logger().info("ICE disconnected, cleaning up...")
                    camera_track.stop()
                    await pc.close()
                    pcs.discard(pc)
                    self._publish_status("client_disconnected")

            @pc.on("connectionstatechange")
            async def on_connection_state_change():
                self.get_logger().info(f"Connection state: {pc.connectionState}")
                if pc.connectionState in ["failed", "disconnected", "closed"]:
                    camera_track.stop()
                    await pc.close()
                    pcs.discard(pc)
                    self._publish_status("client_disconnected")

            offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)

            return web.json_response({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            })

        except Exception as e:
            self.get_logger().error(f"Offer error: {e}")
            return web.Response(text=str(e), status=500)

    async def _on_shutdown(self, app):
        self.get_logger().info("Shutting down, releasing all cameras...")
        coros = [pc.close() for pc in list(pcs)]
        await asyncio.gather(*coros, return_exceptions=True)
        pcs.clear()

    def _publish_status(self, status: str):
        """Publish connection status to ROS2 topic."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status published: {status}")

    def destroy_node(self):
        """Called on ROS2 shutdown — cleanly close all WebRTC connections."""
        self.get_logger().info("Node destroying, closing all peer connections...")
        if self._loop.is_running():
            future = asyncio.run_coroutine_threadsafe(
                self._on_shutdown(None),
                self._loop
            )
            future.result(timeout=5)
        super().destroy_node()


# ─── Entry point ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = WebRTCServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()