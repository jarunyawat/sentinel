#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting camera node...");

        // Open camera with V4L2 backend
        cap_.open("/dev/video0", cv::CAP_V4L2);

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            rclcpp::shutdown();
            return;
        }

        // FORCE MJPG (CRITICAL)
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

        // Set resolution
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 2592);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1944);

        // Set FPS
        cap_.set(cv::CAP_PROP_FPS, 30);

        // Print actual camera settings
        RCLCPP_INFO(this->get_logger(), "Width: %.0f",
                    cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        RCLCPP_INFO(this->get_logger(), "Height: %.0f",
                    cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        RCLCPP_INFO(this->get_logger(), "FPS: %.0f",
                    cap_.get(cv::CAP_PROP_FPS));

        // Publisher
        publisher_ = image_transport::create_publisher(this, "camera/image_raw");

        // Timer for capture loop (~30Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraNode::capture_frame, this));
    }

    ~CameraNode()
    {
        cap_.release();
        cv::destroyAllWindows();
    }

private:
    void capture_frame()
    {
        cv::Mat frame;
        if (!cap_.read(frame))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame");
            return;
        }

        // Convert to ROS Image message
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame).toImageMsg();

        msg->header.stamp = this->now();

        publisher_.publish(msg);
    }

    cv::VideoCapture cap_;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}