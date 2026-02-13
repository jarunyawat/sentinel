#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <boost/asio.hpp>
#include <thread>
#include <iostream>
#include <memory>

// JSON
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
using json = nlohmann::json;

class TcpCmdVelServer : public rclcpp::Node
{
public:
    TcpCmdVelServer()
    : Node("tcp_cmd_vel_server"),
      io_context_(),
      acceptor_(io_context_, tcp::endpoint(tcp::v4(), 5000))
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "TCP Server started on port 5000");

        server_thread_ = std::thread([this]() {
            run_server();
        });
    }

    ~TcpCmdVelServer()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down TCP server...");

        try
        {
            acceptor_.close();
            io_context_.stop();
        }
        catch (...) {}

        if (server_thread_.joinable())
            server_thread_.join();

        RCLCPP_INFO(this->get_logger(), "TCP Server shutdown complete");
    }

private:

    // ================= SERVER LOOP =================
    void run_server()
    {
        while (rclcpp::ok())
        {
            try
            {
                tcp::socket socket(io_context_);

                RCLCPP_INFO(this->get_logger(), "Waiting for client...");
                acceptor_.accept(socket);

                if (!rclcpp::ok()) break;

                RCLCPP_INFO(this->get_logger(), "Client connected");

                handle_client(std::move(socket));
            }
            catch (std::exception &e)
            {
                if (!rclcpp::ok()) break;

                RCLCPP_ERROR(this->get_logger(),
                    "Accept error: %s", e.what());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Server thread exited");
    }

    // ================= CLIENT LOOP =================
    void handle_client(tcp::socket socket)
    {
        try
        {
            boost::asio::streambuf buf;

            while (rclcpp::ok())
            {
                // Read until CRLF
                boost::asio::read_until(socket, buf, "\r\n");

                std::istream is(&buf);
                std::string line;
                std::getline(is, line);

                if (line.empty()) continue;

                parse_and_publish(line);
            }
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Client disconnected");
        }
    }

    // ================= JSON PARSE =================
    void parse_and_publish(const std::string &msg)
    {
        try
        {
            auto j = json::parse(msg);

            double vx = j.value("vx", 0.0);
            double vy = j.value("vy", 0.0);
            double wz = j.value("wz", 0.0);

            geometry_msgs::msg::Twist twist;
            twist.linear.x = vx;
            twist.linear.y = vy;
            twist.angular.z = wz;

            cmd_pub_->publish(twist);
        }
        catch (std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(),
                "JSON parse error: %s", e.what());
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    boost::asio::io_context io_context_;
    tcp::acceptor acceptor_;

    std::thread server_thread_;
};

// ================= MAIN =================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TcpCmdVelServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
