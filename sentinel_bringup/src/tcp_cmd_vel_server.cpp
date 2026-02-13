#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <boost/asio.hpp>
#include <thread>
#include <sstream>

using boost::asio::ip::tcp;

class TcpCmdVelServer : public rclcpp::Node
{
public:
    TcpCmdVelServer() : Node("tcp_cmd_vel_server"), acceptor_(io_context_, tcp::endpoint(tcp::v4(), 5000))
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
        io_context_.stop();
        if (server_thread_.joinable())
            server_thread_.join();
    }

private:
    void run_server()
    {
        while (rclcpp::ok())
        {
            try
            {
                tcp::socket socket(io_context_);
                acceptor_.accept(socket);

                RCLCPP_INFO(this->get_logger(), "Client connected");

                handle_client(std::move(socket));
            }
            catch (std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "Server error: %s", e.what());
            }
        }
    }

    void handle_client(tcp::socket socket)
    {
        try
        {
            boost::asio::streambuf buf;

            while (rclcpp::ok())
            {
                boost::asio::read_until(socket, buf, "\n");

                std::istream is(&buf);
                std::string line;
                std::getline(is, line);

                parse_and_publish(line);
            }
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Client disconnected");
        }
    }

    void parse_and_publish(const std::string &msg)
    {
        // Expect format: "linear_x angular_z"
        std::stringstream ss(msg);

        double linear = 0.0;
        double angular = 0.0;

        ss >> linear >> angular;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear;
        twist.angular.z = angular;

        cmd_pub_->publish(twist);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    boost::asio::io_context io_context_;
    tcp::acceptor acceptor_;

    std::thread server_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpCmdVelServer>());
    rclcpp::shutdown();
    return 0;
}
