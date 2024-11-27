#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Master : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_test;

    Master() : Node("master")
    {
        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_tim_50hz, this));

        //----Publisher
        pub_test = this->create_publisher<std_msgs::msg::String>("chatter", 1);
    }

    void callback_tim_50hz()
    {
        static int count = 0;
        std::string message = "Hello, World! " + std::to_string(count);
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());

        std_msgs::msg::String msg;
        msg.data = message;
        pub_test->publish(msg);

        count++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_master = std::make_shared<Master>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_master);
    executor.spin();

    return 0;
}