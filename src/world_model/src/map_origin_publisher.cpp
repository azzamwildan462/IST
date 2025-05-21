#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

class MapOriginPublisher : public rclcpp::Node
{
public:
    MapOriginPublisher() : Node("map_origin_publisher")
    {
        // Subscriber to /map
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&MapOriginPublisher::mapCallback, this, std::placeholders::_1));

        // Publisher to /map_origin
        origin_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/map_origin", 10);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        geometry_msgs::msg::Pose origin = msg->info.origin; // Extract the origin
        origin_pub_->publish(origin);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr origin_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOriginPublisher>());
    rclcpp::shutdown();
    return 0;
}
