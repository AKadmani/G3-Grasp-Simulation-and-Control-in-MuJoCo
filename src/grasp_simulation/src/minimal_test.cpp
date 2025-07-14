cat > minimal_test.cpp << 'EOF'
#include <rclcpp/rclcpp.hpp>
#include <iostream>

class MinimalTest : public rclcpp::Node
{
public:
    MinimalTest() : Node("minimal_test")
    {
        RCLCPP_INFO(this->get_logger(), "MinimalTest node starting");
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { 
                counter_++;
                RCLCPP_INFO(this->get_logger(), "Timer callback %d", counter_); 
            });
            
        RCLCPP_INFO(this->get_logger(), "MinimalTest node initialized");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;
};

int main(int argc, char** argv)
{
    std::cout << "Starting main..." << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "ROS2 initialized" << std::endl;
    
    try {
        auto node = std::make_shared<MinimalTest>();
        std::cout << "Node created" << std::endl;
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    std::cout << "Shutdown complete" << std::endl;
    return 0;
}
EOF