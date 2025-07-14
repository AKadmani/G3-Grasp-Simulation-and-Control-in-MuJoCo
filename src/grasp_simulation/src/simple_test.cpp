cat > simple_test.cpp << 'EOF'
#include <rclcpp/rclcpp.hpp>
#include <mujoco/mujoco.h>
#include <iostream>
#include <fstream>
#include <cstdio>

class SimpleTest : public rclcpp::Node
{
public:
    SimpleTest() : Node("simple_test")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Simple Test Node");
        
        // Test MuJoCo
        const char* simple_xml = R"(
        <mujoco>
            <worldbody>
                <geom type="plane" size="1 1 0.1"/>
                <body pos="0 0 1">
                    <joint type="free"/>
                    <geom type="box" size=".1 .1 .1"/>
                </body>
            </worldbody>
        </mujoco>
        )";
        
        // Write to temp file
        std::ofstream file("temp_test.xml");
        file << simple_xml;
        file.close();
        
        char error[1000] = "";
        mjModel* m = mj_loadXML("temp_test.xml", NULL, error, 1000);
        
        if (!m) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load MuJoCo model: %s", error);
        } else {
            RCLCPP_INFO(this->get_logger(), "MuJoCo model loaded successfully!");
            RCLCPP_INFO(this->get_logger(), "Number of bodies: %d", m->nbody);
            mj_deleteModel(m);
        }
        
        std::remove("temp_test.xml");
        
        // Timer to keep node alive
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { RCLCPP_INFO(this->get_logger(), "Node is running..."); });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTest>());
    rclcpp::shutdown();
    return 0;
}
EOF