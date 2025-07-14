#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <string>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MuJoCoVisualizer : public rclcpp::Node
{
public:
    MuJoCoVisualizer() : Node("mujoco_visualizer")
    {
        // Initialize MuJoCo
        if (!initializeMuJoCo()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MuJoCo");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize GLFW
        if (!glfwInit()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLFW");
            rclcpp::shutdown();
            return;
        }
        
        // Create window
        window_ = glfwCreateWindow(1200, 900, "Grasp Simulation", NULL, NULL);
        if (!window_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create window");
            glfwTerminate();
            rclcpp::shutdown();
            return;
        }
        
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        
        // Initialize visualization
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);
        
        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);
        
        // Set camera
        cam_.distance = 1.5;
        cam_.elevation = -20;
        cam_.azimuth = 90;
        
        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&MuJoCoVisualizer::jointStateCallback, this, std::placeholders::_1));
        
        // Timer for rendering
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(16),  // ~60 FPS
            std::bind(&MuJoCoVisualizer::render, this));
        
        RCLCPP_INFO(this->get_logger(), "MuJoCo Visualizer initialized");
    }
    
    ~MuJoCoVisualizer()
    {
        // Clean up
        mjv_freeScene(&scn_);
        mjr_freeContext(&con_);
        
        if (m_) mj_deleteModel(m_);
        if (d_) mj_deleteData(d_);
        
        if (window_) {
            glfwDestroyWindow(window_);
        }
        glfwTerminate();
    }

private:
    // MuJoCo components
    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    
    // GLFW window
    GLFWwindow* window_ = nullptr;
    
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool initializeMuJoCo()
    {
        // Get the package share directory
        std::string pkg_dir;
        try {
            pkg_dir = ament_index_cpp::get_package_share_directory("grasp_simulation");
        } catch (const std::exception& e) {
            // If package not found, try local path
            pkg_dir = std::filesystem::current_path().string() + "/src/grasp_simulation";
        }
        
        // Construct model path
        std::string model_path = pkg_dir + "/models/grasp_scene.xml";
        
        RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path.c_str());
        
        // Check if file exists
        if (!std::filesystem::exists(model_path)) {
            RCLCPP_ERROR(this->get_logger(), "Model file not found at: %s", model_path.c_str());
            
            // Try alternative path
            model_path = "src/grasp_simulation/models/grasp_scene.xml";
            if (!std::filesystem::exists(model_path)) {
                RCLCPP_ERROR(this->get_logger(), "Model file not found at alternative path: %s", model_path.c_str());
                return false;
            }
        }
        
        // Load model
        char error[1000] = "Could not load model";
        m_ = mj_loadXML(model_path.c_str(), 0, error, 1000);
        
        if (!m_) {
            RCLCPP_ERROR(this->get_logger(), "Load model error: %s", error);
            return false;
        }
        
        // Create data
        d_ = mj_makeData(m_);
        
        // Initialize simulation
        mj_forward(m_, d_);
        
        RCLCPP_INFO(this->get_logger(), "MuJoCo model loaded successfully");
        return true;
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!d_) return;
        
        // Update joint positions from message
        for (size_t i = 0; i < msg->position.size() && i < 16; i++) {
            d_->qpos[7 + i] = msg->position[i];  // Skip object free joint
        }
        
        // Update physics
        mj_forward(m_, d_);
    }
    
    void render()
    {
        if (!window_ || !m_ || !d_) return;
        
        // Check if window should close
        if (glfwWindowShouldClose(window_)) {
            rclcpp::shutdown();
            return;
        }
        
        // Get framebuffer size
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);
        
        // Update scene
        mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
        
        // Render
        mjrRect viewport = {0, 0, width, height};
        mjr_render(viewport, &scn_, &con_);
        
        // Show frame
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MuJoCoVisualizer>());
    rclcpp::shutdown();
    return 0;
}