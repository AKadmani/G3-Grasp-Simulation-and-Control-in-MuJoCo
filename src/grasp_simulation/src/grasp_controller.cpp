#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

class GraspController : public rclcpp::Node
{
public:
    GraspController() : Node("grasp_controller")
    {
        // Initialize MuJoCo
        initializeMuJoCo();
        
        // Initialize ROS2 components
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_commands", 10, 
            std::bind(&GraspController::jointCommandCallback, this, std::placeholders::_1));
        
        // Timer for simulation step
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&GraspController::simulationStep, this));
        
        // Initialize grasp parameters
        initializeGraspParameters();
        
        RCLCPP_INFO(this->get_logger(), "Grasp Controller initialized");
    }
    
    ~GraspController()
    {
        if (m) mj_deleteModel(m);
        if (d) mj_deleteData(d);
    }

private:
    // MuJoCo components
    mjModel* m = nullptr;
    mjData* d = nullptr;
    
    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Grasp parameters
    enum GraspType { POWER_GRASP, PRECISION_GRASP, LATERAL_GRASP };
    enum ContactModel { SOFT_FINGER, HARD_FINGER, FRICTIONLESS };
    
    GraspType grasp_type_ = POWER_GRASP;
    ContactModel contact_model_ = SOFT_FINGER;
    
    // Control parameters
    enum ControlMode { IMPEDANCE, PID_POSITION, HYBRID };
    ControlMode control_mode_ = IMPEDANCE;
    
    // Jacobian and Grasp matrices
    Eigen::MatrixXd J_;  // Jacobian matrix
    Eigen::MatrixXd G_;  // Grasp matrix
    
    // Impedance control parameters
    Eigen::VectorXd K_p_;  // Position gains
    Eigen::VectorXd K_d_;  // Damping gains
    Eigen::VectorXd desired_positions_;
    Eigen::VectorXd desired_velocities_;
    
    void initializeMuJoCo()
    {
        // Load model
        char error[1000] = "Could not load model";
        std::string model_path = "src/grasp_simulation/models/grasp_scene.xml";
        m = mj_loadXML(model_path.c_str(), 0, error, 1000);
        
        if (!m) {
            RCLCPP_ERROR(this->get_logger(), "Load model error: %s", error);
            return;
        }
        
        // Create data
        d = mj_makeData(m);
        
        // Initialize simulation
        mj_forward(m, d);
    }
    
    void initializeGraspParameters()
    {
        // Initialize control gains for 16 DOF Allegro hand
        K_p_ = Eigen::VectorXd::Constant(16, 10.0);
        K_d_ = Eigen::VectorXd::Constant(16, 2.0);
        
        desired_positions_ = Eigen::VectorXd::Zero(16);
        desired_velocities_ = Eigen::VectorXd::Zero(16);
        
        // Set initial hand configuration based on grasp type
        setInitialHandConfiguration();
    }
    
    void setInitialHandConfiguration()
    {
        switch(grasp_type_) {
            case POWER_GRASP:
                // Power grasp configuration - fingers wrap around object
                // Index finger
                desired_positions_[0] = 0.0;   // Abduction/adduction
                desired_positions_[1] = 0.3;   // MCP flexion
                desired_positions_[2] = 0.5;   // PIP flexion
                desired_positions_[3] = 0.3;   // DIP flexion
                
                // Middle finger
                desired_positions_[4] = 0.0;
                desired_positions_[5] = 0.3;
                desired_positions_[6] = 0.5;
                desired_positions_[7] = 0.3;
                
                // Ring finger
                desired_positions_[8] = 0.0;
                desired_positions_[9] = 0.3;
                desired_positions_[10] = 0.5;
                desired_positions_[11] = 0.3;
                
                // Thumb
                desired_positions_[12] = 0.7;  // Opposition
                desired_positions_[13] = 0.2;
                desired_positions_[14] = 0.3;
                desired_positions_[15] = 0.2;
                break;
                
            case PRECISION_GRASP:
                // Precision grasp - fingertips contact
                // Implement precision grasp configuration
                break;
                
            case LATERAL_GRASP:
                // Lateral grasp - thumb against side of index
                // Implement lateral grasp configuration
                break;
        }
    }
    
    void computeJacobian()
    {
        // Compute the hand Jacobian
        int nv = m->nv;
        J_ = Eigen::MatrixXd::Zero(6 * 4, nv);  // 4 contact points, 6 DOF each
        
        // For each fingertip contact point
        for (int i = 0; i < 4; i++) {
            // Get body ID for fingertip
            std::string tip_names[4] = {"if_tip", "mf_tip", "rf_tip", "th_tip"};
            int body_id = mj_name2id(m, mjOBJ_BODY, tip_names[i].c_str());
            
            if (body_id >= 0) {
                // Compute point Jacobian
                mjtNum jacp[3*nv], jacr[3*nv];
                mj_jacBody(m, d, jacp, jacr, body_id);
                
                // Fill Jacobian matrix
                for (int j = 0; j < nv; j++) {
                    J_(i*6 + 0, j) = jacp[j*3 + 0];
                    J_(i*6 + 1, j) = jacp[j*3 + 1];
                    J_(i*6 + 2, j) = jacp[j*3 + 2];
                    J_(i*6 + 3, j) = jacr[j*3 + 0];
                    J_(i*6 + 4, j) = jacr[j*3 + 1];
                    J_(i*6 + 5, j) = jacr[j*3 + 2];
                }
            }
        }
    }
    
    void computeGraspMatrix()
    {
        // Compute grasp matrix based on contact model
        G_ = Eigen::MatrixXd::Zero(6, 6 * 4);  // Object wrench to contact wrenches
        
        for (int i = 0; i < 4; i++) {
            // Get contact point and normal
            Eigen::Vector3d contact_point = getContactPoint(i);
            Eigen::Vector3d contact_normal = getContactNormal(i);
            
            // Rotation matrix from contact to world frame
            Eigen::Matrix3d R = computeContactFrame(contact_normal);
            
            switch(contact_model_) {
                case SOFT_FINGER:
                    // Soft finger can transmit forces and one moment
                    G_.block<3, 3>(0, i*6) = R;
                    G_.block<3, 3>(3, i*6) = skewSymmetric(contact_point) * R;
                    G_(5, i*6 + 5) = 1.0;  // Moment about normal
                    break;
                    
                case HARD_FINGER:
                    // Hard finger transmits only forces
                    G_.block<3, 3>(0, i*6) = R;
                    G_.block<3, 3>(3, i*6) = skewSymmetric(contact_point) * R;
                    break;
                    
                case FRICTIONLESS:
                    // Only normal force
                    G_.block<3, 1>(0, i*6) = contact_normal;
                    G_.block<3, 1>(3, i*6) = skewSymmetric(contact_point) * contact_normal;
                    break;
            }
        }
    }
    
    Eigen::Vector3d getContactPoint(int finger_idx)
    {
        // Get contact point position for finger
        // This is simplified - in reality would use collision detection
        std::string tip_names[4] = {"if_tip", "mf_tip", "rf_tip", "th_tip"};
        int body_id = mj_name2id(m, mjOBJ_BODY, tip_names[finger_idx].c_str());
        
        if (body_id >= 0) {
            return Eigen::Vector3d(d->xpos[3*body_id], 
                                   d->xpos[3*body_id + 1], 
                                   d->xpos[3*body_id + 2]);
        }
        return Eigen::Vector3d::Zero();
    }
    
    Eigen::Vector3d getContactNormal(int finger_idx)
    {
        // Get contact normal - simplified
        // In reality, would get from contact detection
        Eigen::Vector3d object_pos(d->qpos[0], d->qpos[1], d->qpos[2]);
        Eigen::Vector3d contact_pos = getContactPoint(finger_idx);
        Eigen::Vector3d normal = contact_pos - object_pos;
        normal.normalize();
        return normal;
    }
    
    Eigen::Matrix3d computeContactFrame(const Eigen::Vector3d& normal)
    {
        // Compute rotation matrix for contact frame
        Eigen::Vector3d z = normal;
        Eigen::Vector3d x = Eigen::Vector3d::UnitX();
        
        if (std::abs(z.dot(x)) > 0.9) {
            x = Eigen::Vector3d::UnitY();
        }
        
        Eigen::Vector3d y = z.cross(x).normalized();
        x = y.cross(z).normalized();
        
        Eigen::Matrix3d R;
        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = z;
        
        return R;
    }
    
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d S;
        S << 0, -v(2), v(1),
             v(2), 0, -v(0),
             -v(1), v(0), 0;
        return S;
    }
    
    void computeControl()
    {
        switch(control_mode_) {
            case IMPEDANCE:
                computeImpedanceControl();
                break;
            case PID_POSITION:
                computePIDControl();
                break;
            case HYBRID:
                computeHybridControl();
                break;
        }
    }
    
    void computeImpedanceControl()
    {
        // Get current joint positions and velocities
        Eigen::VectorXd q(16), qd(16);
        for (int i = 0; i < 16; i++) {
            q(i) = d->qpos[7 + i];  // Skip object free joint
            qd(i) = d->qvel[6 + i]; // Skip object velocities
        }
        
        // Compute position and velocity errors
        Eigen::VectorXd pos_error = desired_positions_ - q;
        Eigen::VectorXd vel_error = desired_velocities_ - qd;
        
        // Impedance control law: tau = K_p * pos_error + K_d * vel_error
        Eigen::VectorXd tau = K_p_.cwiseProduct(pos_error) + K_d_.cwiseProduct(vel_error);
        
        // Apply control torques
        for (int i = 0; i < 16; i++) {
            d->ctrl[i] = desired_positions_(i);  // Position actuators
        }
    }
    
    void computePIDControl()
    {
        // Similar to impedance but with integral term
        // Implementation here
    }
    
    void computeHybridControl()
    {
        // Hybrid position/force control
        // Implementation here
    }
    
    void simulationStep()
    {
        if (!m || !d) return;
        
        // Update grasp phase
        updateGraspPhase();
        
        // Compute Jacobian and Grasp matrix
        computeJacobian();
        computeGraspMatrix();
        
        // Compute control
        computeControl();
        
        // Step simulation
        mj_step(m, d);
        
        // Publish joint states
        publishJointStates();
        
        // Check grasp stability
        checkGraspStability();
    }
    
    void updateGraspPhase()
    {
        static int phase = 0;
        static int step_count = 0;
        
        step_count++;
        
        switch(phase) {
            case 0:  // Approach phase
                if (step_count < 500) {
                    // Move hand down towards object
                    // Gradually close fingers
                    for (int i = 0; i < 16; i++) {
                        desired_positions_(i) = desired_positions_(i) * (step_count / 500.0);
                    }
                } else {
                    phase = 1;
                    step_count = 0;
                }
                break;
                
            case 1:  // Grasp phase
                if (step_count < 300) {
                    // Maintain grasp
                } else {
                    phase = 2;
                    step_count = 0;
                }
                break;
                
            case 2:  // Lift phase
                if (step_count < 500) {
                    // Lift object by moving hand up
                    // This would typically involve moving the base
                } else {
                    phase = 3;
                }
                break;
                
            case 3:  // Hold phase
                // Maintain stable grasp
                break;
        }
    }
    
    void publishJointStates()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        
        // Allegro hand joint names
        std::vector<std::string> joint_names = {
            "if_joint_0", "if_joint_1", "if_joint_2", "if_joint_3",
            "mf_joint_0", "mf_joint_1", "mf_joint_2", "mf_joint_3",
            "rf_joint_0", "rf_joint_1", "rf_joint_2", "rf_joint_3",
            "th_joint_0", "th_joint_1", "th_joint_2", "th_joint_3"
        };
        
        msg.name = joint_names;
        msg.position.resize(16);
        msg.velocity.resize(16);
        msg.effort.resize(16);
        
        for (int i = 0; i < 16; i++) {
            msg.position[i] = d->qpos[7 + i];
            msg.velocity[i] = d->qvel[6 + i];
            msg.effort[i] = d->qfrc_actuator[i];
        }
        
        joint_state_pub_->publish(msg);
    }
    
    void checkGraspStability()
    {
        // Check if object is being grasped stably
        double object_height = d->qpos[2];  // z-position of object
        
        // Simple stability check - object should be lifted
        if (object_height > 0.6) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Grasp successful! Object lifted to height: %.3f", object_height);
        }
        
        // Check force closure
        checkForceClosure();
    }
    
    void checkForceClosure()
    {
        // Simplified force closure check
        // In reality, would solve: G * f = w for feasible f > 0
        
        // Check if grasp matrix has full rank
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(G_);
        double min_singular_value = svd.singularValues().minCoeff();
        
        if (min_singular_value > 0.01) {
            RCLCPP_DEBUG(this->get_logger(), "Force closure achieved. Min singular value: %.4f", min_singular_value);
        }
    }
    
    void jointCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 16) {
            for (int i = 0; i < 16; i++) {
                desired_positions_(i) = msg->data[i];
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraspController>());
    rclcpp::shutdown();
    return 0;
}