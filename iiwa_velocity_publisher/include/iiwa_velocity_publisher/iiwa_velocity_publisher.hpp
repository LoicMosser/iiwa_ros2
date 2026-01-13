#ifndef IIWA_END_EFFECTOR_VELOCITY_HPP_
#define IIWA_END_EFFECTOR_VELOCITY_HPP_

// Inclusions ROS2 et KDL
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <memory>
#include <vector>
#include <string>

// Forward declaration pour éviter les inclusions inutiles dans le header
namespace KDL {
    class ChainJntToJacSolver;
    class JntArray;
    class Twist;
}

// Structure pour stocker les données du robot
struct RobotData {
    KDL::Chain chain;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    KDL::Jacobian jacobian;
    std::string base_link;
    std::string tip_link;
};

class IIWAEndEffectorVelocity : public rclcpp::Node {
public:
    /**
     * @brief Constructeur du nœud ROS2 pour calculer la vitesse de l'effecteur.
     */
    IIWAEndEffectorVelocity();

private:
    /**
     * @brief Initialise la chaîne cinématique KDL à partir d'une description URDF.
     * @param urdf_string Description URDF du robot.
     * @return true si l'initialisation a réussi, false sinon.
     */
    bool initKDL(const std::string& urdf_string);

    /**
     * @brief Callback pour le topic /joint_states.
     * @param msg Message contenant les états articulaires.
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Multiplie un jacobien par un vecteur de vitesses articulaires.
     * @param jac Jacobien du robot.
     * @param q_dot Vecteur des vitesses articulaires.
     * @return Vitesse de l'effecteur (Twist).
     */
    static KDL::Twist multiplyJacobianByQDot(const KDL::Jacobian& jac, const KDL::JntArray& q_dot);

    // Données membres
    RobotData data_;
    double flow_rate_conv;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ee_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_norm_velocity_pub_;
};

#endif  // IIWA_END_EFFECTOR_VELOCITY_HPP_
