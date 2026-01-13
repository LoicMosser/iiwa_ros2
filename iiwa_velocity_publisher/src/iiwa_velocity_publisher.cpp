#include "iiwa_velocity_publisher/iiwa_velocity_publisher.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <rclcpp/logging.hpp>
#include <cmath> // pour std::sqrt

double norm(double x, double y, double z) {
    return std::sqrt(x * x + y * y + z * z);
}

IIWAEndEffectorVelocity::IIWAEndEffectorVelocity()
: Node("iiwa_ee_velocity") {
    // Paramètres
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("base_link", "iiwa_base");
    this->declare_parameter<std::string>("tip_link", "tool0");
    this->declare_parameter<double>("flow_rate_conversion", 1.0);
    this->declare_parameter<std::string>("publication_topic_name", "/end_eff_velocity");
    this->declare_parameter<std::string>("ee_velocity_topic_name", "/gpio_command_controller/commands");

    // Récupération des paramètres
    std::string robot_desc = this->get_parameter("robot_description").as_string();
    std::string topic_name = this->get_parameter("publication_topic_name").as_string();
    std::string gpio_topic_name = this->get_parameter("ee_velocity_topic_name").as_string();
    data_.base_link = this->get_parameter("base_link").as_string();
    data_.tip_link = this->get_parameter("tip_link").as_string();
    flow_rate_conv = this->get_parameter("flow_rate_conversion").as_double();

    // Parsing du modèle URDF
    if (!initKDL(robot_desc)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
        rclcpp::shutdown();
    }

    // Initialisation des solveurs KDL
    data_.jac_solver = std::make_shared<KDL::ChainJntToJacSolver>(data_.chain);
    data_.joint_positions.resize(data_.chain.getNrOfJoints(), 0.0);
    data_.joint_velocities.resize(data_.chain.getNrOfJoints(), 0.0);
    data_.jacobian.resize(data_.chain.getNrOfJoints());

    // Abonnement et publication
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&IIWAEndEffectorVelocity::jointStateCallback, this, std::placeholders::_1));
    ee_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name, 10);
    ee_norm_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(gpio_topic_name, 10);

    RCLCPP_INFO(this->get_logger(), "Node initialized");
}

KDL::Twist IIWAEndEffectorVelocity::multiplyJacobianByQDot(const KDL::Jacobian& jac, const KDL::JntArray& q_dot) {
    KDL::Twist result = KDL::Twist::Zero();
    for (int i = 0; i < jac.columns(); ++i) {
        for (int j = 0; j < 6; ++j) {
            result(j) += jac(j, i) * q_dot(i);
        }
    }
    return result;
}

bool IIWAEndEffectorVelocity::initKDL(const std::string& urdf_string) {
    urdf::Model model;
    if (!model.initString(urdf_string)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
        return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
        return false;
    }

    if (!tree.getChain(data_.base_link, data_.tip_link, data_.chain)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get chain from tree");
        return false;
    }
    return true;
}

void IIWAEndEffectorVelocity::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Vérifiez que le message contient des vitesses
    if (msg->velocity.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10, "No joint velocities received");
        return;
    }

    // Mise à jour des positions et vitesses articulaires
    for (size_t i = 0; i < msg->name.size() && i < data_.chain.getNrOfJoints(); ++i) {
        data_.joint_positions[i] = msg->position[i];
        data_.joint_velocities[i] = msg->velocity[i];
    }
    data_.joint_positions[0] = 0.0;
    data_.joint_velocities[0] = 0.0;

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    // "ee_position : \n %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f \n ee_velocity : \n %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f \n",
    // data_.joint_positions[0],
    // data_.joint_positions[1],
    // data_.joint_positions[2],
    // data_.joint_positions[3],
    // data_.joint_positions[4],
    // data_.joint_positions[5],
    // data_.joint_positions[6],
    // data_.joint_velocities[0],
    // data_.joint_velocities[1],
    // data_.joint_velocities[2],
    // data_.joint_velocities[3],
    // data_.joint_velocities[4],
    // data_.joint_velocities[5],
    // data_.joint_velocities[6]);

    // Conversion en KDL::JntArray
    KDL::JntArray q_kdl(data_.chain.getNrOfJoints());
    KDL::JntArray q_dot_kdl(data_.chain.getNrOfJoints());
    for (size_t i = 0; i < data_.chain.getNrOfJoints(); ++i) {
        q_kdl(i) = data_.joint_positions[i];
        q_dot_kdl(i) = data_.joint_velocities[i];
    }

    // Calcul du jacobien
    data_.jac_solver->JntToJac(q_kdl, data_.jacobian);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    // "jacobian : \n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f  \n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f \n",
    // data_.jacobian(0, 0),
    // data_.jacobian(1, 0),
    // data_.jacobian(2, 0),
    // data_.jacobian(3, 0),
    // data_.jacobian(4, 0),
    // data_.jacobian(5, 0),
    // data_.jacobian(0, 1),
    // data_.jacobian(1, 1),
    // data_.jacobian(2, 1),
    // data_.jacobian(3, 1),
    // data_.jacobian(4, 1),
    // data_.jacobian(5, 1),
    // data_.jacobian(0, 2),
    // data_.jacobian(1, 2),
    // data_.jacobian(2, 2),
    // data_.jacobian(3, 2),
    // data_.jacobian(4, 2),
    // data_.jacobian(5, 2),
    // data_.jacobian(0, 3),
    // data_.jacobian(1, 3),
    // data_.jacobian(2, 3),
    // data_.jacobian(3, 3),
    // data_.jacobian(4, 3),
    // data_.jacobian(5, 3),
    // data_.jacobian(0, 4),
    // data_.jacobian(1, 4),
    // data_.jacobian(2, 4),
    // data_.jacobian(3, 4),
    // data_.jacobian(4, 4),
    // data_.jacobian(5, 4),
    // data_.jacobian(0, 5),
    // data_.jacobian(1, 5),
    // data_.jacobian(2, 5),
    // data_.jacobian(3, 5),
    // data_.jacobian(4, 5),
    // data_.jacobian(5, 5),
    // data_.jacobian(0, 6),
    // data_.jacobian(1, 6),
    // data_.jacobian(2, 6),
    // data_.jacobian(3, 6),
    // data_.jacobian(4, 6),
    // data_.jacobian(5, 6));


    // Calcul de la vitesse de l'effecteur
    KDL::Twist ee_velocity = multiplyJacobianByQDot(data_.jacobian, q_dot_kdl);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    // "ee_velocity : \n%.3f, %.3f, %.3f, %.3f, %.3f, %.3f  \n",
    // ee_velocity.vel.x(),
    // ee_velocity.vel.y(),
    // ee_velocity.vel.z(),
    // ee_velocity.rot.x(),
    // ee_velocity.rot.y(),
    // ee_velocity.rot.z());

    // Publication sur /ee_velocity
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.twist.linear.x = ee_velocity.vel.x() * flow_rate_conv;
    twist_msg.twist.linear.y = ee_velocity.vel.y() * flow_rate_conv;
    twist_msg.twist.linear.z = ee_velocity.vel.z() * flow_rate_conv;
    twist_msg.twist.angular.x = ee_velocity.rot.x();
    twist_msg.twist.angular.y = ee_velocity.rot.y();
    twist_msg.twist.angular.z = ee_velocity.rot.z();
    ee_velocity_pub_->publish(twist_msg);

    auto velocity_msg = std_msgs::msg::Float64MultiArray();
    double norm_value = norm(ee_velocity.vel.x(),ee_velocity.vel.y(),ee_velocity.vel.z());
    velocity_msg.data = {1.0, 0.0, flow_rate_conv * norm_value};
    ee_norm_velocity_pub_->publish(velocity_msg);
    // Affichage (optionnel)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "End-effector linear velocity: [%.3f, %.3f, %.3f] m/s : %.3f m/s",
        ee_velocity.vel.x(), ee_velocity.vel.y(), ee_velocity.vel.z(), norm_value);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IIWAEndEffectorVelocity>());
    rclcpp::shutdown();
    return 0;
}
