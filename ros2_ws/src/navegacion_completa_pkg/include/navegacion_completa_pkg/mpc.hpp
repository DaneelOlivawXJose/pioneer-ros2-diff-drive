#ifndef MPC_HPP
#define MPC_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

using json = nlohmann::json;

class MPC_Controller : public rclcpp::Node
{
    public:
        MPC_Controller();
        ~MPC_Controller();
        
    private:
        void main_loop();
        void odom_callback(const geometry_msgs::msg::Point::SharedPtr msg);
        int find_nearest_point();
        void publish_cmd(float v, float w);
        void settings_callback(const std_msgs::msg::String::SharedPtr msg);
        
        Eigen::Vector2d solve_mpc(Eigen::Vector3d x_tilde, Eigen::Matrix3d A, Eigen::Matrix<double, 3, 2> B);

        // Suscriptores y Publicadores
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr rec_coordinate_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr rec_odom_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr settings_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Variables de configuración MPC (Cargadas vía JSON)
        float V_LIN = 0.1;
        float MAX_ANGULAR_VEL = 0.9;
        float MIN_ANGULAR_VEL = -0.9;
        int N_INTERP = 7;
        
        // Parámetros de Optimización
        int N_HORIZONTE = 12;
        float DT = 0.05;
        float Q_POS = 10.0;   // Peso para error en X e Y
        float Q_THETA = 2.0;  // Peso para error en ángulo
        float R_EFFORT = 0.1; // Peso para castigar el uso excesivo de comandos (V, W)

        // Estado actual del robot
        float current_x_ = 0.0;
        float current_y_ = 0.0;
        float theta_actual_ = 0.0;

        // Trayectoria
        std::vector<double> x_trayectoria;
        std::vector<double> y_trayectoria;
        bool ruta_terminada_ = true;
};

#endif