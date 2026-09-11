#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <cstdlib>
#include <rclcpp/qos.hpp>
#include <thread>
#include <chrono>
#include <algorithm>

// Enum para saber qué algoritmo está operando actualmente
enum class NavAlgorithm {
    PROPORTIONAL,
    PURE_PURSUIT,
    MPC
};

class Master : public rclcpp::Node
{
public:
    Master();
    ~Master();

private:
    // Callbacks unificados desde la web
    void rutaCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void settingsCallback(const std_msgs::msg::String::SharedPtr msg);
    void setAlgorithmCallback(const std_msgs::msg::String::SharedPtr msg);

    // Función genérica para lanzar nodos
    void check_and_launch_node(const std::string& node_name, const std::string& launch_cmd);
    

    // Variable de estado que guarda el algoritmo activo (por defecto Pure Pursuit)
    NavAlgorithm current_algorithm_ = NavAlgorithm::PURE_PURSUIT;

    // Suscriptores únicos desde React
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_ruta_web_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_settings_web_;

    // --- PUBLICADORES: PROPORTIONAL (Planificador clásico) ---
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ruta_prop_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_settings_prop_;

    // --- PUBLICADORES: PURE PURSUIT ---
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ruta_pp_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_settings_pp_;

    // --- PUBLICADORES: MPC ---
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ruta_mpc_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_settings_mpc_;

    // 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_set_algorithm_;
};