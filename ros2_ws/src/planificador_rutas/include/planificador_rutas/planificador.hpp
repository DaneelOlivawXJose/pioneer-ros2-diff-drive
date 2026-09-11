#ifndef PLANIFICADOR_HPP
#define PLANIFICADOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <cmath>
#include <vector>

// Definición de la estructura Coordenada
struct Coordenada {
    double x;
    double y;
};

class Planificador : public rclcpp::Node
{
public:
    Planificador();
    ~Planificador();

private:

    void algoritmo();
    void Odometria_Callback(const geometry_msgs::msg::Point::SharedPtr msg);

    void publish_cmd(float v, float w);

    // Variables de ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr rec_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr rec_coordenada_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_objetivo_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables de Odometría
    long sl_prev_;
    long sr_prev_;
    bool first_read_;
    
    double pos_x_global_;
    double pos_y_global_;
    double theta_actual_;
    double kp = 0.3;


    bool primer_objetivo = true;
    bool ruta_terminada_ = false;
    double distancia_acumulada_ = 0.0;
    // --- CONSTANTES ---
    const double R = 0.0347;         // Radio rueda (metros) - AJUSTAR
    const double TICKS_POR_VUELTA = 1452.0; // AJUSTAR
    // const double TICKS_POR_VUELTA = 2074;
    const double BASE_ROBOT = 0.2092;
    const double UMBRAL_ERROR = 0.02; // Umbral de error para comparaciones de distancia (metros)
    const double VELOCIDAD_LINEAL_MAX = 0.1;

    // La primera trayectoria es para el control de orientación (ejercicio 4)
    std::vector<double> x_trayectoria;
    std::vector<double> y_trayectoria;

    int indice_trayectoria_ = 0;
};

#endif // PLANIFICADOR_HPP