#include "planificador_rutas/planificador.hpp"
#include <rclcpp/qos.hpp>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

Planificador::Planificador() : Node("planificador_node")
{
    // Publicadores y Suscriptores
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    rec_odom_ = this->create_subscription<geometry_msgs::msg::Point>(
        "posicion_estimada", 10, std::bind(&Planificador::Odometria_Callback, this, _1));

    timer_ = this->create_wall_timer(
        50ms, std::bind(&Planificador::algoritmo, this));

    rec_coordenada_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/planificador/ruta", 10, 
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
            x_trayectoria.push_back(msg->x);
            y_trayectoria.push_back(msg->y);
            // Si recibimos nuevos puntos y ya habíamos terminado, reactivamos
            if (ruta_terminada_) {
                ruta_terminada_ = false;
            }
        });

    pub_objetivo_ = this->create_publisher<geometry_msgs::msg::Point>("objetivo_ruta", 10);

    // Inicialización
    pos_x_global_ = 0.0;
    pos_y_global_ = 0.0;
    theta_actual_ = 0.0;
    distancia_acumulada_ = 0.0;
    
    first_read_ = true;
    sl_prev_ = 0;
    sr_prev_ = 0;

    indice_trayectoria_ = 0;
    ruta_terminada_ = false; 
    primer_objetivo = true;

    // Ganancia proporcional para el controlador de posición
    kp = 2.2;
    
    RCLCPP_INFO(this->get_logger(), "Nodo iniciado correctamente.");
}

Planificador::~Planificador()
{
    publish_cmd(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Apagando nodo.");
}

void Planificador::Odometria_Callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    pos_x_global_ = msg->x;
    pos_y_global_ = msg->y;
    theta_actual_ = msg->z;
}

void Planificador::algoritmo()
{
    // Si no hay ruta cargada, no hacemos nada
    if (x_trayectoria.empty() || ruta_terminada_) {
        publish_cmd(0.0, 0.0);
        return;
    }

    // Publicar objetivo actual para visualización
    geometry_msgs::msg::Point p_msg;
    p_msg.x = x_trayectoria[indice_trayectoria_];
    p_msg.y = y_trayectoria[indice_trayectoria_];
    p_msg.z = 0.0;
    pub_objetivo_->publish(p_msg);

    // Calcular distancia al objetivo actual (Euclídea)
    double dx = x_trayectoria[indice_trayectoria_] - pos_x_global_;
    double dy = y_trayectoria[indice_trayectoria_] - pos_y_global_;
    double distancia = std::sqrt(dx*dx + dy*dy);

    // Comprobar si hemos llegado (Umbral)
    // Recomendación: Si ves que da vueltas sobre el punto, aumenta un poco este umbral (ej. 0.1 a 0.2)
    if (distancia < UMBRAL_ERROR) 
    {
        indice_trayectoria_++;
        
        // Verificar si hemos terminado toda la ruta
        if (indice_trayectoria_ >= x_trayectoria.size())
        {
            RCLCPP_INFO(this->get_logger(), "Ruta completada.");
            ruta_terminada_ = true;
            publish_cmd(0.0, 0.0);
            return;
        }
    }

    // --- FOLLOW THE CARROT SUAVIZADO ---
    
    // 1. Calcular ángulo deseado hacia el objetivo
    double angulo_deseado = atan2(dy, dx);

    // 2. Calcular error angular
    double err_angulo = angulo_deseado - theta_actual_;

    // 3. Normalizar error angular (-PI a PI)
    while (err_angulo > M_PI) err_angulo -= 2 * M_PI;
    while (err_angulo < -M_PI) err_angulo += 2 * M_PI;

    // 4. Control P Suavizado para velocidad angular
    // Al haber bajado la kp a 0.5, la reacción será menos violenta.
    double w_cmd = kp * err_angulo; 

    // [MEJORA 1] Saturación de velocidad angular
    // Evita que el robot intente girar a velocidades imposibles si el error es grande (ej. 180 grados)
    double W_MAX = 0.5; // Rad/s máximo permitido (ajústalo según tu robot)
    if (w_cmd > W_MAX) w_cmd = W_MAX;
    if (w_cmd < -W_MAX) w_cmd = -W_MAX;

    // 5. Control de velocidad lineal Adaptativo (Sin saltos bruscos)
    // En lugar de un IF brusco, usamos una reducción proporcional.
    // Si el error es 0 rad, factor = 1.0 (Velocidad máxima).
    // Si el error es grande (ej. > 1 rad), factor baja rápidamente.
    
    // Fórmula gaussiana simple para suavidad extrema:
    // v = v_max * e^(-error^2)
    double factor_reduccion = std::exp(-1.0 * (err_angulo * err_angulo));
    
    // Aplicamos una velocidad mínima para que no se quede parado girando (opcional)
    double v_cmd = VELOCIDAD_LINEAL_MAX * factor_reduccion;
    
    if (v_cmd < 0.05) v_cmd = 0.05; // Velocidad mínima de arrastre si el error es gigante

    publish_cmd(v_cmd, w_cmd);
}

void Planificador::publish_cmd(float v, float w)
{
    geometry_msgs::msg::Twist msg;
    
    msg.linear.x = v;
    msg.angular.z = w;
    
    pub_vel_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nodo_planificador = std::make_shared<Planificador>();
    rclcpp::spin(nodo_planificador);
    rclcpp::shutdown();
    return 0;
}

// ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
// ros2 launch rosbridge_server rosbridge_websocket_launch.xml
// ros2 run planificador_rutas planificador_node