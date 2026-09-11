#include <memory>
#include <chrono>
#include <algorithm>
#include <cmath> // Necesario para std::abs
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class JoyTeleopSmooth : public rclcpp::Node
{
public:
  JoyTeleopSmooth() : Node("joy_teleop_smooth")
  {
    // --- PARÁMETROS CONFIGURABLES ---

    // Velocidades Máximas
    this->declare_parameter("linear_max", 0.2);   // m/s
    this->declare_parameter("angular_max", 1.0);  // rad/s 

    // --- MODO DE CONDUCCIÓN ---
    // 0 = Normal (Omnidireccional / Diferencial normal)
    // 1 = Solo Recto (Ignora inputs de giro)
    this->declare_parameter("drive_mode", 0); 

    // Aceleración
    this->declare_parameter("linear_accel", 0.1);  // m/s^2 
    this->declare_parameter("angular_accel", 1.3); // rad/s^2

    // --- Frecuencia de Red ---
    this->declare_parameter("publish_rate", 30.0); 

    // --- Deadzone (Zona Muerta) ---
    this->declare_parameter("joy_deadzone", 0.07);

    // Configuración Ejes/Botones
    this->declare_parameter("axis_linear", 1);     // Eje Y
    this->declare_parameter("axis_angular", 0);    // Eje X
    this->declare_parameter("enable_button", 0);   // Botón Deadman
    this->declare_parameter("joy_timeout", 1.0);   // Tiempo antes de parar por seguridad

    // --- INICIALIZACIÓN ---
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyTeleopSmooth::joy_callback, this, _1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    double rate = this->get_parameter("publish_rate").as_double();
    std::chrono::duration<double> period(1.0 / rate);
    timer_ = this->create_wall_timer(period, std::bind(&JoyTeleopSmooth::control_loop, this));

    last_joy_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Teleop Suavizado: ON. Frecuencia: %.1f Hz. Deadzone activa.", rate);
  }

private:
  double target_lin_ = 0.0;
  double target_ang_ = 0.0;
  double current_lin_ = 0.0;
  double current_ang_ = 0.0;
  rclcpp::Time last_joy_time_;
  bool safety_pressed_ = false;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = this->now();

    int axis_lin = this->get_parameter("axis_linear").as_int();
    int axis_ang = this->get_parameter("axis_angular").as_int();
    int btn_idx = this->get_parameter("enable_button").as_int();
    double max_lin = this->get_parameter("linear_max").as_double();
    double max_ang = this->get_parameter("angular_max").as_double();
    double deadzone = this->get_parameter("joy_deadzone").as_double();
    
    // Leemos el flag de modo (0 o 1)
    int drive_mode = this->get_parameter("drive_mode").as_int();

    if (msg->buttons.size() > static_cast<size_t>(btn_idx) && msg->buttons[btn_idx] == 1) {
      safety_pressed_ = true;
      
      if (msg->axes.size() > static_cast<size_t>(axis_lin) && 
          msg->axes.size() > static_cast<size_t>(axis_ang)) {
        
        // Lectura cruda
        double raw_lin = msg->axes[axis_lin];
        double raw_ang = msg->axes[axis_ang]; // Restaurado lectura real

        // Aplicar Deadzone (Filtrado de ruido)
        if (std::abs(raw_lin) < deadzone) raw_lin = 0.0;
        if (std::abs(raw_ang) < deadzone) raw_ang = 0.0;

        target_lin_ = raw_lin * max_lin;

        // --- LÓGICA DEL FLAG ---
        if (drive_mode == 1) {
            // Si el flag es 1, forzamos el giro a 0 independientemente del joystick
            target_ang_ = 0.0;
        } else {
            // Si el flag es 0 (u otro), funcionamos normalmente
            target_ang_ = raw_ang * max_ang;
        }
      }
    } else {
      safety_pressed_ = false;
      target_lin_ = 0.0;
      target_ang_ = 0.0;
    }
  }

  void control_loop()
  {
    double dt = 1.0 / this->get_parameter("publish_rate").as_double();
    double timeout = this->get_parameter("joy_timeout").as_double();

    // Watchdog
    if ((this->now() - last_joy_time_).seconds() > timeout) {
      target_lin_ = 0.0;
      target_ang_ = 0.0;
    }

    // Suavizado (Rampas)
    double acc_lin = this->get_parameter("linear_accel").as_double();
    double acc_ang = this->get_parameter("angular_accel").as_double();

    current_lin_ = smooth_value(current_lin_, target_lin_, acc_lin, dt);
    
    // Restaurado el suavizado angular.
    // Si target_ang_ es 0 (porque drive_mode=1), esto suavizará hasta parar el giro.
    current_ang_ = smooth_value(current_ang_, target_ang_, acc_ang, dt); 

    // Optimización de red
    if (std::abs(current_lin_) < 0.001 && std::abs(current_ang_) < 0.001 && 
        std::abs(target_lin_) < 0.001 && std::abs(target_ang_) < 0.001) {
        current_lin_ = 0.0;
        current_ang_ = 0.0;
    }

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = current_lin_;
    msg.angular.z = current_ang_;
    vel_pub_->publish(msg);
  }

  double smooth_value(double current, double target, double accel, double dt)
  {
    double step = accel * dt;
    double diff = target - current;

    if (std::abs(diff) <= step) return target;
    return current + (step * (diff > 0 ? 1.0 : -1.0));
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleopSmooth>());
  rclcpp::shutdown();
  return 0;
}

// PREVIO
// ejecutar ds4windows
// usbipd list --> usbipd attach --wsl --busid <BUS_ID>
// sudo chmod 666 /dev/input/event0
// python3 driver_joystick_wsl.py


// ros2 run joy_controller joy_controller
// ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
// C:\Users\34601\Desktop\2025_2026\primer_cuatri\LabRobotica>py puente_udp.py