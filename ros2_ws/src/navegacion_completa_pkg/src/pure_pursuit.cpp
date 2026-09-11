#include "/home/jose/Desktop/4RobUMA_ws/src/navegacion_completa_pkg/include/navegacion_completa_pkg/pure_pursuit.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PurePursuit::PurePursuit() : Node("pure_pursuit")
{
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PurePursuit::main_loop, this));


    rec_coordinate_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/planificador/pure_pursuit", 10, 
        [this](const geometry_msgs::msg::Point::SharedPtr msg)
        {
            // Asumiendo que inicializas N_INTERP = 10 en tu constructor
            if (x_trayectoria.empty()) {
                // Es el primer punto, no hay nada con qué interpolar
                x_trayectoria.push_back(msg->x);
                y_trayectoria.push_back(msg->y);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "PUNTO RECIBIDO");

            // Interpolar desde el último punto guardado hasta el nuevo punto recibido
            float x_prev = x_trayectoria.back();
            float y_prev = y_trayectoria.back();

            float dx = (msg->x - x_prev) / N_INTERP;
            float dy = (msg->y - y_prev) / N_INTERP;

            for (int i = 1; i <= N_INTERP; i++) {
                x_trayectoria.push_back(x_prev + i * dx);
                y_trayectoria.push_back(y_prev + i * dy);
            }

            ruta_terminada_ = false;
        });

    settings_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/settings/pure_pursuit", 10,
            std::bind(&PurePursuit::settings_callback, this, std::placeholders::_1)
        );


    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    rec_odom_ = this->create_subscription<geometry_msgs::msg::Point>(
        "posicion_estimada", 10, std::bind(&PurePursuit::odom_callback, this, _1));
}

PurePursuit::~PurePursuit()
{
    RCLCPP_INFO(this->get_logger(), "Apagando nodo pure_pursuit");
}

void PurePursuit::odom_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    current_x_ = msg->x;
    current_y_ = msg->y;
    theta_actual_ = msg->z;
}

void PurePursuit::settings_callback(const std_msgs::msg::String::SharedPtr msg) {
    try {
        json j = json::parse(msg->data);

        // Actualizar variables en memoria
        LOOKAHEAD_DIST = j["lookahead_distance"];
        V_LIN = j["linear_velocity"];
        N_INTERP = j["interpolated_points"];
        MAX_ANGULAR_VEL = j["max_angular_velocity"];
        MIN_ANGULAR_VEL = j["min_angular_velocity"];

        RCLCPP_INFO(this->get_logger(), "Nuevos parámetros de Pure Pursuit aplicados.");

    } catch (const json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "JSON inválido recibido: %s", e.what());
    }
}

int PurePursuit::find_nearest_point()
{
    float min_dist_ = 100000.0;
    int min_index_ = 0;
    
    // Cambiamos n_points_ por x_trayectoria.size()
    for(size_t i = 0; i < x_trayectoria.size(); i++)
    {
        float dist_ = sqrt(pow(x_trayectoria[i] - current_x_, 2) + pow(y_trayectoria[i] - current_y_, 2));
        if (dist_ < min_dist_)
        {
            min_dist_ = dist_;  // ¡Esta es la línea clave!
            min_index_ = i;
        }
    }

    return min_index_;
}

int PurePursuit::find_lookahead_point(int min_index_)
{
    for(size_t i = min_index_ + 1; i < x_trayectoria.size(); i++)
    {
        // ¡Importante! La distancia es desde la posición actual del robot
        float dist_ = sqrt(pow(x_trayectoria[i] - current_x_, 2) + pow(y_trayectoria[i] - current_y_, 2));
        
        if (dist_ >= LOOKAHEAD_DIST)
        {
            return i;
        }
    }
    // Si no encuentra ningún punto más lejos que Ld, apunta al último punto de la ruta
    return x_trayectoria.size() - 1; 
}

void PurePursuit::main_loop() 
{
    if (x_trayectoria.empty() || ruta_terminada_)
    {   
        publish_cmd(0.0, 0.0);
        return;
    } else {
        // --- PASO 0: Comprobar si hemos llegado a la meta ---
        float last_x = x_trayectoria.back(); // Último punto X de la ruta
        float last_y = y_trayectoria.back(); // Último punto Y de la ruta
        
        // Calculamos la distancia desde el robot hasta el punto final
        float dist_to_goal = sqrt(pow(last_x - current_x_, 2) + pow(last_y - current_y_, 2));
        
        // Tolerancia de llegada (ej. 0.15 metros = 15 cm). 
        // Si tu robot se pasa de largo, auméntala un poco (0.2).
        const float GOAL_TOLERANCE = 0.15; 

        if (dist_to_goal <= GOAL_TOLERANCE) {
            RCLCPP_INFO(this->get_logger(), "¡Meta alcanzada! Deteniendo el robot.");
            publish_cmd(0.0, 0.0);
            
            ruta_terminada_ = true;
            
            // Opcional pero recomendado: limpiamos la ruta para dejar el nodo en reposo 
            // perfecto hasta que el planificador mande una nueva
            x_trayectoria.clear();
            y_trayectoria.clear();
            return;
        }
        // ----------------------------------------------------

        // 1) Busco el punto de la trayectoria más cercano al robot
        int closest_index = find_nearest_point();

        // 2) Desde ese punto, busco el siguiente de la trayectoria a distancia >= LOOKAHEAD
        int lookahead_point_ = find_lookahead_point(closest_index);

        // 3) Transformar al frame del robot
        float dx = x_trayectoria[lookahead_point_] - current_x_;
        float dy = y_trayectoria[lookahead_point_] - current_y_;
        
        // Usamos solo y_rel para la curvatura del pure pursuit
        float y_rel = -sin(theta_actual_) * dx + cos(theta_actual_) * dy;

        // 4) Cálculo de curvatura
        float L_cuadrado = (dx * dx) + (dy * dy); 
        // Evitamos división por cero si el punto está exactamente debajo
        if (L_cuadrado < 0.0001) L_cuadrado = 0.0001; 
        float curv = (2 * y_rel) / L_cuadrado;

        // 5) Cálculo de w
        float w_cmd = curv * V_LIN;
        w_cmd = std::clamp(w_cmd, MIN_ANGULAR_VEL, MAX_ANGULAR_VEL);
        
        publish_cmd(V_LIN, w_cmd);
    }  
}

void PurePursuit::publish_cmd(float v, float w)
{
    geometry_msgs::msg::Twist msg;
    
    msg.linear.x = v;
    msg.angular.z = w;
    
    pub_vel_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pure_pursuit_node = std::make_shared<PurePursuit>();
    rclcpp::spin(pure_pursuit_node);
    rclcpp::shutdown();
    return 0;
}

// ros2 run navegacion_completa_pkg pure_pursuit