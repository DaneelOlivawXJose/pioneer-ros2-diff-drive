#include "/home/jose/Desktop/4RobUMA_ws/src/master_esp32/include/master.hpp"

using std::placeholders::_1;
using json = nlohmann::json;

Master::Master() : Node("master_node")
{
    // Suscriptores desde React
    sub_ruta_web_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/web/ruta", 10, std::bind(&Master::rutaCallback, this, _1));
        
    sub_settings_web_ = this->create_subscription<std_msgs::msg::String>(
        "/web/settings/algoritmo", 10, std::bind(&Master::settingsCallback, this, _1));

    sub_set_algorithm_ = this->create_subscription<std_msgs::msg::String>(
        "/set/algorithm", 10, std::bind(&Master::setAlgorithmCallback, this, _1));

    // Publicadores hacia los distintos algoritmos
    // PROPORTIONAL
    pub_ruta_prop_ = this->create_publisher<geometry_msgs::msg::Point>("/planificador/ruta_prop", 10);
    pub_settings_prop_ = this->create_publisher<std_msgs::msg::String>("/settings/proportional", 10);

    // PURE PURSUIT
    pub_ruta_pp_ = this->create_publisher<geometry_msgs::msg::Point>("/planificador/pure_pursuit", 10);
    pub_settings_pp_ = this->create_publisher<std_msgs::msg::String>("/settings/pure_pursuit", 10);

    // MPC
    pub_ruta_mpc_ = this->create_publisher<geometry_msgs::msg::Point>("/planificador/mpc_controller", 10);
    pub_settings_mpc_ = this->create_publisher<std_msgs::msg::String>("/settings/mpc_controller", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo Master (Multiplexor) iniciado");
}

Master::~Master()
{
    RCLCPP_INFO(this->get_logger(), "Apagando nodo Master. Matando todos los algoritmos...");
    std::system("pkill -f 'planificador_node'"); 
    std::system("pkill -f 'pure_pursuit'"); 
    std::system("pkill -f 'mpc_node'"); 
    RCLCPP_INFO(this->get_logger(), "Nodos hijos cerrados correctamente.");
}

void Master::check_and_launch_node(const std::string& node_name, const std::string& launch_cmd)
{
    auto nodos_activos = this->get_node_names();
    bool esta_corriendo = std::find(nodos_activos.begin(), nodos_activos.end(), node_name) != nodos_activos.end();

    if (!esta_corriendo)
    {
        RCLCPP_INFO(this->get_logger(), "El algoritmo '%s' NO está corriendo. Lanzándolo...", node_name.c_str());
        std::system((launch_cmd + " &").c_str());
        
        // Damos tiempo a que se inicie y se suscriba
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void Master::setAlgorithmCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string alg = msg->data; // "CARROT", "PURE_PURSUIT" o "MPC"

    if (alg == "CARROT" || alg == "PROPORTIONAL") {
        current_algorithm_ = NavAlgorithm::PROPORTIONAL;
        RCLCPP_INFO(this->get_logger(), "Modo de Navegación fijado a: PROPORTIONAL");

    } else if (alg == "PURE_PURSUIT") {
        current_algorithm_ = NavAlgorithm::PURE_PURSUIT;
        RCLCPP_INFO(this->get_logger(), "Modo de Navegación fijado a: PURE PURSUIT");

    } else if (alg == "MPC") {
        current_algorithm_ = NavAlgorithm::MPC;
        RCLCPP_INFO(this->get_logger(), "Modo de Navegación fijado a: MPC");

    } else {
        RCLCPP_WARN(this->get_logger(), "Algoritmo '%s' no reconocido.", alg.c_str());
    }
}

// CALLBACK DE RUTAS
void Master::rutaCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr active_pub;
    std::string alg_name;

    switch (current_algorithm_)
    {
        case NavAlgorithm::PROPORTIONAL:
            check_and_launch_node("planificador_node", "ros2 run planificador_rutas planificador_node");
            active_pub = pub_ruta_prop_;
            alg_name = "PROPORTIONAL";
            break;

        case NavAlgorithm::PURE_PURSUIT:
            check_and_launch_node("pure_pursuit", "ros2 run navegacion_completa_pkg pure_pursuit");
            active_pub = pub_ruta_pp_;
            alg_name = "PURE PURSUIT";
            break;

        case NavAlgorithm::MPC:
            check_and_launch_node("mpc_node", "ros2 run navegacion_completa_pkg mpc_controller");
            active_pub = pub_ruta_mpc_;
            alg_name = "MPC";
            break;
    }

    // Esperamos 2 segundos (20 x 100ms) a que el suscriptor se conecte
    int retries = 0;
    while (active_pub->get_subscription_count() == 0 && retries < 20) {
        RCLCPP_WARN(this->get_logger(), "Esperando a enlazar con %s...", alg_name.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retries++;
    }

    if (active_pub->get_subscription_count() > 0) {
        active_pub->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Punto reenviado correctamente a %s.", alg_name.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Fallo crítico: No se pudo enlazar con %s. El punto se ha perdido.", alg_name.c_str());
    }
}

// CALLBACK DE SETTINGS
void Master::settingsCallback(const std_msgs::msg::String::SharedPtr msg)
{
    try {
        json j = json::parse(msg->data);
        std::string alg = j.value("algorithm", "");

        if (alg == "CARROT" || alg == "PROPORTIONAL") {
            current_algorithm_ = NavAlgorithm::PROPORTIONAL;
            check_and_launch_node("planificador_node", "ros2 run planificador_rutas planificador_node");
            pub_settings_prop_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Ajustes aplicados al planificador PROPORTIONAL.");

        } else if (alg == "PURE_PURSUIT") {
            current_algorithm_ = NavAlgorithm::PURE_PURSUIT;
            check_and_launch_node("pure_pursuit", "ros2 run navegacion_completa_pkg pure_pursuit");
            pub_settings_pp_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Ajustes aplicados a PURE PURSUIT.");

        } else if (alg == "MPC") {
            current_algorithm_ = NavAlgorithm::MPC;
            check_and_launch_node("mpc_node", "ros2 run navegacion_completa_pkg mpc_controller");
            pub_settings_mpc_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Ajustes aplicados a MPC.");

        } else {
            RCLCPP_WARN(this->get_logger(), "Algoritmo '%s' no reconocido en los settings.", alg.c_str());
        }

    } catch (const json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parseando JSON de settings: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Master>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
// ros2 launch rosbridge_server rosbridge_websocket_launch.xml
// ros2 run master_esp32 master_node