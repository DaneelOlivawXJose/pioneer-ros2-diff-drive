#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <cmath>

// Para usar sufijos como ms
using namespace std::chrono_literals;
using json = nlohmann::json;

// En IDLE simplemente espera a que se llame al action
// En Planing actúa como cliente de un servidor action que le devuelve la ruta de nodos (array)
// En NAVIGATING publica la ruta para el otro nodo local_controller y manda el feedback del action
//  * Detecta que se esté moviendo (on_recieve_odometry(), es la función principal del estado)
//  * Si el de antes detecta que no (on_collision_error())
// En ERROR cortamos ejecución y devolvemos error (error_feedback())
enum FSM_States {
    IDLE,
    PLANNING,
    NAVIGATING,
    ERROR
};

enum TOPOLOGICAL_METHOD {
    DIJKSTRA,
    A_STAR
};

// Estructura para Dijkstra
struct NodeDist {
    int id;
    double dist;
    
    // Sobrecarga del operador para que la cola de prioridad ordene de menor a mayor distancia
    bool operator>(const NodeDist& other) const {
        return dist > other.dist;
    }
};

struct Point2D {
    double x;
    double y;
};

// 2. Estructura para la cola de prioridad de A*
struct AStarNode {
    int id;
    double f_cost; // Coste total estimado: f(n) = g(n) + h(n) (se usa para ordenar)
    double g_cost; // Coste real desde el nodo inicial: g(n)
    
    // El Min-Heap ordenará los nodos por el menor f_cost
    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};


class Navigator : public rclcpp::Node
{
    public:
        Navigator();
        ~Navigator();
    private:
        std::string current_node_id_;
        std::string target_node_id_;

        FSM_States current_state;

        // Bucle principal de la FSM que gestiona las transiciones de fase y todo
        void main_loop();
        void on_recieve_odometry();
        void on_collision_error();
        bool node_is_valid(const std::string& node_str);
        void map_callback(const std_msgs::msg::String::SharedPtr msg);
        void goal_callback(const std_msgs::msg::String::SharedPtr msg);

        // Variables generales
        TOPOLOGICAL_METHOD actual_method_ = DIJKSTRA;
        std::vector<std::vector<double>> current_adj_matrix_;
        std::vector<int> current_route_;
        std::vector<Point2D> current_node_coords_;

        std::unordered_map<std::string, Point2D> nodes_dict_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

        // Timer para la FSM
        rclcpp::TimerBase::SharedPtr timer_;
};