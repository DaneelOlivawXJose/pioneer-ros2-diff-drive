#include "/home/jose/Desktop/4RobUMA_ws/src/navegacion_completa_pkg/include/navegacion_completa_pkg/nav_manager.hpp"

using std::placeholders::_1;


// Acciones a realizar mas tarde
// CREAR NUEVO NODO QUE EJECUTE NAVEGACIÓN LOCAL
// Seleccion de método topológico desde react (añadir todo)
// Calibracion de sensores desde react


Navigator::Navigator() : Node("nav_manager")
{
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Navigator::main_loop, this));

    map_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/update/topological_map", 10,
        std::bind(&Navigator::map_callback, this, std::placeholders::_1)
    );

    // NUEVO: Suscriptor para la meta de la web y Publicador para el estado
    goal_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/topo_nav/goal", 10,
        std::bind(&Navigator::goal_callback, this, std::placeholders::_1)
    );

    current_node_id_ = "0";
    
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/topo_nav/status", 10);
    
    current_state = IDLE;
    RCLCPP_INFO(this->get_logger(), "Nodo nav_manager listo esperando metas por tópicos.");
}

Navigator::~Navigator()
{
    RCLCPP_INFO(this->get_logger(), "Apagando nodo navigation_node");
}

bool Navigator::node_is_valid(const std::string& node_str)
{
    // Si la matriz está vacía, no hay nodos válidos
    if (current_adj_matrix_.empty()) {
        return false;
    }

    try {
        // Convertimos el string a entero
        int node_id = std::stoi(node_str);
        int rows = current_adj_matrix_.size();

        // Comprobamos que el ID esté dentro de los límites de la matriz (0 a rows-1)
        if (node_id >= 0 && node_id < rows) {
            return true;
        } else {
            return false;
        }
    } catch (const std::exception& e) {
        // Si std::stoi falla (ej. node_str es "A" en vez de "1"), devolvemos falso
        RCLCPP_ERROR(this->get_logger(), "El ID del nodo no es un numero valido.");
        return false;
    }
}

void Navigator::map_callback(const std_msgs::msg::String::SharedPtr msg)
{
    try {
        // 1. Convertir el texto plano a un objeto JSON de C++
        json map_json = json::parse(msg->data);

        // Obtenemos la cantidad de nodos para preparar las matrices
        int num_nodes = map_json["nodes"].size();

        // 2. Limpiar y redimensionar las variables de la clase (FSM)
        nodes_dict_.clear();
        
        current_node_coords_.clear();
        current_node_coords_.resize(num_nodes);
        
        current_adj_matrix_.clear();
        // Creamos una matriz cuadrada num_nodes x num_nodes inicializada a 0.0
        current_adj_matrix_.assign(num_nodes, std::vector<double>(num_nodes, 0.0));

        // 3. Extraer los Nodos
        for (const auto& node : map_json["nodes"]) {
            std::string id_str = node["id"]; // Ejemplo: "0", "1", "2"...
            double x = node["x"];
            double y = node["y"];
            
            nodes_dict_[id_str] = {x, y};

            try {
                // Convertimos el ID string a entero para usarlo como índice en los vectores
                int id_idx = std::stoi(id_str);
                
                if (id_idx >= 0 && id_idx < num_nodes) {
                    current_node_coords_[id_idx] = {x, y};
                    RCLCPP_INFO(this->get_logger(), "Nodo cargado: [%s] en X:%.2f, Y:%.2f", id_str.c_str(), x, y);
                } else {
                    RCLCPP_WARN(this->get_logger(), "El ID de nodo (%d) está fuera del rango [0, %d]", id_idx, num_nodes - 1);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "El ID del nodo no es un entero válido: %s", id_str.c_str());
            }
        }

        // 4. Extraer las Aristas (Edges) y calcular distancias
        for (const auto& edge : map_json["edges"]) {
            // ¡OJO! Si tu JSON usa "from" y "to" en lugar de "source" y "target", cámbialo aquí abajo
            std::string source_str = edge["from"]; 
            std::string target_str = edge["to"];
            
            try {
                int u = std::stoi(source_str);
                int v = std::stoi(target_str);

                // Comprobamos que existan en la matriz
                if (u >= 0 && u < num_nodes && v >= 0 && v < num_nodes) {
                    
                    // Calculamos la distancia (peso) real de la arista para la matriz de adyacencia
                    double dx = current_node_coords_[u].x - current_node_coords_[v].x;
                    double dy = current_node_coords_[u].y - current_node_coords_[v].y;
                    double distance = std::hypot(dx, dy);

                    // Asignamos el peso en la matriz (asumiendo que los caminos son de doble sentido)
                    current_adj_matrix_[u][v] = distance;
                    current_adj_matrix_[v][u] = distance;
                    
                    RCLCPP_INFO(this->get_logger(), "Arista cargada: %s <---> %s (Distancia: %.2f m)", 
                                source_str.c_str(), target_str.c_str(), distance);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Los IDs de la arista no son válidos: %s - %s", source_str.c_str(), target_str.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "¡Mapa topológico parseado y matriz de adyacencia generada exitosamente!");

    } catch (const json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parseando el JSON del mapa: %s", e.what());
    }
}

void Navigator::goal_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Petición web recibida para ir al nodo: %s", msg->data.c_str());

    if (current_state != IDLE) {
        RCLCPP_WARN(this->get_logger(), "El robot ya está ocupado. Ignorando petición.");
        
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "ERROR: Robot ocupado";
        status_pub_->publish(status_msg);
        return;
    }

    if(node_is_valid(msg->data))
    {
        target_node_id_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Meta válida. Iniciando misión...");
        
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "ACCEPTED";
        status_pub_->publish(status_msg);
        
        // Arrancamos la máquina de estados
        current_state = PLANNING;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Nodo inválido.");
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "ERROR: Nodo inválido";
        status_pub_->publish(status_msg);
    }
}

/**
 * @brief Calcula la ruta más corta usando Dijkstra.
 * @param start_node_str ID del nodo origen en formato texto (ej. "0").
 * @param dest_node_str ID del nodo destino en formato texto (ej. "4").
 * @param adj_matrix Matriz cuadrada donde adj_matrix[i][j] es el peso/distancia. 0 o negativo = no hay camino.
 * @return std::vector<int> Secuencia de nodos a seguir. Vacío si no hay ruta.
 */
std::vector<int> calculate_dijkstra(
    const std::string& start_node_str,
    const std::string& dest_node_str,
    const std::vector<std::vector<double>>& adj_matrix)
{
    // 1. Convertir los strings a índices enteros
    int start = std::stoi(start_node_str);
    int dest = std::stoi(dest_node_str);
    int n = adj_matrix.size();

    // Validar seguridad
    if (start < 0 || start >= n || dest < 0 || dest >= n) {
        throw std::invalid_argument("Error: Nodos fuera del rango de la matriz.");
    }

    // 2. Inicializar distancias y registro de la ruta
    double INF = std::numeric_limits<double>::infinity();
    std::vector<double> distances(n, INF);
    std::vector<int> previous(n, -1); // Para rastrear de dónde venimos
    
    // Cola de prioridad (Min-Heap)
    std::priority_queue<NodeDist, std::vector<NodeDist>, std::greater<NodeDist>> pq;

    // Configurar nodo inicial
    distances[start] = 0.0;
    pq.push({start, 0.0});

    // 3. Bucle principal de Dijkstra
    while (!pq.empty()) {
        NodeDist current = pq.top();
        pq.pop();

        int u = current.id;

        // ¡Optimización! Salida temprana si ya hemos llegado al destino
        if (u == dest) break;

        // Descartar si encontramos un camino más corto previamente en la cola
        if (current.dist > distances[u]) continue;

        // Revisar todos los vecinos del nodo 'u'
        for (int v = 0; v < n; ++v) {
            double weight = adj_matrix[u][v];
            
            // Asumimos que un peso > 0 significa que hay conexión
            if (weight > 0) {
                double new_dist = distances[u] + weight;
                
                // Si encontramos un camino más corto hacia 'v'
                if (new_dist < distances[v]) {
                    distances[v] = new_dist;
                    previous[v] = u;
                    pq.push({v, new_dist});
                }
            }
        }
    }

    // 4. Reconstruir la ruta final
    std::vector<int> path;
    
    // Si la distancia al destino sigue siendo infinita, es inalcanzable
    if (distances[dest] == INF) {
        return path; // Retorna array vacío
    }

    // Navegar hacia atrás desde el destino hasta el origen usando el array 'previous'
    for (int at = dest; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    
    // Invertir el array para que vaya de Origen -> Destino
    std::reverse(path.begin(), path.end());

    return path;
}


/**
 * @brief Función heurística: Calcula la distancia euclidiana entre dos puntos.
 */
double calculate_heuristic(const Point2D& p1, const Point2D& p2) {
    // std::hypot calcula la hipotenusa: sqrt(dx^2 + dy^2) evitando desbordamientos
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

/**
 * @brief Calcula la ruta más corta usando el algoritmo A*.
 * @param start_node_str ID del nodo origen en formato texto.
 * @param dest_node_str ID del nodo destino en formato texto.
 * @param adj_matrix Matriz de pesos/distancias (0 = no hay camino).
 * @param node_coords Coordenadas (x,y) de cada nodo. El índice del array debe coincidir con la matriz.
 * @return std::vector<int> Secuencia de nodos a seguir. Vacío si no hay ruta.
 */
std::vector<int> calculate_astar(
    const std::string& start_node_str,
    const std::string& dest_node_str,
    const std::vector<std::vector<double>>& adj_matrix,
    const std::vector<Point2D>& node_coords)
{
    // Convertir strings a enteros
    int start = std::stoi(start_node_str);
    int dest = std::stoi(dest_node_str);
    int n = adj_matrix.size();

    // Validaciones de seguridad
    if (start < 0 || start >= n || dest < 0 || dest >= n) {
        throw std::invalid_argument("Error: Nodos fuera del rango.");
    }
    if (node_coords.size() != static_cast<size_t>(n)) {
        throw std::invalid_argument("Error: Faltan coordenadas. Debes proporcionar un Point2D por cada nodo.");
    }

    double INF = std::numeric_limits<double>::infinity();
    
    // Array para guardar el coste real recorrido desde el inicio: g(n)
    std::vector<double> g_score(n, INF);
    // Para reconstruir la ruta
    std::vector<int> previous(n, -1);
    
    // Cola de prioridad
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq;

    // Configurar nodo inicial
    g_score[start] = 0.0;
    double h_start = calculate_heuristic(node_coords[start], node_coords[dest]);
    
    // Insertamos: {id, f_cost, g_cost}
    pq.push({start, h_start, 0.0});

    while (!pq.empty()) {
        AStarNode current = pq.top();
        pq.pop();

        int u = current.id;

        // ¡Salida temprana! A* garantiza que si sacamos el destino de la cola, es el camino más corto
        if (u == dest) break;

        // Descartar si encontramos un camino real más corto hacia este nodo anteriormente
        if (current.g_cost > g_score[u]) continue;

        // Revisar vecinos
        for (int v = 0; v < n; ++v) {
            double weight = adj_matrix[u][v];
            
            if (weight > 0) { // Si hay conexión física
                // El nuevo coste real para llegar a 'v' a través de 'u'
                double tentative_g_score = g_score[u] + weight;
                
                // Si encontramos un mejor camino real hacia 'v'
                if (tentative_g_score < g_score[v]) {
                    g_score[v] = tentative_g_score;
                    previous[v] = u;
                    
                    // Calculamos la heurística desde el vecino 'v' hasta el destino
                    double h_score = calculate_heuristic(node_coords[v], node_coords[dest]);
                    
                    // f(n) = g(n) + h(n)
                    double f_score = tentative_g_score + h_score;
                    
                    pq.push({v, f_score, tentative_g_score});
                }
            }
        }
    }

    // Reconstruir la ruta
    std::vector<int> path;
    if (g_score[dest] == INF) {
        return path; // Retorna vacío si es inalcanzable
    }

    for (int at = dest; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}


// Gestiona toda la lógica de la FSM, es el callback del timer
void Navigator::main_loop()
{
    switch (current_state)
    {
    case IDLE:
        // Realmente no hace nada, espera a que la web mande el inicio
        break;

    case PLANNING:
    {
        // El action ha validado y se está ejecutando, calculamos la ruta (ej COCINA-BAÑO-SALÓN-DORMITORIO) y se la 
        // damos al nodo que ejecute la navegación local

        if (current_node_id_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error: No se conoce la posición actual del robot (current_node_id_ está vacío). Abortando.");
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "ERROR: Posicion desconocida";
            status_pub_->publish(status_msg);
            
            current_state = IDLE;
            break;
        }

        std::vector<int> calculated_route;

        // Aquí habría que distinguir entre A* o Dijkstra según seleccione el user
        switch (actual_method_)
        {
        case DIJKSTRA:
            calculated_route = calculate_dijkstra(current_node_id_, target_node_id_, current_adj_matrix_);
            break;

        case A_STAR:
            /* code */
            // Definir funcion
            calculated_route = calculate_astar(current_node_id_, target_node_id_, current_adj_matrix_, current_node_coords_);
            break;
        
        default:
            RCLCPP_WARN(this->get_logger(), "Método de planificación no definido.");
            break;
        }

        // Transicion de estado
        if (!calculated_route.empty()) 
        {
            RCLCPP_INFO(this->get_logger(), "Ruta encontrada con %zu nodos. Pasando a NAVIGATING.", calculated_route.size());
            
            // IMPORTANTE: Guardamos la ruta en una variable de la clase para que 
            // el estado NAVIGATING sepa qué ruta seguir.

            // Imprimo la ruta
            printf("[");
            for (int i = 0; i < calculated_route.size(); i++)
            {
                
                if (i == calculated_route.size() - 1)
                {
                    printf("%i]\n", calculated_route[i]);
                } else {
                    printf("%i, ", calculated_route[i]);
                }
            }
            this->current_route_ = calculated_route; 
            
            // Transición de estado
            current_state = NAVIGATING;
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Imposible calcular ruta. Abortando misión.");
            
            // Volvemos a reposo
            current_state = IDLE;
        }

        break;
    }
    
    case NAVIGATING:
        /* code */
        break;

    case ERROR:
        /* code */
        break;
    
    default:
        break;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto navigator_node = std::make_shared<Navigator>();
    rclcpp::spin(navigator_node);
    rclcpp::shutdown();
    return 0;
}

// ros2 run navegacion_completa_pkg nav_manager