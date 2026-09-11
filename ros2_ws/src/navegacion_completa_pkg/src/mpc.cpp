#include "navegacion_completa_pkg/mpc.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

MPC_Controller::MPC_Controller() : Node("mpc_node")
{
    timer_ = this->create_wall_timer(
      50ms, std::bind(&MPC_Controller::main_loop, this));

    rec_coordinate_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/planificador/mpc_controller", 10,  
        [this](const geometry_msgs::msg::Point::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "PUNTO RECIBIDO: X=%.2f, Y=%.2f", msg->x, msg->y);

            // Si es el primer punto, el inicio de la línea es donde está el robot ahora mismo
            if (x_trayectoria.empty()) {
                x_trayectoria.push_back(current_x_);
                y_trayectoria.push_back(current_y_);
            }

            float x_prev = x_trayectoria.back();
            float y_prev = y_trayectoria.back();
            
            // Prevención de división por cero
            int n_puntos = (N_INTERP > 0) ? N_INTERP : 1; 

            float dx = (msg->x - x_prev) / n_puntos;
            float dy = (msg->y - y_prev) / n_puntos;

            // Interpolar puntos
            for (int i = 1; i <= n_puntos; i++) {
                x_trayectoria.push_back(x_prev + i * dx);
                y_trayectoria.push_back(y_prev + i * dy);
            }

            // Activamos el main_loop quitando el return anticipado que tenías
            ruta_terminada_ = false; 
        });

    // Suscripción a la configuración del MPC
    settings_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/settings/mpc_controller", 10,
        std::bind(&MPC_Controller::settings_callback, this, std::placeholders::_1));

    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    rec_odom_ = this->create_subscription<geometry_msgs::msg::Point>(
        "posicion_estimada", 10, std::bind(&MPC_Controller::odom_callback, this, _1));
}

MPC_Controller::~MPC_Controller()
{
    RCLCPP_INFO(this->get_logger(), "Apagando nodo mpc_controller");
}

void MPC_Controller::odom_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    current_x_ = msg->x;
    current_y_ = msg->y;
    theta_actual_ = msg->z;
}

void MPC_Controller::settings_callback(const std_msgs::msg::String::SharedPtr msg) 
{
    try {
        json j = json::parse(msg->data);
        
        // Usamos j.value para evitar crashes si falta alguna clave
        N_HORIZONTE = j.value("horizon", 12);
        DT = j.value("dt", 0.05);
        Q_POS = j.value("weightPos", 10.0);
        Q_THETA = j.value("weightHeading", 2.0);
        R_EFFORT = j.value("weightEffort", 0.1);
        
        V_LIN = j.value("linear_velocity", 0.1);
        MAX_ANGULAR_VEL = j.value("max_angular_velocity", 0.9);
        MIN_ANGULAR_VEL = j.value("min_angular_velocity", -0.9);
        N_INTERP = j.value("interpolated_points", 7);

        RCLCPP_INFO(this->get_logger(), "Parámetros MPC aplicados -> Horizonte: %d, Q_Pos: %.2f", N_HORIZONTE, Q_POS);
    } catch (const json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "JSON inválido recibido: %s", e.what());
    }
}

int MPC_Controller::find_nearest_point()
{
    float min_dist_ = 100000.0;
    int min_index_ = 0;
    
    for(size_t i = 0; i < x_trayectoria.size(); i++)
    {
        float dist_ = sqrt(pow(x_trayectoria[i] - current_x_, 2) + pow(y_trayectoria[i] - current_y_, 2));
        if (dist_ < min_dist_)
        {
            min_dist_ = dist_;
            min_index_ = i;
        }
    }
    return min_index_;
}

Eigen::Vector2d MPC_Controller::solve_mpc(Eigen::Vector3d x_tilde, Eigen::Matrix3d A, Eigen::Matrix<double, 3, 2> B)
{
    int n_states = 3;   
    int n_controls = 2; 
    int N = N_HORIZONTE;

    // matrices Q y R con las variables de la clase
    Eigen::Matrix3d Q;
    Q << Q_POS, 0.0, 0.0,
         0.0, Q_POS, 0.0,
         0.0, 0.0, Q_THETA; 
         
    Eigen::Matrix2d R;
    R << R_EFFORT, 0.0,
         0.0, R_EFFORT;

    // istema
    Eigen::MatrixXd Phi(n_states * N, n_states);
    Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(n_states * N, n_controls * N);
    
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(n_states * N, n_states * N);
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_controls * N, n_controls * N);

    Eigen::Matrix3d A_power = A; 
    
    for (int i = 0; i < N; i++) 
    {
        Phi.block(i * n_states, 0, n_states, n_states) = A_power;

        for (int j = 0; j <= i; j++) 
        {
            if (i == j) {
                Gamma.block(i * n_states, j * n_controls, n_states, n_controls) = B;
            } else {
                Eigen::Matrix3d A_temp = Eigen::Matrix3d::Identity();
                for (int k = 0; k < (i - j); k++) {
                    A_temp *= A;
                }
                Gamma.block(i * n_states, j * n_controls, n_states, n_controls) = A_temp * B;
            }
        }

        Q_bar.block(i * n_states, i * n_states, n_states, n_states) = Q;
        R_bar.block(i * n_controls, i * n_controls, n_controls, n_controls) = R;

        A_power = A_power * A;
    }

    // resolver el problema Cuadrático
    Eigen::MatrixXd H = 2.0 * (Gamma.transpose() * Q_bar * Gamma + R_bar);
    Eigen::VectorXd f = 2.0 * Gamma.transpose() * Q_bar * Phi * x_tilde;

    Eigen::VectorXd U = -H.ldlt().solve(f);
    Eigen::Vector2d u_tilde_0(U(0), U(1));

    return u_tilde_0;
}

void MPC_Controller::main_loop() 
{
    if (x_trayectoria.empty() || ruta_terminada_)
    {   
        publish_cmd(0.0, 0.0);
        return;
    } 
        
    // parar al llegar al final
    size_t last_idx = x_trayectoria.size() - 1;
    float dist_al_final = sqrt(pow(x_trayectoria[last_idx] - current_x_, 2) + 
                               pow(y_trayectoria[last_idx] - current_y_, 2));
    
    // Tolerancia de 15 centímetros
    if (dist_al_final < 0.15) {
        RCLCPP_INFO(this->get_logger(), "¡Destino alcanzado! Parando motores.");
        ruta_terminada_ = true;
        x_trayectoria.clear();
        y_trayectoria.clear();
        publish_cmd(0.0, 0.0);
        return;
    }
    // ---------------------------------------------------

    int ref_index = find_nearest_point();

    float theta_ref = theta_actual_;
    if (ref_index + 1 < static_cast<int>(x_trayectoria.size()))
    {
        float dx_path = x_trayectoria[ref_index + 1] - x_trayectoria[ref_index];
        float dy_path = y_trayectoria[ref_index + 1] - y_trayectoria[ref_index];
        theta_ref = atan2(dy_path, dx_path);
    }

    Eigen::Vector2d u_r(V_LIN, 0.0);
    Eigen::Vector3d x_r(x_trayectoria[ref_index], y_trayectoria[ref_index], theta_ref);

    Eigen::Vector3d x_actual(current_x_, current_y_, theta_actual_);
    Eigen::Vector3d x_tilde = x_actual - x_r;
    x_tilde(2) = atan2(sin(x_tilde(2)), cos(x_tilde(2))); 

    Eigen::Matrix3d A;
    A << 1.0, 0.0, -u_r(0) * sin(theta_ref) * DT,
         0.0, 1.0,  u_r(0) * cos(theta_ref) * DT,
         0.0, 0.0,  1.0;

    Eigen::Matrix<double, 3, 2> B; 
    B << cos(theta_ref) * DT, 0.0,
         sin(theta_ref) * DT, 0.0,
         0.0,                 DT;
    
    Eigen::Vector2d u_tilde = solve_mpc(x_tilde, A, B);

    float v_cmd = u_r(0) + u_tilde(0);
    float w_cmd = u_r(1) + u_tilde(1);

    v_cmd = std::clamp(v_cmd, 0.0f, V_LIN * 1.5f);
    w_cmd = std::clamp(w_cmd, MIN_ANGULAR_VEL, MAX_ANGULAR_VEL);

    // Compensación de Zona Muerta (Deadband)
    float min_w_fisico = 0.33f; 
    
    // Si la velocidad pedida es muy pequeña (ruido) la ignoramos.
    if (std::abs(w_cmd) < 0.01f) {
        w_cmd = 0.0f;
    } 
    // Si la velocidad es mayor que el ruido pero no tiene fuerza para mover el motor, la subimos.
    else if (std::abs(w_cmd) < min_w_fisico) {
        // Mantenemos el signo (dirección del giro) pero aplicamos la fuerza mínima
        w_cmd = (w_cmd > 0.0f) ? min_w_fisico : -min_w_fisico;
    }
    
    publish_cmd(v_cmd, w_cmd);
}

void MPC_Controller::publish_cmd(float v, float w)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    pub_vel_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mpc_node = std::make_shared<MPC_Controller>();
    rclcpp::spin(mpc_node);
    rclcpp::shutdown();
    return 0;
}