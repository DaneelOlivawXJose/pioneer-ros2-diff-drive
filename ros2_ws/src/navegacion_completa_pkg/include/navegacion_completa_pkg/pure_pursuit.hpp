#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>

using json = nlohmann::json;

class PurePursuit : public rclcpp::Node
{
    public:
        PurePursuit();
        ~PurePursuit();
    private:
        void main_loop();
        int find_nearest_point();
        void odom_callback(const geometry_msgs::msg::Point::SharedPtr msg);
        int find_lookahead_point(int min_index_);
        void publish_cmd(float v, float w);
        void settings_callback(const std_msgs::msg::String::SharedPtr msg);

        // Subscriptions and Publishers
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr rec_coordinate_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr rec_odom_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr settings_sub_;

        // Variables varias
        float LOOKAHEAD_DIST = 0.6;
        float V_LIN = 0.1;
        float MAX_ANGULAR_VEL = 0.9;
        float MIN_ANGULAR_VEL = -0.9;
        int N_INTERP = 7;

        float current_x_;
        float current_y_;
        float theta_actual_;

        std::vector<double> x_trayectoria;
        std::vector<double> y_trayectoria;
        bool ruta_terminada_;

        rclcpp::TimerBase::SharedPtr timer_;
};