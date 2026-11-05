// Obstacle simulator: moves the obstacles of config.yaml with the behaviours of ObstacleManager.hpp and 
// publishes them as legged_controllers/ObstacleArray in odom, the same message and topic of the perception
// that in the future we will implement
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <legged_controllers/msg/obstacle_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "acados_legged_controller/LipmConfig.hpp"
#include "acados_legged_controller/ObstacleManager.hpp"



namespace legged {

class ObstacleSimulator : public rclcpp::Node {
public:
    // legged::Obstacle is the base class of ObstacleManager.hpp, message gets another name
    using ObstacleMsg = legged_controllers::msg::Obstacle;
    using ObstacleArrayMsg = legged_controllers::msg::ObstacleArray;

    ObstacleSimulator() : rclcpp::Node("obstacle_simulator") {
        std::string config_file = declare_parameter<std::string>("lipmConfigFile", "");
        if (config_file.empty()) {
            config_file = ament_index_cpp::get_package_share_directory("acados_legged_controller") + "/config/config.yaml";
        }
        const double rate = declare_parameter<double>("rate", 50.0);    // Hz
        const std::string topic = declare_parameter<std::string>("obstacle_topic", "/opti_pessi/obstacles");
        frame_id_ = declare_parameter<std::string>("frame_id", "odom");
        moving_ = declare_parameter<bool>("autostart", false);  // still until ~/set_moving true

        cfg_ = LipmConfig::fromYaml(config_file);
        manager_ = std::make_unique<ObstacleManager>(cfg_);
        // class of each obstacle, same order as ObstacleManager::active(), instances first then random
        for (const auto& obs : cfg_.obstacles) {
            classes_.push_back(obs.obstacle_class == "car" ? ObstacleMsg::CAR : ObstacleMsg::HUMAN);
        }
        for (int i = 0; i < cfg_.random_obstacles.count; ++i) classes_.push_back(ObstacleMsg::HUMAN);

        obstacles_pub_ = create_publisher<ObstacleArrayMsg>(topic, 1);
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 1);

        // Robot position for the adversarial obstacles (state estimator, odom frame)
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                robot_x_ = msg->pose.pose.position.x;
                robot_y_ = msg->pose.pose.position.y;
            });

        moving_srv_ = create_service<std_srvs::srv::SetBool>("~/set_moving",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                        moving_ = req->data;
                        res->success = true;
                        res->message = moving_ ? "obstacles moving" : "obstacles paused";
            });

        // Timer on the node clock: follows /clock with use_sim_time:=true
        last_time_ = now();
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() { step(); });
        RCLCPP_INFO(get_logger(), "[ObstacleSimulator]: %zu obstacle(s) from %s on %s", classes_.size(), config_file.c_str(), topic.c_str());

    }


private:
    void step() {
        const rclcpp::Time time = now();
        double dt = (time - last_time_).seconds();
        last_time_ = time;
        if (dt < 0.0 || dt > 0.5) dt = 0.0; // clock jump: no teleport

        if (moving_) {
            t_moving_ += dt;    // time of the motion only: dynamic ones restart when paused
            manager_->updateAll(dt, t_moving_, robot_x_, robot_y_);
        }
        const std::vector<ObstacleState> states = manager_->active();

        // Every obstacle is one message, also when empty: receiver replaces its set with each message
        ObstacleArrayMsg msg;
        msg.header.stamp = time;
        msg.header.frame_id = frame_id_;
        for (size_t i = 0; i < states.size(); ++i) {
            ObstacleMsg obstacle;
            obstacle.type = classes_[i];
            obstacle.position.x = states[i].x;
            obstacle.position.y = states[i].y;
            obstacle.position.z = 0.0;
            msg.obstacles.push_back(obstacle);
        }
        obstacles_pub_->publish(msg);
        publishMarkers(states, time);
    }


    // what the controller will assume: radius of the class, not r_obs of the instance
    void publishMarkers(const std::vector<ObstacleState>& states, const rclcpp::Time& time) {
        visualization_msgs::msg::MarkerArray markers;
        for (size_t i = 0; i < states.size(); ++i) {
            const double radius = (classes_[i] == ObstacleMsg::CAR) ? cfg_.car_type.radius : cfg_.human_type.radius;
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = time;
            marker.header.frame_id = frame_id_;
            marker.ns = "simulated_obstacle";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = states[i].x;
            marker.pose.position.y = states[i].y;
            marker.pose.position.z = 0.25;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = 2.0 * radius;
            marker.scale.z = 0.5;
            marker.color.r = 0.9f;
            marker.color.g = 0.5f;
            marker.color.b = 0.1f;
            marker.color.a = 0.6f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            markers.markers.push_back(marker);
        }
        markers_pub_->publish(markers);
    }

    LipmConfig cfg_;
    std::unique_ptr<ObstacleManager> manager_;
    std::vector<uint8_t> classes_;
    std::string frame_id_;
    bool moving_ = false;
    double t_moving_ = 0.0;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    rclcpp::Time last_time_;

    rclcpp::Publisher<ObstacleArrayMsg>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr moving_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

};


} // namespace legged



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<legged::ObstacleSimulator>());
    rclcpp::shutdown();
    return 0;
}