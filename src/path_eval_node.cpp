#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "eigen3/Eigen/Dense"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer.

TODO:
make header file
put the visualization stuff in a separate file
use the eigen io module to print the matrices
check the values on python side
change the makers to arrows and the right colors
 */



class PathEvaluator : public rclcpp::Node
{
  public:
    PathEvaluator()
    : Node("path_eval_node")
    {
      sub_pos = this->create_subscription<nav_msgs::msg::Path>(
        "/bspline_node/pos", 10, std::bind(&PathEvaluator::pos_cb, this, _1));
      sub_vel = this->create_subscription<nav_msgs::msg::Path>(
        "/bspline_node/vel", 10, std::bind(&PathEvaluator::vel_cb, this, _1));
      sub_acc = this->create_subscription<nav_msgs::msg::Path>(
        "/bspline_node/acc", 10, std::bind(&PathEvaluator::acc_cb, this, _1));

      timer = this->create_wall_timer(500ms, std::bind(&PathEvaluator::timer_cb, this));
      pub_maxvel = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/max_vel", 10);
      pub_maxacc = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/max_acc", 10);
      pub_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
    }
  private:
    Eigen::Matrix<double, Eigen::Dynamic, 3> pos_path;
    float path_length = 0.0;
    int index_maxvel  = 0;
    float maxvel      = 0.0;
    int index_minvel  = 0;
    float minvel      = 0.0;
    int index_maxacc  = 0;
    float maxacc      = 0.0;
    int index_minacc  = 0;
    float minacc      = 0.0;

    void pos_cb(const nav_msgs::msg::Path & msg)
    {
      Eigen::Matrix<double, Eigen::Dynamic, 3> pos_path = path_to_matrix(msg);  
      this->pos_path = pos_path;
      int n = pos_path.rows();

      // make a copy of pos_path
      Eigen::Matrix<double, Eigen::Dynamic, 3> copy;
      copy.resize(n,3);
      copy.block(0,0,n-1,3) = pos_path.block(1,0,n-1,3);
      
      Eigen::Matrix<double, Eigen::Dynamic, 3> diff;
      diff.resize(n,3);
      diff = copy - pos_path;
      Eigen::Matrix<double, Eigen::Dynamic, 1> norms;
      norms.resize(n,1);
      norms = diff.rowwise().norm();
      float path_length = norms.sum();

      this->path_length = path_length;
      }
    
    
    void vel_cb(const nav_msgs::msg::Path & msg)
    {
      Eigen::Matrix<double, Eigen::Dynamic, 3> vel_path = path_to_matrix(msg); 
      Eigen::Matrix<double, Eigen::Dynamic, 1> vel_norms;
      int n = vel_path.rows();
      vel_norms.resize(n,1);
      vel_norms = vel_path.rowwise().norm();

      Eigen::Matrix2f m = min_max(vel_norms);
      this->index_minvel = m(0,0);
      this->minvel = m(0,1);
      this->index_maxvel = m(1,0);
      this->maxvel = m(1,1);
      // Eigen::Matrix3d pos_minvel = get_position(this->index_minvel,);
      // convert pos_minvel to string
      // std::string str_pos_minvel = std::to_string(pos_minvel(0,0)) + ", " + std::to_string(pos_minvel(0,1)) + ", " + std::to_string(pos_minvel(0,2));
      // print to terminal
      // RCLCPP_INFO(this->get_logger(), "pos_minvel : '%s'", str_pos_minvel);
    }
    

    void acc_cb(const nav_msgs::msg::Path & msg)
    {
      Eigen::Matrix<double, Eigen::Dynamic, 3> acc_path = path_to_matrix(msg); 
      Eigen::Matrix<double, Eigen::Dynamic, 1> acc_norms;
      int n = acc_path.rows();
      acc_norms.resize(n,1);
      acc_norms = acc_path.rowwise().norm();

      Eigen::Matrix2f m = min_max(acc_norms);
      this->index_minacc = m(0,0);
      this->minacc = m(0,1);
      this->index_maxacc = m(1,0);
      this->maxacc = m(1,1);
    }

    Eigen::Matrix<double, Eigen::Dynamic, 3> path_to_matrix(const nav_msgs::msg::Path & msg)
    {
      Eigen::Matrix<double, Eigen::Dynamic, 3> path_matrix;
      int n = msg.poses.size();
      path_matrix.resize(n,3);
      int i = 0;
        for(std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it=msg.poses.begin(); it!=msg.poses.end();  ++it)
        {
          path_matrix.row(i) << it->pose.position.x, it->pose.position.y, it->pose.position.z;
          i++;
        }
      return path_matrix;
      
    }
    Eigen::Matrix2f min_max(Eigen::Matrix<double, Eigen::Dynamic, 1> & norms)
    {
      // get value and location of min and max
      Eigen::Index index_max;
      Eigen::Index index_min;
      float max = norms.maxCoeff(&index_max);
      float min = norms.minCoeff(&index_min);
    
      // RCLCPP_INFO(this->get_logger(), "max: '%f' messages",max);
      Eigen::Matrix2f out;
      out << index_min, min, 
             index_max, max;
      return out;
    }

    Eigen::Matrix3d get_position(int index)
    {

      Eigen::Matrix3d pos = Eigen::Matrix3d::Zero();
      // pos = this->pos_path;
    
      // float x = this->pos_path(index,0);
      pos.block(0,0,1,3) = this->pos_path.block(index,0,1,3);
      return pos;
    }

    void timer_cb()
    {
      int n = this->pos_path.rows();
      if  (n == 0)
      {
        RCLCPP_INFO(this->get_logger(), "position path is empty: waiting for data...");
        return;
      }
      else
      {
        Eigen::Matrix3d pos_minvel = get_position(this->index_minvel);
        Eigen::Matrix3d pos_maxvel = get_position(this->index_maxvel);
        Eigen::Matrix3d pos_minacc = get_position(this->index_minacc);
        Eigen::Matrix3d pos_maxacc = get_position(this->index_maxacc);
        
        //convert to string
        // std::string str_pos_maxvel = std::to_string(pos_maxvel(0,0)) + ", " + std::to_string(pos_maxvel(0,1)) + ", " + std::to_string(pos_maxvel(0,2));
        // std::string str_pos_maxacc = std::to_string(pos_maxacc(0,0)) + ", " + std::to_string(pos_maxacc(0,1)) + ", " + std::to_string(pos_maxacc(0,2));

        auto pose_maxvel = geometry_msgs::msg::PoseStamped();
        pose_maxvel.header.frame_id = "global";
        pose_maxvel.header.stamp = this->now();
        pose_maxvel.pose.position.x = pos_maxvel(0,0);
        pose_maxvel.pose.position.y = pos_maxvel(0,1);
        pose_maxvel.pose.position.z = pos_maxvel(0,2);
        pose_maxvel.pose.orientation.w = 1.0;

        auto pose_maxacc = geometry_msgs::msg::PoseStamped();
        pose_maxacc.header.frame_id = "global";
        pose_maxacc.header.stamp = this->now();
        pose_maxacc.pose.position.x = pos_maxacc(0,0);
        pose_maxacc.pose.position.y = pos_maxacc(0,1);
        pose_maxacc.pose.position.z = pos_maxacc(0,2);
        pose_maxacc.pose.orientation.w = 1.0;

        auto markers = visualization_msgs::msg::MarkerArray();
        auto m1 = visualization_msgs::msg::Marker();
        m1.header.frame_id = "global";
        m1.header.stamp = this->now();
        m1.id = 0;
        m1.type = visualization_msgs::msg::Marker::SPHERE;
        m1.action = visualization_msgs::msg::Marker::ADD;
        m1.pose = pose_maxvel.pose;
        m1.scale.x = 0.25;
        m1.scale.y = 0.25;
        m1.scale.z = 0.25;
        m1.color.a = 1.0; 
        m1.color.r = 1.0;
        m1.color.g = 0.0;
        m1.color.b = 0.0;
        markers.markers.push_back(m1);
        auto m2 = visualization_msgs::msg::Marker();
        m2.header.frame_id = "global";
        m2.header.stamp = this->now();
        m2.id = 1;
        m2.type = visualization_msgs::msg::Marker::SPHERE;
        m2.action = visualization_msgs::msg::Marker::ADD;
        m2.pose = pose_maxacc.pose;
        m2.scale.x = 0.25;
        m2.scale.y = 0.25;
        m2.scale.z = 0.25;
        m2.color.a = 1.0; 
        m2.color.r = 0.0;
        m2.color.g = 0.0;
        m2.color.b = 1.0;
        markers.markers.push_back(m2);

        // Publish the marker array
        pub_markers->publish(markers);

        RCLCPP_INFO(this->get_logger(), "path_length: '%f'", this->path_length);
        RCLCPP_INFO(this->get_logger(), "max_vel: '%f'", this->maxvel);
        RCLCPP_INFO(this->get_logger(), "max_acc: '%f'", this->maxacc);
        RCLCPP_INFO(this->get_logger(), " ");
        pub_maxvel->publish(pose_maxvel);
        pub_maxvel->publish(pose_maxacc);
      }
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_pos;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_vel;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_acc;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_maxvel;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_maxacc;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathEvaluator>());
  rclcpp::shutdown();
  return 0;
}