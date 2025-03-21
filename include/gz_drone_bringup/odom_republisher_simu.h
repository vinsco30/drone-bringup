/*Class for ROS2 node that republish the odometry from the 
GZ simulator into the PX4 vehicle odometry in emulating a 
visual inertial odometry system. Additionally it publishes
tf transformation between odom_ned and base_link_frd*/

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "utils.h"

/*ROS2 msgs*/
#include <nav_msgs/msg/odometry.hpp>

/*TF libraries*/
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

/*PX4 odometry msg*/
#include <px4_msgs/msg/vehicle_odometry.hpp>

/*Eigen libraries*/
#include <Eigen/Eigen>
#include <Eigen/Dense>

class OdomRepublisherSimu : public rclcpp::Node
{
    public:
        OdomRepublisherSimu();
    
    private:
        void odom_gz_cb( const nav_msgs::msg::Odometry::SharedPtr ); 
        void odom_px4_cb( const px4_msgs::msg::VehicleOdometry::SharedPtr );
        void px4_odom_repub();

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_gz_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_px4_sub;

        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_px4_pub;

        rclcpp::TimerBase::SharedPtr _timer_px4_out;

        Eigen::Vector3d _gz_pos, _gz_vel, _gz_ang_vel, _px4_pose_flu;
        Eigen::Vector4d _gz_quat, _px4_quat_flu;
        Eigen::Vector3d _px4_pos_out, _px4_vel_out, _px4_ang_vel_out;
        Eigen::Vector4d _px4_quat_out;
        Eigen::Vector3d _px4_pos_in, _px4_vel_in, _px4_ang_vel_in;
        Eigen::Vector4d _px4_quat_in;

        /*Broadcaster*/
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster_flu;
        geometry_msgs::msg::TransformStamped _t, _t_flu;

        /*Rotation matrices*/
        Eigen::Matrix3d _R_flu2frd;

        /*Flags*/
        bool _first_odom_sent = false;


};



