#include "../include/gz_drone_bringup/odom_republisher_simu.h"

OdomRepublisherSimu::OdomRepublisherSimu() : rclcpp::Node( "odom_republisher_simu" ) {

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    _odom_gz_sub = 
        this->create_subscription<nav_msgs::msg::Odometry>( "/model/x500_0/odometry", qos, 
        std::bind( &OdomRepublisherSimu::odom_gz_cb, this, std::placeholders::_1 ) );

    _odom_px4_sub = 
        this->create_subscription<px4_msgs::msg::VehicleOdometry>( "/fmu/out/vehicle_odometry", qos, 
        std::bind( &OdomRepublisherSimu::odom_px4_cb, this, std::placeholders::_1 ) );

    _odom_px4_pub = 
        this->create_publisher<px4_msgs::msg::VehicleOdometry>( "/fmu/in/vehicle_visual_odometry", 10 );

    _tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_broadcaster_flu =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _timer_px4_out = 
        this->create_wall_timer( std::chrono::milliseconds(100), 
        std::bind( &OdomRepublisherSimu::px4_odom_repub, this ) );

    _R_flu2frd = utilities::rotx(-M_PI);
}   

void OdomRepublisherSimu::odom_gz_cb( const nav_msgs::msg::Odometry::SharedPtr msg ) {

    _gz_pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    _gz_quat << msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;
    _gz_vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    _gz_ang_vel << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;

}

void OdomRepublisherSimu::odom_px4_cb( const px4_msgs::msg::VehicleOdometry::SharedPtr msg ) {

    if( _first_odom_sent ) {
        _px4_pos_out << msg->position[0], msg->position[1], msg->position[2];
        _px4_quat_out << msg->q[0], msg->q[1], msg->q[2], msg->q[3];

        /*Broadcaster FRD transform*/
        _t.header.stamp = this->get_clock()->now();
        _t.header.frame_id = "odom";
        _t.child_frame_id = "base_link_frd";
        _t.transform.translation.x = _px4_pos_out[0];
        _t.transform.translation.y = _px4_pos_out[1];
        _t.transform.translation.z = _px4_pos_out[2];
        _t.transform.rotation.w = _px4_quat_out[0];
        _t.transform.rotation.x = _px4_quat_out[1];
        _t.transform.rotation.y = _px4_quat_out[2];
        _t.transform.rotation.z = _px4_quat_out[3];

        _tf_broadcaster->sendTransform(_t);

        /*Broadcaster FLU transform*/
        _px4_pose_flu = _R_flu2frd.transpose()*_px4_pos_out;
        _px4_quat_flu = utilities::rot2quat( utilities::Q2R( _px4_quat_out )*_R_flu2frd.transpose() );

        _t_flu.header.stamp = this->get_clock()->now();
        _t_flu.header.frame_id = "odom";
        _t_flu.child_frame_id = "base_link";
        _t_flu.transform.translation.x = _px4_pose_flu[0];
        _t_flu.transform.translation.y = _px4_pose_flu[1];
        _t_flu.transform.translation.z = _px4_pose_flu[2];
        _t_flu.transform.rotation.w = _px4_quat_flu[0];
        _t_flu.transform.rotation.x = _px4_quat_flu[1];
        _t_flu.transform.rotation.y = _px4_quat_flu[2];
        _t_flu.transform.rotation.z = _px4_quat_flu[3];

        _tf_broadcaster_flu->sendTransform(_t_flu);
    }

}

void OdomRepublisherSimu::px4_odom_repub() {

    /*From FLU to FRD*/
    _px4_pos_in = _R_flu2frd*_gz_pos;
    _px4_vel_in = _R_flu2frd*_gz_vel;
    _px4_quat_in = utilities::rot2quat( utilities::Q2R( _gz_quat )*_R_flu2frd );
    _px4_ang_vel_in = _R_flu2frd*_gz_ang_vel;

    auto odom_px4 = std::make_unique<px4_msgs::msg::VehicleOdometry>();
    odom_px4->pose_frame = 2;
    odom_px4->timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
    odom_px4->position[0] = _px4_pos_in[0];
    odom_px4->position[1] = _px4_pos_in[1];
    odom_px4->position[2] = _px4_pos_in[2];
    odom_px4->q[0] = NAN;
    odom_px4->q[1] = NAN;
    odom_px4->q[2] = NAN;
    odom_px4->q[3] = NAN;
    odom_px4->velocity_frame = 2;
    odom_px4->velocity[0] = _px4_vel_in[0];
    odom_px4->velocity[1] = _px4_vel_in[1];
    odom_px4->velocity[2] = _px4_vel_in[2];
    // odom_px4->angular_velocity[0] = _px4_ang_vel[0];
    // odom_px4->angular_velocity[1] = _px4_ang_vel[1];
    // odom_px4->angular_velocity[2] = _px4_ang_vel[2];
    // odom_px4->velocity[0] = NAN;
    // odom_px4->velocity[1] = NAN;
    // odom_px4->velocity[2] = NAN;
    odom_px4->angular_velocity[0] = NAN;
    odom_px4->angular_velocity[1] = NAN;
    odom_px4->angular_velocity[2] = NAN;

    _odom_px4_pub->publish( std::move( odom_px4 ) );
    _first_odom_sent = true;
}

int main(int argc, char *argv[]) {

    std::cout<< "Starting Odometry republisher node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomRepublisherSimu>());

    rclcpp::shutdown();
    return 0;
}