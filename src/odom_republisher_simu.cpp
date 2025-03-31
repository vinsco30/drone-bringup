#include "../include/gz_drone_bringup/odom_republisher_simu.h"

OdomRepublisherSimu::OdomRepublisherSimu() : rclcpp::Node( "odom_republisher_simu" ) {

    this->declare_parameter<string>("prefix_gz", "x500_0");
    _prefix_gz = this->get_parameter("prefix_gz").as_string();
    this->declare_parameter<string>("prefix_tf", "uav");
    _prefix_tf = this->get_parameter("prefix_tf").as_string();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    const char* first_part = "/model/";
    const char* second_part = "/odometry";
    char result[30];

    strcpy(result, first_part);
    strcat(result, _prefix_gz.c_str());
    strcat(result, second_part);

    _odom_gz_sub = 
        this->create_subscription<nav_msgs::msg::Odometry>( result, qos, 
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

    _tf_broadcaster_static1 =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_broadcaster_static2 =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_broadcaster_static3 = 
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_broadcaster_static4 = 
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_broadcaster_static5 = 
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_broadcaster_static6 = 
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_broadcaster_static7 = 
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_broadcaster1 = 
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    _tf_listener_ned = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    _timer_static_tf =
        this->create_wall_timer( std::chrono::milliseconds(100),
        std::bind( &OdomRepublisherSimu::static_tf_pub, this ) );

    _timer_tf_listener =
        this->create_wall_timer( std::chrono::milliseconds(100),
        std::bind( &OdomRepublisherSimu::tf_listener, this ) );

    // _timer_px4_out = 
    //     this->create_wall_timer( std::chrono::milliseconds(100), 
    //     std::bind( &OdomRepublisherSimu::px4_odom_repub, this ) );

    _R_flu2frd = utilities::rotx(-M_PI);
} 

void OdomRepublisherSimu::static_tf_pub() {
    
    _t_static2.header.stamp = this->get_clock()->now();
    _t_static2.header.frame_id = _prefix_tf+"/odom";
    _t_static2.child_frame_id = _prefix_tf+"/odom_ned";
    _t_static2.transform.translation.x = 0.0;
    _t_static2.transform.translation.y = 0.0;
    _t_static2.transform.translation.z = 0.0;
    _t_static2.transform.rotation.w = 0.0;
    _t_static2.transform.rotation.x = 0.707;
    _t_static2.transform.rotation.y = 0.707;
    _t_static2.transform.rotation.z = 0.0;

    _tf_broadcaster_static2->sendTransform(_t_static2);

    _t_static3.header.stamp = this->get_clock()->now();
    _t_static3.header.frame_id =  _prefix_tf+"/odom_px4_ned";
    _t_static3.child_frame_id =  _prefix_tf+"/odom_px4";
    _t_static3.transform.translation.x = 0.0;
    _t_static3.transform.translation.y = 0.0;
    _t_static3.transform.translation.z = 0.0;
    _t_static3.transform.rotation.w = 0.0;
    _t_static3.transform.rotation.x = 0.707;
    _t_static3.transform.rotation.y = 0.707;
    _t_static3.transform.rotation.z = 0.0;

    _tf_broadcaster_static3->sendTransform(_t_static3);

    _t_static5.header.stamp = this->get_clock()->now();
    _t_static5.header.frame_id =  _prefix_tf+"/map";
    _t_static5.child_frame_id =  _prefix_tf+"/map_ned";
    _t_static5.transform.translation.x = 0.0;
    _t_static5.transform.translation.y = 0.0;
    _t_static5.transform.translation.z = 0.0;
    _t_static5.transform.rotation.w = 0.0;
    _t_static5.transform.rotation.x = 0.707;
    _t_static5.transform.rotation.y = 0.0;
    _t_static5.transform.rotation.z = 0.707;

    _tf_broadcaster_static5->sendTransform(_t_static5);

    _t_static6.header.stamp = this->get_clock()->now();
    _t_static6.header.frame_id =  _prefix_tf+"/base_link";
    _t_static6.child_frame_id =  _prefix_tf+"/camera_link";
    _t_static6.transform.translation.x = 0.12;
    _t_static6.transform.translation.y = 0.03;
    _t_static6.transform.translation.z = 0.0;
    _t_static6.transform.rotation.w = 1.0;
    _t_static6.transform.rotation.x = 0.0;
    _t_static6.transform.rotation.y = 0.0;
    _t_static6.transform.rotation.z = 0.0;

    _tf_broadcaster_static6->sendTransform(_t_static6);

    _t_static7.header.stamp = this->get_clock()->now();
    _t_static7.header.frame_id =  _prefix_tf+"/camera_link";
    _t_static7.child_frame_id =  "x500_depth_0/OakD-Lite/base_link/StereoOV7251";
    _t_static7.transform.translation.x = 0.0;
    _t_static7.transform.translation.y = 0.0;
    _t_static7.transform.translation.z = 0.0;
    _t_static7.transform.rotation.w = 1.0;
    _t_static7.transform.rotation.x = 0.0;
    _t_static7.transform.rotation.y = 0.0;
    _t_static7.transform.rotation.z = 0.0;

    _tf_broadcaster_static7->sendTransform(_t_static7);

}

void OdomRepublisherSimu::odom_gz_cb( const nav_msgs::msg::Odometry::SharedPtr msg ) {

    _gz_pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    _gz_quat << msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;
    _gz_vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    _gz_ang_vel << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;

    _t_static1.header.stamp = this->get_clock()->now();
    _t_static1.header.frame_id =  _prefix_tf+"/base_link";
    _t_static1.child_frame_id =  _prefix_tf+"/base_link_frd";
    _t_static1.transform.translation.x = 0.0;
    _t_static1.transform.translation.y = 0.0;
    _t_static1.transform.translation.z = 0.0;
    _t_static1.transform.rotation.w = 0.0;
    _t_static1.transform.rotation.x = 1.0;
    _t_static1.transform.rotation.y = 0.0;
    _t_static1.transform.rotation.z = 0.0;

    _tf_broadcaster_static1->sendTransform(_t_static1);

    _t.header.stamp = this->get_clock()->now();
    _t.header.frame_id =  _prefix_tf+"/odom";
    _t.child_frame_id =  _prefix_tf+"/base_link";
    _t.transform.translation.x = _gz_pos[0];
    _t.transform.translation.y = _gz_pos[1];
    _t.transform.translation.z = _gz_pos[2];
    _t.transform.rotation.w = _gz_quat[0];
    _t.transform.rotation.x = _gz_quat[1];
    _t.transform.rotation.y = _gz_quat[2];
    _t.transform.rotation.z = _gz_quat[3];

    _tf_broadcaster->sendTransform(_t);


}

void OdomRepublisherSimu::tf_listener() {
    
    try {
        auto transform = _tf_buffer->lookupTransform( _prefix_tf+"/odom_ned",  _prefix_tf+"/base_link_frd", tf2::TimePointZero);
        _px4_pos_in << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z;
        _px4_quat_in << transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }

    auto odom_px4 = std::make_unique<px4_msgs::msg::VehicleOdometry>();
    odom_px4->pose_frame = 2;
    odom_px4->timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
    odom_px4->position[0] = _px4_pos_in[0];
    odom_px4->position[1] = _px4_pos_in[1];
    odom_px4->position[2] = _px4_pos_in[2];
    odom_px4->q[0] = _px4_quat_in[0];
    odom_px4->q[1] = _px4_quat_in[1];
    odom_px4->q[2] = _px4_quat_in[2];
    odom_px4->q[3] = _px4_quat_in[3];
    odom_px4->velocity_frame = 2;
    odom_px4->velocity[0] = NAN;
    odom_px4->velocity[1] = NAN;
    odom_px4->velocity[2] = NAN;
    odom_px4->angular_velocity[0] = NAN;
    odom_px4->angular_velocity[1] = NAN;
    odom_px4->angular_velocity[2] = NAN;

    _odom_px4_pub->publish( std::move( odom_px4 ) );
    _first_odom_sent = true;


}

void OdomRepublisherSimu::odom_px4_cb( const px4_msgs::msg::VehicleOdometry::SharedPtr msg ) {

    if( _first_odom_sent ) {
        _px4_pos_out << msg->position[0], msg->position[1], msg->position[2];
        _px4_quat_out << msg->q[0], msg->q[1], msg->q[2], msg->q[3];


        /*Broadcaster FRD transform*/
        _t_px4.header.stamp = this->get_clock()->now();
        _t_px4.header.frame_id =  _prefix_tf+"/odom_px4_ned";
        _t_px4.child_frame_id =  _prefix_tf+"/base_link_px4_frd";
        _t_px4.transform.translation.x = _px4_pos_out[0];
        _t_px4.transform.translation.y = _px4_pos_out[1];
        _t_px4.transform.translation.z = _px4_pos_out[2];
        _t_px4.transform.rotation.w = _px4_quat_out[0];
        _t_px4.transform.rotation.x = _px4_quat_out[1];
        _t_px4.transform.rotation.y = _px4_quat_out[2];
        _t_px4.transform.rotation.z = _px4_quat_out[3];

        _t_static4.header.stamp = this->get_clock()->now();
        _t_static4.header.frame_id =  _prefix_tf+"/base_link_px4_frd";
        _t_static4.child_frame_id =  _prefix_tf+"/base_link_px4";
        _t_static4.transform.translation.x = 0.0;
        _t_static4.transform.translation.y = 0.0;
        _t_static4.transform.translation.z = 0.0;
        _t_static4.transform.rotation.w = 0.0;
        _t_static4.transform.rotation.x = 1.0;
        _t_static4.transform.rotation.y = 0.0;
        _t_static4.transform.rotation.z = 0.0;

        _tf_broadcaster1->sendTransform(_t_px4);
        _tf_broadcaster_static4->sendTransform(_t_static4);
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