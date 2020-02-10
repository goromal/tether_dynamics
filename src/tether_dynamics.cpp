#include "tether_dynamics/tether_dynamics.h"

namespace tether_dynamics {

TetherDynamics::TetherDynamics() : X_(0.0), Y_(0.0), L_(0.0), L_euclid_(0.0),
    a_(4.0), nh_(), nh_private_("~"), tfBuffer_(), tfListener_(tfBuffer_)
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("tether_marker", 1);
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("uav_ext_wrench", 1);
    motor_wrench_sub_ = nh_.subscribe("uav_motor_wrench", 1, &TetherDynamics::motorWrenchCallback, this);
    uav_state_sub_ = nh_.subscribe("uav_truth_NED", 1, &TetherDynamics::uavStateCallback, this);

    lambda_ = nh_private_.param<double>("tether_unit_mass", 0.001);
    g_ = nh_private_.param<double>("gravity", 9.80665);

    kp_ = nh_private_.param<double>("tether_P_gain", 1.0);
    ki_ = nh_private_.param<double>("tether_I_gain", 0.1);
    kd_ = nh_private_.param<double>("tether_D_gain", 0.5);
    tether_pid_.init(kp_, ki_, kd_, 100.0);

    kpT_ = nh_private_.param<double>("tether_P_yaw_gain", 0.0);
    kiT_ = nh_private_.param<double>("tether_I_yaw_gain", 0.0);
    kdT_ = nh_private_.param<double>("tether_D_yaw_gain", 0.0);
    tether_torque_pid_.init(kpT_, kiT_, kdT_, 100.0);

    L_lim_ = nh_private_.param<double>("tether_limit", 20.0);
    L_buff_ = nh_private_.param<double>("tether_limit_buffer", 0.1);

    marker_.header.frame_id = "boat";
    marker_.id = 3;
    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose = geometry_msgs::Pose();
    marker_.scale.x = 0.075;
    marker_.scale.y = 0.075;
    marker_.scale.z = 0.075;
    setWhiteLine();
    marker_.color.a = 1.0;

    memset(&wrench_, 0.0, sizeof(wrench_));
    memset(&motor_wrench_, 0.0, sizeof(motor_wrench_));
    memset(&UAV_vel_, 0.0, sizeof(UAV_vel_));

    x_moment_arm_ = nh_private_.param<double>("uav_tether_x_moment_arm", 0.1);
    double y_moment_arm = nh_private_.param<double>("uav_tether_y_moment_arm", 0.0);
    double z_moment_arm = nh_private_.param<double>("uav_tether_z_moment_arm", 0.0);
    moment_arm_ = Vector3d(x_moment_arm_, y_moment_arm, z_moment_arm);

    timer_ = nh_.createTimer(ros::Duration(ros::Rate(100)), &TetherDynamics::onUpdate, this); // DIRECTLY RELATED TO HARD-CODED PID DT VALUE!
}

void TetherDynamics::onUpdate(const ros::TimerEvent &event)
{
    try
    {
        tf_BOAT_UAV_ = tfBuffer_.lookupTransform("boat", "UAV", ros::Time(0));
        q_BOAT_UAV_ = Quatd(Vector4d(tf_BOAT_UAV_.transform.rotation.w,
                                     tf_BOAT_UAV_.transform.rotation.x,
                                     tf_BOAT_UAV_.transform.rotation.y,
                                     tf_BOAT_UAV_.transform.rotation.z));
        tf_NED_UAV_  = tfBuffer_.lookupTransform("NED", "UAV", ros::Time(0));
        q_NED_UAV_ = Quatd(Vector4d(tf_NED_UAV_.transform.rotation.w,
                                    tf_NED_UAV_.transform.rotation.x,
                                    tf_NED_UAV_.transform.rotation.y,
                                    tf_NED_UAV_.transform.rotation.z));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    resetWrench();

    // Tether dynamics parameters
    double x = tf_BOAT_UAV_.transform.translation.x;
    double y = tf_BOAT_UAV_.transform.translation.y;
    X_ = Vector2d(x, y).norm();
    Y_ = tf_BOAT_UAV_.transform.translation.z;

    // Only proceed if UAV is in a relative position that makes sense for the tether
    if (X_ < 0.5 || Y_ < 0.5)
        return;

    Theta_ = Quatd::from_two_unit_vectors(-e1, Vector3d(x, y, 0.0).normalized());

    // Find shape factor, a
    if (!estimateCurvature())
        return;

    // Draw the tether
    drawPoints();

    // Calculate tether length
    calculateLengths();

    // Calculate baseline weight forces and torques imposed on UAV
    calculateBaselines();

    // Handle taut forces
    if (L_euclid_ >= L_lim_ - L_buff_)
    {
        setRedLine();
        computeControl();
    }
    else
    {
        tether_pid_.reset();
        setWhiteLine();
    }

//    // Get the overall induced torque on UAV
//    calculateTorque();

    // Publish external forces wrench
    wrench_.force.x = Force_.x();
    wrench_.force.y = Force_.y();
    wrench_.force.z = Force_.z();
    wrench_.torque.x = Torque_.x();
    wrench_.torque.y = Torque_.y();
    wrench_.torque.z = Torque_.z();
    wrench_pub_.publish(wrench_);
}

bool TetherDynamics::estimateCurvature()
{
    // Newton Rhapson Method
    double prev_e = 1000.0;
    double e = 0.0;
    int i = 0;
    double Y_bar = 0.0;
    while (i < MAX_ITER)
    {
        Y_bar = a_ * (cosh(X_ / a_) - 1.0);
        e = Y_bar - Y_;
        if (abs(e) <= ERROR_TOLERANCE || prev_e < abs(e))
            break;
        prev_e = abs(e);
        a_ -= e / (cosh(X_ / a_) - 1 - X_ * sinh(X_ / a_) / a_);
        i++;
    }
    if (prev_e < abs(e))
    {
        a_ = 4.0;
        return false;
    }
    return true;
}

void TetherDynamics::drawPoints()
{
    std::vector<geometry_msgs::Point> points;
//    std::cout << X_ << " " << Y_ << " " << a_ << std::endl;

    double dx = X_ / (NUM_POINTS - 1);
    Matrix3d R_active = Theta_.R().transpose();

    // Calculate points
    for (int i = 0; i < NUM_POINTS; i++)
    {
        double x = i * dx;
        geometry_msgs::Point point;
        Vector3d rotated_point = R_active * Vector3d(-x, 0.0, a_ * (cosh(x / a_) - 1));
        point.x = rotated_point.x();
        point.y = rotated_point.y();
        point.z = rotated_point.z();
        points.push_back(point);
    }

    // Publish marker list
    marker_.header.stamp = ros::Time::now();
    marker_.points = points;
    marker_pub_.publish(marker_);
}

void TetherDynamics::calculateLengths()
{
//    L_ = atanh(X_ / a_) * Vector2d(1, sinh(X_ / a_)).norm(); // NOPE...

    // Numerically integrate the arc length to get the total tether length
    // using Quadratic interpolation version of Simpson's rule
    L_ = 0.0;
    auto f = [](double x, double a) { return Vector2d(1, sinh(x / a)).norm();};
    double dx = X_ / (NUM_POINTS - 1);
    for (int i = 1; i < NUM_POINTS; i++)
    {
        double x_prev = (i - 1) * dx;
        double x = i * dx;
        L_ += dx / 6.0 * (f(x_prev, a_) + 4 * f((x_prev + x) / 2.0, a_) + f(x, a_));
    }

    // Simplified length formula for control purposes
    L_euclid_ = Vector2d(X_, Y_).norm();
}

void TetherDynamics::setWhiteLine()
{
    marker_.color.r = 1.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;
}

void TetherDynamics::setRedLine()
{
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;
}

//void TetherDynamics::calculateTorque()
//{
//    Torque_ = moment_arm_.cross(Force_);
////    std::cout << Torque_.transpose() << std::endl;
//}

void TetherDynamics::calculateBaselines()
{
    // Calculate unrotated forces in the boat frame
    double F_yI = lambda_ * g_ * L_;
    double gamma = atan(sinh(X_ / a_));
    double F_xI = -F_yI / tan(gamma);

    // Rotate forces and re-express in the UAV frame
    Matrix3d R_active = Theta_.R().transpose();
    Force_ = R_active * Vector3d(-F_xI, 0., F_yI);
    Force_ = q_BOAT_UAV_.R() * Force_;

    // Calculate resulting moment from moment arm
    Torque_ = moment_arm_.cross(Force_);
}

void TetherDynamics::computeControl()
{
    /// TENSION CONTROL

    // Calculate unit vector of tether line in UAV frame
    Vector3d p_boat2UAV_boat(tf_BOAT_UAV_.transform.translation.x,
                             tf_BOAT_UAV_.transform.translation.y,
                             tf_BOAT_UAV_.transform.translation.z);
    Vector3d unit = q_BOAT_UAV_.R() * p_boat2UAV_boat.normalized();

    // Calculate feed forward term
    Vector3d motor_forces(motor_wrench_.force.x, motor_wrench_.force.y, motor_wrench_.force.z);
    double feed_forward = (motor_forces - Force_).dot(unit);

    // Calculate UAV velocity along the unit
    double vel_error_unit = UAV_vel_.dot(unit);

    // Calculate the control term
    double control = tether_pid_.run(0.01, L_euclid_, L_lim_, true, vel_error_unit);

    // Add control terms to final force
    Force_ += unit * (control - feed_forward);

    /// YAW CONTROL
    /// pretty simplistic; should work well as long as the UAV is somewhat upright...

    // Calculate desired yaw angle of the UAV in the NED frame
    Vector3d p_UAV2boat_NED = q_NED_UAV_.inverse().rotp(-unit);
    psi_d_ = Quatd::from_two_unit_vectors(e1, p_UAV2boat_NED).yaw();

    // Calculate actual yaw angle in the NED frame (should be same as body frame value,
    // because of how yaw is defined)
    psi_a_ = q_NED_UAV_.yaw();

    // Calculate feed forward term
    double yaw_ff = motor_wrench_.torque.z;

    // Calculate the control term
    double yaw_control = tether_torque_pid_.run(0.01, psi_a_, psi_d_, true);

    // Add control terms to final torque
    Torque_(2, 0) += yaw_control - yaw_ff;
}

void TetherDynamics::resetWrench()
{
    Force_.setZero();
    Torque_.setZero();
}

void TetherDynamics::motorWrenchCallback(const geometry_msgs::Wrench &msg)
{
    motor_wrench_ = msg;
}

void TetherDynamics::uavStateCallback(const rosflight_sil::ROSflightSimState &msg)
{
    UAV_vel_.x() = msg.vel.x;
    UAV_vel_.y() = msg.vel.y;
    UAV_vel_.z() = msg.vel.z;
}

} // end namespace tether_dynamics
