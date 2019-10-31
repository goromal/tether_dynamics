#pragma once
#include <ros/ros.h>
#include "utils/xform.h"
#include "utils/control.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <rosflight_msgs/ROSflightSimState.h>
#include <Eigen/Core>

#define ERROR_TOLERANCE 0.0001
#define NUM_POINTS 30
#define MAX_ITER 50

using namespace transforms;
using namespace Eigen;

namespace tether_dynamics {

class TetherDynamics
{
public:
    TetherDynamics();

private:
    void onUpdate(const ros::TimerEvent &event);
    bool estimateCurvature();
    void drawPoints();
    void calculateBaselines();
    void computeControl();
    void calculateLengths();
    void setWhiteLine();
    void setRedLine();
//    void calculateTorque();
    void resetWrench();
    void motorWrenchCallback(const geometry_msgs::Wrench &msg);
    void uavStateCallback(const rosflight_msgs::ROSflightSimState &msg);

    geometry_msgs::TransformStamped tf_BOAT_UAV_;
    Quatd q_BOAT_UAV_;
    geometry_msgs::TransformStamped tf_NED_UAV_;
    Quatd q_NED_UAV_;

    double lambda_;
    double g_;
    double kp_;
    double ki_;
    double kd_;
    control::PID<double> tether_pid_;
    double kpT_;
    double kiT_;
    double kdT_;
    double psi_d_;
    double psi_a_;
    control::PID<double> tether_torque_pid_;

    double X_;
    double Y_;
    double L_;
    double L_lim_;
    double L_euclid_;
    double L_buff_;
    Quatd Theta_;
    double a_;
    double x_moment_arm_;
    Vector3d moment_arm_;

    Vector3d Force_;
    Vector3d Torque_;

    visualization_msgs::Marker marker_;
    geometry_msgs::Wrench wrench_;
    geometry_msgs::Wrench motor_wrench_;
    Vector3d UAV_vel_;
    ros::Subscriber uav_state_sub_;
    ros::Subscriber motor_wrench_sub_;
    ros::Publisher wrench_pub_;
    ros::Publisher marker_pub_;

    ros::NodeHandle nh_;
    ros::Timer timer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
};

} // end namespace tether_dynamics
