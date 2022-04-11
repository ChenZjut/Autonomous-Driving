/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/6/9 下午7:08
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/6/9 下午7:08
 * @Version 1.0
 */
#ifndef PURE_PERSUIT_H
#define PURE_PERSUIT_H

// ROS includes
#include "smartcar_msgs/ControlCommandStamped.h"
#include "smartcar_msgs/Lane.h"
#include "smartcar_msgs/LaneArray.h"
#include <can_msgs/ecu.h>
#include <can_msgs/vehicle_status.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace waypoint_follower {
class PurePursuitNode {
private:
    // handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher pub_ctl;
    ros::Publisher pub_path;
    ros::Publisher pub_yunle_control;
    ros::Publisher pub_next_waypoint;
    ros::Publisher pub_next_target;
    ros::Publisher pub_circle;

    // subscriber
    ros::Subscriber sub_currentpose, sub_lane, sub_speed;

    // constant
    const int LOOP_RATE_; //processing frequency
    const double curvature_MIN_;
    double const_velocity_; //km/h
    double const_lookahead_distance_; //meter

    // variable
    bool is_linear_interpolation_; // interpolation's flag
    bool is_waypoint_set_;
    bool is_pose_set_;
    bool is_const_lookahead_dis_;
    bool is_const_speed_command_;
    double current_linear_velocity_;
    double command_linear_velocity_;
    int next_waypoint_number_;
    geometry_msgs::Point next_target_position_;
    geometry_msgs::Pose current_pose_;
    std::vector<smartcar_msgs::Waypoint> current_waypoints_;
    double wheel_base_;
    double lookahead_distance_;
    double lookahead_distance_ratio_; // 这个是前视距离与当前速度的比例系数，将前视距离与速度关联起来，速度越快，前视觉距离越远
    double minimum_lookahead_distance_; // 最小前视距离，通过速度乘系数得到的前视距离不小于这个值，不大于当前速度×10

    bool is_last_point;
    bool is_yunleCar;
public:
    PurePursuitNode();
    ~PurePursuitNode();

    void run();

    // callbacks
    void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);

    void callbackFromCurrentVelocity(const can_msgs::vehicle_status &msg);

    void callbackFromWayPoints(const smartcar_msgs::LaneConstPtr& msg);

    // initializer
    void initForROS();
    // functions
    visualization_msgs::Marker displayNextWaypoint(geometry_msgs::Point position);

    visualization_msgs::Marker displayNextTarget(geometry_msgs::Point target);

    double calcRadius(geometry_msgs::Point target, geometry_msgs::Pose current_pose);

    // generate the locus of pure pursuit
    std::vector<geometry_msgs::Point> generateTrajectoryCircle(geometry_msgs::Point target,
                                                               geometry_msgs::Pose current_pose);
    // display the locus of pure pursuit by markers.
    visualization_msgs::Marker displayTrajectoryCircle(std::vector<geometry_msgs::Point> traj_circle_array);

    bool computeCurvature(double* output_curvature);

    double calcCurvature(geometry_msgs::Point target);

    bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target);

    void getNextWaypoint();

    void publishControlCommandStamped(const bool& can_get_curvature, const double& curvature) const;

    double computeLookaheadDistance() const;

    double computeCommandVelocity() const;

    geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose);

    double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2);

    double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c);

    tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree);

    bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double* a, double* b, double* c);

    tf::Vector3 point2vector(geometry_msgs::Point point);

    // calculation absolute coordinate of point on current_pose frame
    geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose);

    geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree);

    double deg2rad(double deg)
    {
        return deg * M_PI / 180;
    }
};
}
#endif //PURE_PERSUIT_H
