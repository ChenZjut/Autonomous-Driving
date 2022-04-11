/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/7/2 下午2:46
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/7/2 下午2:46
 * @Version 1.0
 */

#include "pure_pursuit.h"
#include <ros/ros.h>

namespace waypoint_follower {
// Constructor
PurePursuitNode::PurePursuitNode()
        : private_nh_("~")
        , LOOP_RATE_(10)
        , curvature_MIN_(1 / 9e10)
        , is_waypoint_set_(false)
        , is_pose_set_(false)
        , current_linear_velocity_(0)
        , command_linear_velocity_(0)
        , next_waypoint_number_(-1)
        , lookahead_distance_(0)
        , is_last_point(false)
        , bPath(false)
{
    // 设置了订阅，发布的话题消息， 读取了launch文件中的配置变量
    initForROS();
}

// Destructor
PurePursuitNode::~PurePursuitNode()
{
}

void PurePursuitNode::initForROS()
{
    // ros parameter settings
    private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(false));
    private_nh_.param("wheel_base", wheel_base_, double(0.5));
    private_nh_.param("lookahead_distance_ratio", lookahead_distance_ratio_, double(4.0)); //假设速度的单位是m/s, 1M/S的预瞄距离是4m
    private_nh_.param("minimum_lookahead_distance_", minimum_lookahead_distance_, double(2.5));
    private_nh_.param("const_lookahead_distance_", const_lookahead_distance_, double(4));
    private_nh_.param("const_velocity_", const_velocity_, double(1)); //1m/s
    private_nh_.param("is_const_lookahead_dis", is_const_lookahead_dis_, true);
    private_nh_.param("is_const_speed_command_", is_const_speed_command_, true);
    private_nh_.param("is_yunleCar", is_yunleCar, true);

    // setup subscriber
    sub_Path = nh_.subscribe("local_trajectories", 10, &PurePursuitNode::callbackFromWayPoints, this);
    sub_CurrentPose = nh_.subscribe("current_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);
    sub_CommandVelocity = nh_.subscribe("current_velocity", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);

    // setup publisher
    pub_ctl = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_yunle_control = nh_.advertise<can_msgs::ecu>("ecu", 10);

    // debug tool
    pub_NextWaypoint = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
    pub_NextTargetPosition = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
    pub_SearchRadius = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
    pub_TrajectoryCircle = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
    pub_AngularGravity = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
    pub_DeviationCurrentPosition = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
    pub_linear_viz = nh_.advertise<std_msgs::Float32>("linear_velocity_viz", 1);
}

void PurePursuitNode::run()
{
    ros::Rate loop_rate(LOOP_RATE_);
    while (ros::ok()) {
        ros::spinOnce();
        if (!is_pose_set_ || !bPath) {
            if (!is_pose_set_) {
                ROS_WARN("Waiting for current_pose topic ...");
            }
            if (!bPath) {
                ROS_WARN("Waiting for Path topic ...");
            }
            loop_rate.sleep();
            continue;
        }
        lookahead_distance_ = computeLookaheadDistance();

        double curvature = 0;
        bool can_get_curvature = computeCurvature(&curvature);
        //ROS_INFO_STREAM("curvature:" << can_get_curvature);
        publishControlCommandStamped(can_get_curvature, curvature);

        pub_NextWaypoint.publish(displayNextWaypoint(m_Path.poses.at(next_waypoint_number_).pose.position));
        pub_SearchRadius.publish(displaySearchRadius(current_pose_.position, lookahead_distance_));
        pub_NextTargetPosition.publish(displayNextTarget(next_target_position_));
        pub_TrajectoryCircle.publish(displayTrajectoryCircle(generateTrajectoryCircle(next_target_position_, current_pose_)));
        std_msgs::Float32 angular_gravity_msg;
        angular_gravity_msg.data = computeAngularGravity(computeCommandVelocity(), curvature);
        pub_AngularGravity.publish(angular_gravity_msg);

        publishDeviationCurrentPosition(current_pose_.position, m_Path);

        loop_rate.sleep();
    }
}

double PurePursuitNode::computeLookaheadDistance() const
{
    double maximum_lookahead_distance = current_linear_velocity_ * 10;
    double ld = current_linear_velocity_ * lookahead_distance_ratio_;
    return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ : ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
}

double PurePursuitNode::computeAngularGravity(double velocity, double kappa)
{
    const double gravity = 9.80665;
    return (velocity * velocity) / (1.0 / kappa * gravity);
}

double PurePursuitNode::computeCommandVelocity() const
{
    if (is_const_speed_command_ == true)
        return const_velocity_;

    return command_linear_velocity_;
}

void PurePursuitNode::publishVelocityViz(const geometry_msgs::Twist &msg)
{
    std_msgs::Float32 fl;
    fl.data = msg.linear.x * 3.6;
    pub_linear_viz.publish(fl);
}

void PurePursuitNode::publishDeviationCurrentPosition(const geometry_msgs::Point &point, const nav_msgs::Path &path)
{
    if (path.poses.size() < 3)
    {
        return;
    }

    double a, b, c;
    getLinearEquation(path.poses.at(2).pose.position, path.poses.at(1).pose.position, &a, &b, &c);

    std_msgs::Float32 msg;
    msg.data = getDistanceBetweenLineAndPoint(point, a, b, c);
    pub_DeviationCurrentPosition.publish(msg);
}

void PurePursuitNode::publishControlCommandStamped(const bool &can_get_curvature, const double &curvature) const
{
    if (is_yunleCar) {
        can_msgs::ecu ecu_ctl;
        ecu_ctl.motor = can_get_curvature ? computeCommandVelocity() : 0;

        double steer = atan(wheel_base_ * curvature);

        if(steer > 0){
            steer = steer / 0.06 * 140;
        }else{
            steer = steer / 0.06 * 93;
        }
        ecu_ctl.steer = can_get_curvature ? steer : 0;

        ecu_ctl.shift = ecu_ctl.SHIFT_D;
        if (is_last_point) {
            ecu_ctl.motor = 0;
            ecu_ctl.shift = ecu_ctl.SHIFT_N;
        }
        pub_yunle_control.publish(ecu_ctl);
    } else {
        geometry_msgs::Twist control_msg;
        control_msg.linear.x = can_get_curvature ? computeCommandVelocity() : 0.01;
        control_msg.angular.z = can_get_curvature ? atan(wheel_base_ * curvature) : 0;
        if (is_last_point) {
            control_msg.linear.x = 0;
        }
        pub_ctl.publish(control_msg);
    }
}

bool PurePursuitNode::computeCurvature(double *output_curvature)
{
    getNextWaypoint();
    if (next_waypoint_number_ == -1) {
        ROS_INFO("lost next waypoint");
        return false;
    }
    bool is_valid_curve = false;
    int path_size = utility.getSize(m_Path);
    for (int i = 0; i < path_size; i++)
    {
        if (getPlaneDistance(m_Path.poses.at(i).pose.position, current_pose_.position) > minimum_lookahead_distance_)
        {
            is_valid_curve = true;
            break;
        }
    }

    // if is_linear_interpolation_ is false or next waypoint is first or last
    if (!is_linear_interpolation_ || next_waypoint_number_ == 0 || next_waypoint_number_ == (static_cast<int>(current_waypoints_.size() - 1))) {
        next_target_position_ = m_Path.poses.at(next_waypoint_number_).pose.position;
        *output_curvature = calcCurvature(next_target_position_);
        return true;
    }

    // linear interpolation and calculate angular velocity
    const bool interpolation = interpolateNextTarget(next_waypoint_number_, &next_target_position_);
    if (!interpolation) {
        ROS_INFO_STREAM("lost target! ");
        return false;
    }
    *output_curvature = calcCurvature(next_target_position_);
    return true;
}

void PurePursuitNode::getNextWaypoint()
{
    const int path_size = m_Path.poses.size();

    if (path_size == 0) {
        next_waypoint_number_ = -1;
        return;
    }

    while (true) {
        for (int i = 0; i < path_size; i++) {
            if (i == (path_size - 1)) {
                ROS_INFO("search waypoint is the last");
                next_waypoint_number_ = i;
                return;
            }
            if (utility.getPlaneDistance(m_Path.poses.at(i).pose.position, current_pose_.position) > lookahead_distance_)
            {
               next_waypoint_number_ = i;
               return;
            }
        }
    }

    next_waypoint_number_ = -1;
    return;
}

// linear interpolation of next target
bool PurePursuitNode::interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target)
{
    const double ERROR = pow(10, -5); // 0.00001

    int path_size = m_Path.poses.size();
    if (next_waypoint == path_size - 1) {
        *next_target = m_Path.poses.at(next_waypoint).pose.position;
        return true;
    }
    double search_radius = lookahead_distance_;
    geometry_msgs::Point zero_p;
    geometry_msgs::Point end = m_Path.poses.at(next_waypoint).pose.position;
    geometry_msgs::Point start = m_Path.poses.at(next_waypoint - 1).pose.position;

    // let the linear equation be "ax + by + c = 0"
    // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(x1 - x2" ,c = "(y1-y2)*x1 + (x2-x1)*y1"
    // 这一步就是根据当前计算出来的预瞄点和前一个点，得到一个直线表达式
    double a = 0;
    double b = 0;
    double c = 0;
    double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
    if (!get_linear_flag)
        return false;

    // let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
    // the distance  "d" between the foot of a perpendicular line and the center of circle is ...
    // distance between target 1 and target2 in 2-D
    //    | a * x0 + b * y0 + c |
    // d = -------------------------------
    //          √( a~2 + b~2)
    // 这一步就是计算当前位置到前面拟合出来的直线的“点到直线的距离”
    double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);

    // ROS_INFO("a : %lf ", a);
    // ROS_INFO("b : %lf ", b);
    // ROS_INFO("c : %lf ", c);
    // ROS_INFO("distance : %lf ", d);
    if (d > search_radius)
        return false;

    // unit vector of point 'start' to point 'end'
    // 求取这两个点的向量
    tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
    // Normalize this vector x^2 + y^2 + z^2 = 1.
    // 就是三维矢量的方向不变，但是模长变为1,变成单位向量了
    tf::Vector3 unit_v = v.normalize();

    // normal unit vectors of v
    // 将上面的单位向量分别顺时针和逆时针旋转90度
    tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90); // rotate to counter clockwise 90 degree
    tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90); // rotate to counter clockwise -90 degree

    // the foot of a perpendicular line
    // 现在的h1的坐标就是 当前车辆位置到前面拟合直线的垂足的坐标
    // 他的做法是将车辆当前坐标朝着垂足的方向移动了垂线的距离（当前位置到拟合直线的距离）
    geometry_msgs::Point h1;
    h1.x = current_pose_.position.x + d * unit_w1.getX();
    h1.y = current_pose_.position.y + d * unit_w1.getY();
    h1.z = current_pose_.position.z;

    geometry_msgs::Point h2;
    h2.x = current_pose_.position.x + d * unit_w2.getX();
    h2.y = current_pose_.position.y + d * unit_w2.getY();
    h2.z = current_pose_.position.z;

    // 为什么要写两个垂足，这是因为拟合直线的斜率不一样，情况是不同的，
    // 如果拟合直线的斜率是负，那么h1垂足是在直线上，保留，h2舍弃
    // 如果直线的的斜率是正的，那么h2垂足是在直线上，保留，h1舍弃
    // check which of two foot of a perpendicular line is on the line equation
    geometry_msgs::Point h;
    if (fabs(a * h1.x + b * h1.y + c) < ERROR) {
        h = h1;
        //   ROS_INFO("use h1");
    } else if (fabs(a * h2.x + b * h2.y + c) < ERROR) {
        //   ROS_INFO("use h2");
        h = h2;
    } else {
        return false;
    }

    // get intersection[s]
    // 这里是以车辆当前位置画一个圈，圈的半径就是前视距离
    // 如果计算出来的 当前位置到拟合直线的距离 大于这个圈，即拟合直线和圈没有交点，那么情况错误，函数直接返回false
    // 如果有一个交点，那么这个交点就是我们的预瞄点，如果有两个交点，那么就选取教前方那个点。
    // if there is a intersection
    if (d == search_radius) {
        *next_target = h;
        return true;
    } else {
        // if there are two intersection
        // get intersection in front of vehicle
        double s = sqrt(pow(search_radius, 2) - pow(d, 2));
        geometry_msgs::Point target1;
        target1.x = h.x + s * unit_v.getX();
        target1.y = h.y + s * unit_v.getY();
        target1.z = current_pose_.position.z;

        geometry_msgs::Point target2;
        target2.x = h.x - s * unit_v.getX();
        target2.y = h.y - s * unit_v.getY();
        target2.z = current_pose_.position.z;

        // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
        // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
        // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

        // check intersection is between end and start
        double interval = getPlaneDistance(end, start);
        if (getPlaneDistance(target1, end) < interval) {
            // ROS_INFO("result : target1");
            *next_target = target1;
            return true;
        } else if (getPlaneDistance(target2, end) < interval) {
            // ROS_INFO("result : target2");
            *next_target = target2;
            return true;
        } else {
            // ROS_INFO("result : false ");
            return false;
        }
    }
}

double PurePursuitNode::calcCurvature(geometry_msgs::Point target)
{
    // 计算曲率
    // 曲率 = 2 * el / ld^2  el是当前位置和目标预瞄点的横向误差
    double curvature;
    double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
    double numerator = -2 * calcRelativeCoordinate(target, current_pose_).y;

    if (denominator != 0)
        curvature = numerator / denominator;
    else {
        if (numerator > 0)
            curvature = curvature_MIN_;
        else
            curvature = -curvature_MIN_;
    }
    return curvature;
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    current_pose_ = msg->pose;
    is_pose_set_ = true;
}

void PurePursuitNode::callbackFromCurrentVelocity(const geometry_msgs::Twist &msg)
{
    static double kp = 0.5;
    static double pre_vel = 0; //初始状态停车 0 km/h
    std::cout << "pre_vel: " << pre_vel << std::endl;
    double cur_vel = pre_vel + kp * (msg.linear.x - pre_vel);
    std::cout << "cur_vel: " << cur_vel << std::endl;
    current_linear_velocity_ = cur_vel;
    command_linear_velocity_ = cur_vel;
    pre_vel = cur_vel;
    geometry_msgs::Twist velocity;
    velocity.linear.x = cur_vel;
    std::cout << "pre_vel_1: " << velocity.linear.x << std::endl;
//    static double pre_time = ros::Time::now().toSec();
//    static double pre_vel = msg.linear.x;
//    double cur_time = ros::Time::now().toSec();
//    double cur_vel = msg.linear.x;
//    double diff_time = cur_time - pre_time;
//    if (diff_time > 0.0001) {
//        double acc = (cur_vel - pre_vel) / diff_time;
//        if (std::abs(acc) > 1.0) {
//            cur_vel = pre_vel + acc * diff_time;
//        }
//    }
//    else {
//        ROS_ERROR("error in callbackFromVelocity");
//    }
//    current_linear_velocity_ = cur_vel;
//    command_linear_velocity_ = cur_vel;
//    ROS_INFO("vel: %f", cur_vel);
//    pre_time = cur_time;
//    pre_vel = cur_vel;
    publishVelocityViz(velocity);
}

void PurePursuitNode::callbackFromWayPoints(const nav_msgs::PathPtr &msg)
{
    if (msg->poses.empty()) {
        current_linear_velocity_ = 0;
    }

    m_Path = * msg;
    bPath = true;
}

geometry_msgs::Point PurePursuitNode::calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
    // calculation relative coordinate of point from current_pose frame
    tf::Transform inverse;
    tf::poseMsgToTF(current_pose, inverse);
    tf::Transform transform = inverse.inverse();

    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = transform * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);

    return tf_point_msg;
}

double PurePursuitNode::getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
{
    // distance between target 1 and target2 in 2-D
    tf::Vector3 v1 = point2vector(target1);
    v1.setZ(0);
    tf::Vector3 v2 = point2vector(target2);
    v2.setZ(0);
    return tf::tfDistance(v1, v2);
}

double PurePursuitNode::getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c)
{
    double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

    return d;
}

tf::Vector3 PurePursuitNode::rotateUnitVector(tf::Vector3 unit_vector, double degree)
{
    tf::Vector3
            w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
               sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
    tf::Vector3 unit_w1 = w1.normalize();
    return unit_w1;
}

tf::Vector3 PurePursuitNode::point2vector(geometry_msgs::Point point)
{
    tf::Vector3 vector(point.x, point.y, point.z);
    return vector;
}

bool PurePursuitNode::getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double* a, double* b, double* c)
{
    //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
    double sub_x = fabs(start.x - end.x);
    double sub_y = fabs(start.y - end.y);
    double error = pow(10, -5); // 0.00001

    if (sub_x < error && sub_y < error) {
        ROS_INFO("two points are the same point!!");
        return false;
    }

    *a = end.y - start.y;
    *b = (-1) * (end.x - start.x);
    *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

    return true;
}

}
