/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/6/9 下午7:55
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/6/9 下午7:55
 * @Version 1.0
 */

#include "pure_persuit.h"

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
{
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
    private_nh_.param("is_const_lookahead_dis_", is_const_lookahead_dis_, bool(false));
    private_nh_.param("is_const_speed_command_", is_const_speed_command_, bool(false));
    private_nh_.param("is_yunleCar", is_yunleCar, bool(false));

    // setup subscriber
     sub_lane = nh_.subscribe("best_local_trajectories", 10, &PurePursuitNode::callbackFromWayPoints, this);
    //sub_lane = nh_.subscribe("global_path", 10, &PurePursuitNode::callbackFromWayPoints, this);
    sub_currentpose = nh_.subscribe("/current_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);
    sub_speed = nh_.subscribe("vehicle_status", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);

    // setup publisher
    pub_ctl = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    //pub_target = nh_.advertise<visualization_msgs::MarkerArray>("target_waypoint", 10);
    pub_path = nh_.advertise<nav_msgs::Path>("followed_path", 10);
    //pub_car_model = nh_.advertise<visualization_msgs::Marker>("car_model", 10);
    pub_yunle_control = nh_.advertise<can_msgs::ecu>("ecu", 50);
    pub_next_waypoint = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
    pub_next_target = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
    pub_circle = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);

}

void PurePursuitNode::run() {
    ROS_INFO_STREAM("pure pursuit start");
    ros::Rate loop_rate(LOOP_RATE_);
    while (ros::ok()) {
        ros::spinOnce();
        if (!is_pose_set_ || !is_waypoint_set_) {
            //ROS_WARN("Necessary topics are not subscriber yet");
            loop_rate.sleep();
            continue;
        }

        lookahead_distance_ = computeLookaheadDistance();
        ROS_INFO("lookahead_distance_ %lf", lookahead_distance_);

        double curvature = 0;
        bool can_get_curvature = computeCurvature(&curvature);
        publishControlCommandStamped(can_get_curvature, curvature);

        // for visualization with Rviz
        pub_next_waypoint.publish(displayNextWaypoint(current_waypoints_.at(next_waypoint_number_).pose.pose.position));
        pub_next_target.publish(displayNextTarget(next_target_position_));
        pub_circle.publish(displayTrajectoryCircle(generateTrajectoryCircle(next_target_position_, current_pose_)));

        is_pose_set_ = false;
        is_waypoint_set_ = false;

        loop_rate.sleep();
    }
}

bool PurePursuitNode::computeCurvature(double *output_curvature)
{
    //search next waypoint
    getNextWaypoint();
    if (next_waypoint_number_ == -1) {
        ROS_INFO("lost next waypoint");
        return false;
    }
    // check whether curvature is valid or not
    bool is_valid_curve = false;
    for (const auto &el : current_waypoints_) {
        double dis = getPlaneDistance(el.pose.pose.position, current_pose_.position);
        if (dis > minimum_lookahead_distance_) {
            is_valid_curve = true;
            break;
        }
    }
    if (!is_valid_curve) {
        return false;
    }

    // if is_linear_interpolation_ is false or next waypoint is first or last
    if (!is_linear_interpolation_ || next_waypoint_number_ == 0 || next_waypoint_number_ == (static_cast<int>(current_waypoints_.size() - 1))) {
        next_target_position_ = current_waypoints_.at(next_waypoint_number_).pose.pose.position;
        *output_curvature = calcCurvature(next_target_position_);
        return true;
    }

    // linear interpolation and calculate angular velocity
    bool interpolation = interpolateNextTarget(next_waypoint_number_, &next_target_position_);
    if (!interpolation) {
        ROS_INFO_STREAM("lost target! ");
        return false;
    }
    *output_curvature = calcCurvature(next_target_position_);
    return true;
}

void PurePursuitNode::getNextWaypoint()
{
    int path_size = static_cast<int>(current_waypoints_.size());

    // if waypoints are not given, do nothing.
    if (path_size == 0) {
        next_waypoint_number_ = -1;
        return;
    }

    while (true) {
        // look for the next waypoint
        for (int i = 0; i < path_size; i++) {
            // if there exits an effective waypoint
            double dis = getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position);
            if (dis > lookahead_distance_) {
                next_waypoint_number_ = i;
                return;
            }
        }
    }
    // if this program reaches here , it means we lost the waypoint!
    next_waypoint_number_ = -1;
    return;
}

// linear interpolation of next target
bool PurePursuitNode::interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target)
{
    const double ERROR = pow(10, -5); // 0.00001

    int path_size = static_cast<int>(current_waypoints_.size());
    if (next_waypoint == path_size - 1) {
        *next_target = current_waypoints_.at(next_waypoint).pose.pose.position;
        return true;
    }
    double search_radius = lookahead_distance_;
    geometry_msgs::Point zero_p;
    geometry_msgs::Point end = current_waypoints_.at(next_waypoint).pose.pose.position;
    geometry_msgs::Point start = current_waypoints_.at(next_waypoint - 1).pose.pose.position;

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
    ROS_INFO("curvature: %lf", curvature);
    return curvature;
}

void PurePursuitNode::publishControlCommandStamped(const bool &can_get_curvature, const double &curvature) const
{
    if (is_yunleCar) {
        can_msgs::ecu ecu_ctl;
        ecu_ctl.motor = can_get_curvature ? computeCommandVelocity() : 0;

        double steer = atan(wheel_base_ * curvature);
        ROS_INFO("steer: %lf", steer);
        if (steer > 0) {
            steer = steer / 0.06 * 140;
        }else {
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
        control_msg.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
        control_msg.angular.z = can_get_curvature ? atan(wheel_base_ * curvature) : 0;
        ROS_INFO("Angular: %lf", control_msg.angular.z);
        if (is_last_point) {
            control_msg.linear.x = 0;
        }
        pub_ctl.publish(control_msg);
    }
}

double PurePursuitNode::computeLookaheadDistance() const
{
    if (is_const_lookahead_dis_ == true)
        return const_lookahead_distance_;

    // 通过速度乘系数得到的前视距离不小于最小前视距离，不大于当前速度×10
    ROS_INFO("current_linear_velocity_: %lf", current_linear_velocity_);
    double maximum_lookahead_distance = current_linear_velocity_ * 10;
    double ld = current_linear_velocity_ * lookahead_distance_ratio_;
    return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ : ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;

}

double PurePursuitNode::computeCommandVelocity() const
{
    if (is_const_speed_command_ == true)
        return const_velocity_;

    return command_linear_velocity_;
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    // 读取了当前的车辆pose消息，里面有xyz和四元数
    current_pose_ = msg->pose;
    is_pose_set_ = true;
    ROS_INFO("current pose is set");
}

void PurePursuitNode::callbackFromCurrentVelocity(const can_msgs::vehicle_status &msg)
{
    current_linear_velocity_ = msg.cur_speed;
}

void PurePursuitNode::callbackFromWayPoints(const smartcar_msgs::LaneConstPtr& msg)
{
    command_linear_velocity_ = 4; //  1m/s

    current_waypoints_ = msg->waypoints;

    is_waypoint_set_ = true;
    ROS_INFO("Waypoints is set");
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

// distance between target 1 and target2 in 2-D
double PurePursuitNode::getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
{
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

geometry_msgs::Point PurePursuitNode::calcAbsoluteCoordinate(geometry_msgs::Point point_msg,
    geometry_msgs::Pose current_pose)
{
    tf::Transform inverse;
    tf::poseMsgToTF(current_pose, inverse);

    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = inverse * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);
    return tf_point_msg;
}

geometry_msgs::Point PurePursuitNode::rotatePoint(geometry_msgs::Point point, double degree)
{
    geometry_msgs::Point rotate;
    rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
    rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

    return rotate;
}

visualization_msgs::Marker PurePursuitNode::displayNextWaypoint(geometry_msgs::Point position)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "next_waypoint_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = position;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.frame_locked = true;
    return marker;
}

visualization_msgs::Marker PurePursuitNode::displayNextTarget(geometry_msgs::Point target)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "next_target_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = target;
    std_msgs::ColorRGBA green;
    green.a = 1.0;
    green.b = 0.0;
    green.r = 0.0;
    green.g = 1.0;
    marker.color = green;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.frame_locked = true;
    return marker;
}

double PurePursuitNode::calcRadius(geometry_msgs::Point target, geometry_msgs::Pose current_pose)
{
    double radius;
    double denominator = 2 * calcRelativeCoordinate(target, current_pose).y;
    double numerator = pow(getPlaneDistance(target, current_pose.position), 2);

    if (denominator != 0)
        radius = numerator / denominator;
    else
        radius = 0;

    // ROS_INFO("radius : %lf", radius);
    return radius;
}

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> PurePursuitNode::generateTrajectoryCircle(geometry_msgs::Point target,
                                                           geometry_msgs::Pose current_pose)
{
    std::vector<geometry_msgs::Point> traj_circle_array;
    double radius = calcRadius(target, current_pose);
    double range = M_PI / 8;
    double increment = 0.01;

    for (double i = 0; i < range; i += increment)
    {
        // calc a point of circumference
        geometry_msgs::Point p;
        p.x = radius * cos(i);
        p.y = radius * sin(i);

        // transform to (radius,0)
        geometry_msgs::Point relative_p;
        relative_p.x = p.x - radius;
        relative_p.y = p.y;

        // rotate -90°
        geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

        // transform to vehicle plane
        geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, current_pose);

        traj_circle_array.push_back(tf_p);
    }

    // reverse vector
    std::reverse(traj_circle_array.begin(), traj_circle_array.end());

    for (double i = 0; i > (-1) * range; i -= increment)
    {
        // calc a point of circumference
        geometry_msgs::Point p;
        p.x = radius * cos(i);
        p.y = radius * sin(i);

        // transform to (radius,0)
        geometry_msgs::Point relative_p;
        relative_p.x = p.x - radius;
        relative_p.y = p.y;

        // rotate -90°
        geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

        // transform to vehicle plane
        geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, current_pose);

        traj_circle_array.push_back(tf_p);
    }

    return traj_circle_array;
}
// display the locus of pure pursuit by markers.
visualization_msgs::Marker PurePursuitNode::displayTrajectoryCircle(std::vector<geometry_msgs::Point> traj_circle_array)
{
    visualization_msgs::Marker traj_circle;
    traj_circle.header.frame_id = "map";
    traj_circle.header.stamp = ros::Time();
    traj_circle.ns = "trajectory_circle_marker";
    traj_circle.id = 0;
    traj_circle.type = visualization_msgs::Marker::LINE_STRIP;
    traj_circle.action = visualization_msgs::Marker::ADD;

    std_msgs::ColorRGBA white;
    white.a = 1.0;
    white.b = 1.0;
    white.r = 1.0;
    white.g = 1.0;
    //
    for (auto el : traj_circle_array)
        for (std::vector<geometry_msgs::Point>::iterator it = traj_circle_array.begin(); it != traj_circle_array.end();
             it++)
        {
            // traj_circle.points.push_back(*it);
            traj_circle.points.push_back(el);
            traj_circle.colors.push_back(white);
        }

    traj_circle.scale.x = 0.1;
    traj_circle.color.a = 0.3;
    traj_circle.color.r = 1.0;
    traj_circle.color.g = 0.0;
    traj_circle.color.b = 0.0;
    traj_circle.frame_locked = true;
    return traj_circle;
}

}