/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/6/28 上午9:13
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/6/28 上午9:13
 * @Version 1.0
 */
#ifndef COSTMAP_GENERATE_H
#define COSTMAP_GENERATE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_generate/GenCostmapConfig.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <string.h>
#include <cmath>
#include <pthread.h>
#include <limits>
#include <vector>
#include <algorithm>



namespace GenCostmap {
class UpdateCostmap {
private:
    typedef struct {
        float rad;
        float dist;
        int x;
        int y;
    } fake_point;

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    std::string NAME_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_obstacle_pc_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_pose_;
    nav_msgs::OccupancyGrid msg_map;
    geometry_msgs::PoseStamped msg_pose_;
    tf::TransformListener tf_listener_;

    void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);
    void obstacleCB(const sensor_msgs::PointCloud2ConstPtr& msg);
    void mapCB(const nav_msgs::OccupancyGridConstPtr& msg);
    void cfgCB(const costmap_generate::GenCostmapConfig& config, uint32_t level);
    void getLine(const int &s_x, const int &s_y, const int& e_x, const int& e_y, std::vector<std::pair<int, int>> &points);

    ros::Publisher pub_costmap_;
    ros::Publisher pub_viz_;
    nav_msgs::OccupancyGrid msg_update_map_;

    // params
    std::string param_frame_map_;
    std::string param_frame_laser_;
    std::string param_frame_base_;
    double param_max_range_;
    double param_angle_resolution_;
    int param_decay_;

    tf::StampedTransform tf_b2l_;
    bool map_set_;
    bool pose_set_;
    int max_range_pixel_;
    int fake_num_rays_;
    std::vector<fake_point> fake_scan_;
    std::vector<fake_point> fake_scan_t;
    double fake_angle_resolution_;

    pthread_mutex_t mutex_;
public:
    UpdateCostmap();
    ~UpdateCostmap();

    void initROS();
    void run();

};
}

#endif //COSTMAP_GENERATE_H
