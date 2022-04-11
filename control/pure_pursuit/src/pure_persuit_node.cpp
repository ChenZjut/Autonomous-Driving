/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/6/9 下午8:02
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/6/9 下午8:02
 * @Version 1.0
 */
#include "pure_persuit.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    waypoint_follower::PurePursuitNode ppn;
    ppn.run();
    return 0;
}