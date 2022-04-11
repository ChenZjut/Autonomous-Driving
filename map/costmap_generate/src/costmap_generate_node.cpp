/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/6/28 上午9:29
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/6/28 上午9:29
 * @Version 1.0
 */
#include <costmap_generate.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "costmap_generate");
    GenCostmap::UpdateCostmap app;
    app.run();
    return 0;
}