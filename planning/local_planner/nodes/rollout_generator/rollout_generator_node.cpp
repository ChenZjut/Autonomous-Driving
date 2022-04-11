/*
 * @Description: local planner implement
 * @Author: ubuntu
 * @Date: 2021/6/4 下午3:38
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/6/4 下午3:38
 * @Version 1.0
 */


#include "rollout_generator/rollout_generator.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rollout_generator_node");
    RolloutGeneratorNS::RolloutGenerator app;
    app.run();
    return 0;
}
