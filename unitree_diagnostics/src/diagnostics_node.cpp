
#include <ros/ros.h>
#include "diagnostics.h"

using namespace unitree_diagnostics;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "reach_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    Diagnostics diagnostics(node, private_nh);
    ros::spin();
}
