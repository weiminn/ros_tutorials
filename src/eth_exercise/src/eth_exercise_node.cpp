#include <ros/ros.h>
#include "eth_exercise/eth_exercise.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "husky_highlevel_controller");
    ros::NodeHandle nodeHandle("~");

    husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle, false);

    ros::spin();
    return 0;
}