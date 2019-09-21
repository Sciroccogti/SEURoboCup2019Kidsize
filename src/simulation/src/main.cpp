#include "unirobot.hpp"
#include "ros/master.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc<2){
      cout<<"no robot name set!"<<endl;
      return 0;
    }
    //setenv("ROS_MASTER_URI", "http://localhost:11311", 0);
    string name="redrobot";
    name=string(argv[1]);
    setenv("WEBOTS_ROBOT_NAME", name.c_str(), 0);
    ros::init(argc, argv, name);
    if (!ros::master::check()) {
      ROS_FATAL("Failed to contact master at %s. Please start ROS master and restart this controller.", getenv("ROS_MASTER_URI"));
      exit(EXIT_SUCCESS);
    }
    shared_ptr<UniRobot> robot = make_shared<UniRobot>(name);
    robot->run();
    return 0;
}