#ifndef __UNIROBOT_HPP
#define __UNIROBOT_HPP

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/Accelerometer.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <simulation/ImageInfo.h>
#include <simulation/Task.h>
#include <simulation/GameData.h>
#include <simulation/Stand.h>

#include "walk/IKWalk.hpp"

struct UpperOutputs{
    double left_shoulder, left_elbow;
    double right_shoulder, right_elbow;
    double head_yaw, head_pitch;
};

class UniRobot: public webots::Robot
{
public:
    UniRobot(std::string name);
    ~UniRobot();
    void run();

private:
    int mTimeStep;
    int totalTime;
    std::string mName;
    ros::NodeHandle *mNode;
    ros::ServiceServer mImageInfoServer;
    ros::Publisher mImagePublisher;
    ros::Subscriber mHeadSubscriber;
    ros::Subscriber mTaskSubscriber;
    ros::Subscriber mGdataSubscriber;
    ros::Subscriber mStandSubscriber;

    std::list<simulation::Task> mTasks;
    simulation::GameData mGdata;
    simulation::Stand mStand;

    void HeadChanged(const geometry_msgs::Point::ConstPtr &p);
    void TaskUpdate(const simulation::Task::ConstPtr &p);
    void GdataUpdate(const simulation::GameData::ConstPtr &p);
    void StandUpdate(const simulation::Stand::ConstPtr &p);

    bool ImageInfoService(simulation::ImageInfo::Request &req, 
        simulation::ImageInfo::Response &res);
    
    void PublishImage();

    void runWalk(const Rhoban::IKWalkParameters& params, double timeLength,
                  double& phase, double& time);
    void stopWalk(bool &isWalking, double &phase, double &time);
    float ComputeVtorso(float theta);
    float ComputeVrightleg(float theta, float x, float y, float z);

    void ActionRightFootPos(float x, float y, float z);
    void ActionRightKick(float x, float y, float z, float alpha);
    void ActionReady();

    void SetPositions();
    void GetPositions(Rhoban::IKWalkOutputs &lOuts, UpperOutputs &uOuts);
    
    int myStep();
    void wait(int ms);

    webots::Camera *mCamera;
    webots::Accelerometer *mAccelerometer;

    UpperOutputs mUOuts = {M_PI/4, -2*M_PI/3, M_PI/4, -2*M_PI/3, 0, -M_PI/4};
    Rhoban::IKWalkOutputs mLOuts;
};


#endif
