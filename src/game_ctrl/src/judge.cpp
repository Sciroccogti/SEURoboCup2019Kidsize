#include <ros/ros.h>
#include <memory>
#include <mutex>
#include <geometry_msgs/Point.h>
#include <webots/Supervisor.hpp>
#include <game_ctrl/Score.h>
#include <eigen3/Eigen/Dense>
#include <simulation/GameData.h>
#include <simulation/Stand.h>

using namespace std;
using namespace webots;

extern void GdataUpdate(const simulation::GameData::ConstPtr &p);
extern void StandUpdate(const simulation::Stand::ConstPtr &s);

const int basicTime = 20;
const double ballR=0.07;
const double robotR=0.2;
const double fieldX=4.5+ballR;
const double fieldZ=3.0+ballR;
const double goalX=4.5+ballR;
const double goalZ=0.7+ballR;
const double ballInit[3] = {0.0, ballR, 0.0};

simulation::GameData mGdata;
mutex gdata_mutex;
simulation::Stand mStand;
mutex stand_mutex;

struct RobotInfo
{
    Eigen::Vector3d position;
    Eigen::Vector4d rotate;
};

inline double sign(double v)
{
    return v<0?-1:1;
}

inline double distance(const double *p1, const  double *p2)
{
    double x=p2[0]-p1[0], z=p2[2]-p1[2];
    return sqrt(x*x+z*z);
}

int main(int argc, char **argv)
{
    setenv("WEBOTS_ROBOT_NAME", "judge", 0);
    ros::init(argc, argv, "judge");
    ros::NodeHandle node;
    std::shared_ptr<Supervisor> super = make_shared<Supervisor>();
    Node *ballNode = super->getFromDef("Ball");
    Node *redNode = super->getFromDef("RedRobot");
    Node *blueNode = super->getFromDef("BlueRobot");
    ros::Publisher scorePublisher = node.advertise<game_ctrl::Score>("/judge/score", 1);
    ros::Publisher redPosPublisher = node.advertise<geometry_msgs::Point>("/redrobot/position", 1);
    ros::Publisher bluePosPublisher = node.advertise<geometry_msgs::Point>("/bluerobot/position", 1);
    ros::Subscriber gdataSubscriber = node.subscribe("/gamedata", 1, GdataUpdate);
    ros::Subscriber standSubscriber = node.subscribe("/stand", 1, StandUpdate);

    RobotInfo redInitInfo, blueInitInfo;
    redInitInfo.position = Eigen::Vector3d(-0.75, 0.364, 0.0);
    redInitInfo.rotate = Eigen::Vector4d(1, 0, 0, -M_PI/2);
    blueInitInfo.position = Eigen::Vector3d(0.75, 0.364, 0.0);
    blueInitInfo.rotate = Eigen::Vector4d(0.0, 0.711, 0.703, M_PI);

    game_ctrl::Score score;
    bool initSeted=false;
    while(super->step(basicTime)!=-1 && ros::ok()){
        const double *ballPos = ballNode->getPosition();
        const double *redPos = redNode->getPosition();
        const double *bluePos = blueNode->getPosition();
        const double *redOriens = redNode->getOrientation();
        const double *blueOriens = blueNode->getOrientation();
        stand_mutex.lock();
        if(mStand.red){
            redNode->getField("translation")->setSFVec3f(redPos);
            redNode->getField("rotation")->setSFRotation(redInitInfo.rotate.data());
        }
        if(mStand.blue){
            blueNode->getField("translation")->setSFVec3f(bluePos);
            blueNode->getField("rotation")->setSFRotation(blueInitInfo.rotate.data());
        }
        mStand.red=false;
        mStand.blue=false;
        stand_mutex.unlock();
        gdata_mutex.lock();
        simulation::GameData tmpdata = mGdata;
        gdata_mutex.unlock();
        if(mGdata.state == simulation::GameData::STATE_INIT && !initSeted){
            redNode->getField("translation")->setSFVec3f(redInitInfo.position.data());
            redNode->getField("rotation")->setSFRotation(redInitInfo.rotate.data());
            blueNode->getField("translation")->setSFVec3f(blueInitInfo.position.data());
            blueNode->getField("rotation")->setSFRotation(blueInitInfo.rotate.data());
            initSeted=true;
        }
        if(mGdata.state != simulation::GameData::STATE_INIT){
            initSeted = false;
        }
        geometry_msgs::Point redPoint, bluePoint;
        redPoint.x = redPos[0];
        redPoint.y = redPos[2];
        bluePoint.x = bluePos[0];
        bluePoint.y = bluePos[2];
        redPosPublisher.publish(redPoint);
        bluePosPublisher.publish(bluePoint);
        bool goal=false;
        if(fabs(ballPos[2])<goalZ) {
            if(ballPos[0]>goalX){
                score.blue++;
                ballNode->getField("translation")->setSFVec3f(ballInit);
                Eigen::Vector3d newRedPos = redInitInfo.position;
                Eigen::Vector3d newBluePos = blueInitInfo.position;
                newRedPos.x() += 0.25;
                newBluePos.x() += 0.25;
                redNode->getField("translation")->setSFVec3f(newRedPos.data());
                redNode->getField("rotation")->setSFRotation(redInitInfo.rotate.data());
                blueNode->getField("translation")->setSFVec3f(newBluePos.data());
                blueNode->getField("rotation")->setSFRotation(blueInitInfo.rotate.data());
                goal=true;
            }
            else if(ballPos[0]<-goalX){
                score.red++;
                ballNode->getField("translation")->setSFVec3f(ballInit);
                Eigen::Vector3d newRedPos = redInitInfo.position;
                Eigen::Vector3d newBluePos = blueInitInfo.position;
                newRedPos.x() -= 0.25;
                newBluePos.x() -= 0.25;
                redNode->getField("translation")->setSFVec3f(newRedPos.data());
                redNode->getField("rotation")->setSFRotation(redInitInfo.rotate.data());
                blueNode->getField("translation")->setSFVec3f(newBluePos.data());
                blueNode->getField("rotation")->setSFRotation(blueInitInfo.rotate.data());
                goal=true;
            }
        }
        if(!goal){
            if(fabs(ballPos[2])>fieldZ || fabs(ballPos[0])>fieldX){
                redNode->getField("translation")->setSFVec3f(redInitInfo.position.data());
                redNode->getField("rotation")->setSFRotation(redInitInfo.rotate.data());
                blueNode->getField("translation")->setSFVec3f(blueInitInfo.position.data());
                blueNode->getField("rotation")->setSFRotation(blueInitInfo.rotate.data());
                ballNode->getField("translation")->setSFVec3f(ballInit);
            }
        }
        else{
            ballNode->getField("translation")->setSFVec3f(ballInit);
            scorePublisher.publish(score);
        }
        ros::spinOnce();
    }
    return 0;
}

void GdataUpdate(const simulation::GameData::ConstPtr &p)
{
    gdata_mutex.lock();
    mGdata = *(p.get());
    gdata_mutex.unlock();
}

void StandUpdate(const simulation::Stand::ConstPtr &s)
{
    stand_mutex.lock();
    mStand = *(s.get());
    stand_mutex.unlock();
}
