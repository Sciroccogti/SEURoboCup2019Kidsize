#include "unirobot.hpp"
#include <cstdlib>
#include <mutex>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <webots/PositionSensor.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace webots;

const float p1[2] = {-0.15, 0.09};
const float p2[2] = {0.0, 0.06};
const float p3[2] = {0.20, 0.09};
const float times = 20;
const double engineFrequency = 50.0;

const char *Neck[2] = {"Neck", "Neck2"};
const char *LeftArmJoint[2] = {"LeftShoulder", "LeftElbow"};
const char *RightArmJoint[2] = {"RightShoulder", "RightElbow"};
const char *LeftLegJoint[6] = {"LeftLegX", "LeftLegY", "LeftLegZ", "LeftKnee", "LeftAnkleX", "LeftAnkleY"};
const char *RightLegJoint[6] = {"RightLegX", "RightLegY", "RightLegZ", "RightKnee", "RightAnkleX", "RightAnkleY"};
const float mx28 = 0.0441;
const float mx64 = 0.0724;
const float mx106 = 0.0865;
const float computer = 2 * 0.0865;
const float torso = 6 * 0.0441 + 4 * 0.0724 + 5 * 0.0865;
const double mfoot = 4.7 * 0.0724;
const float bone_hip = 0.0549;
const float bone_upperleg = 0.15;
const float bone_lowerleg = 0.15;
const float bone_ankle = 0.03;
const float roll = M_PI / 8;
float rightlegx = 0.0;
float rightlegy = 0.0;
float rightlegz = 0.0;
float righthipx = 0.0;
float righthipy = 0.0;
float righthipz = 0.0;
float rightkneey = 0.0;
float rightanklex = 0.0;
float rightankley = 0.0;

mutex head_mutex, task_mutex, gdata_mutex, stand_mutex;

Rhoban::IKWalkParameters params
{
  params.distHipToKnee = 0.15,
  params.distKneeToAnkle = 0.15,
  params.distAnkleToGround = 0.03,
  params.distFeetLateral = 0.1098,
  params.freq = 2.0,
  params.enabledGain = 1.0,
  params.supportPhaseRatio = 0.0,
  params.footYOffset = 0.02,
  params.stepGain = 0.0,
  params.riseGain = 0.035,
  params.turnGain = 0.0,
  params.lateralGain = 0.0,
  params.trunkZOffset = 0.02,
  params.swingGain = 0.02,
  params.swingRollGain = 0.0,
  params.swingPhase = 0.25,
  params.stepUpVel = 3.0,
  params.stepDownVel = 3.0,
  params.riseUpVel = 3.0,
  params.riseDownVel = 3.0,
  params.swingPause = 0.0,
  params.swingVel = 4.0,
  params.trunkXOffset = 0.02,
  params.trunkYOffset = 0.0,
  params.trunkPitch = 0.15,
  params.trunkRoll = 0.0,
  params.extraLeftX = 0.0,
  params.extraLeftY = 0.0,
  params.extraLeftZ = 0.0,
  params.extraRightX = 0.0,
  params.extraRightY = 0.0,
  params.extraRightZ = 0.0,
  params.extraLeftYaw = 0.0,
  params.extraLeftPitch = 0.0,
  params.extraLeftRoll = 0.0,
  params.extraRightYaw = 0.0,
  params.extraRightPitch = 0.0,
  params.extraRightRoll = 0.0
};
  
UniRobot::UniRobot(string name): Robot()
{
    mName = name;
    totalTime = 0;
    mTimeStep = getBasicTimeStep();
    mCamera = getCamera("Camera");
    mCamera->enable(5 * mTimeStep);

    mNode = new ros::NodeHandle;
    mImagePublisher = mNode->advertise<sensor_msgs::CompressedImage>(mName+"/image/compressed", 1);
    mHeadSubscriber = mNode->subscribe(mName+"/head", 1, &UniRobot::HeadChanged, this);
    mTaskSubscriber = mNode->subscribe(mName+"/task", 1, &UniRobot::TaskUpdate, this);
    mGdataSubscriber = mNode->subscribe("/gamedata", 1, &UniRobot::GdataUpdate, this);
    mStandSubscriber = mNode->subscribe("/stand", 1, &UniRobot::StandUpdate, this);
    mImageInfoServer = mNode->advertiseService(name+"/imageinfo", &UniRobot::ImageInfoService, this);
}

void UniRobot::HeadChanged(const geometry_msgs::Point::ConstPtr &p)
{
    head_mutex.lock();
    mUOuts.head_pitch = p->y;
    mUOuts.head_yaw = p->z;
    if(mUOuts.head_pitch>M_PI/2) mUOuts.head_pitch=M_PI/2;
    if(mUOuts.head_pitch<-M_PI/2) mUOuts.head_pitch=-M_PI/2;
    if(mUOuts.head_yaw>M_PI/2) mUOuts.head_yaw=M_PI/2;
    if(mUOuts.head_yaw<-M_PI/2) mUOuts.head_yaw=-M_PI/2;
    head_mutex.unlock(); 
}

void UniRobot::TaskUpdate(const simulation::Task::ConstPtr &p)
{
    task_mutex.lock();
    if(!mTasks.empty()) mTasks.clear();
    mTasks.push_back(*(p.get()));
    task_mutex.unlock();
}

void UniRobot::GdataUpdate(const simulation::GameData::ConstPtr &p)
{
    gdata_mutex.lock();
    mGdata = *(p.get());
    gdata_mutex.unlock();
}

void UniRobot::StandUpdate(const simulation::Stand::ConstPtr &p)
{
    stand_mutex.lock();
    mStand = *(p.get());
    stand_mutex.unlock();
}

bool UniRobot::ImageInfoService(simulation::ImageInfo::Request &req, 
        simulation::ImageInfo::Response &res)
{
    res.channels = 4;
    res.width = mCamera->getWidth();
    res.height = mCamera->getHeight();
    return true;
}

UniRobot::~UniRobot(){
    delete mNode;
    delete mCamera;
}

int UniRobot::myStep() {
  SetPositions();
  ros::spinOnce();
  totalTime += mTimeStep;
  if(totalTime%(5*mTimeStep)==0)
      PublishImage();
  return step(mTimeStep);
}

void UniRobot::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void UniRobot::run(){
    double phase = 0.0;
    double time = 0.0;
    bool isWalking = false;
    //ctionReady();
    //ActionRightKick(0.0, -0.05, 0.0, 0 * M_PI / 180);
    int cnt=0;
    while(ros::ok())
    {
        bool fall=false;
        stand_mutex.lock();
        if((mName == "redrobot" && mStand.red)
            ||(mName == "bluerobot" && mStand.blue))
        {
            fall=true;
            Rhoban::IKWalkOutputs outs={0.0};
            mLOuts = outs;
            myStep();
            cnt++;
            if(cnt>10){
                mStand.red=false;
                mStand.blue=false;
            }
        }
        else{
            cnt=0;
        }
        stand_mutex.unlock();
        if(fall) continue;
        gdata_mutex.lock();
        simulation::GameData gdata=mGdata;
        gdata_mutex.unlock();
        if(gdata.state != simulation::GameData::STATE_PLAY){
            if(isWalking) stopWalk(isWalking, phase, time);
            else myStep();
            Rhoban::IKWalkOutputs outs={0.0};
            mLOuts = outs;
            continue;
        }
        simulation::Task task;
        task.type = simulation::Task::TASK_NONE;
        task_mutex.lock();
        if(!mTasks.empty()) task=mTasks.front();
        mTasks.clear();
        task_mutex.unlock();
        if(task.type == simulation::Task::TASK_WALK){
            if(!isWalking){
                params.enabledGain = 1.0;
                params.stepGain = 0.0;
                params.lateralGain = 0.0;
                params.turnGain = 0.0;
                isWalking = true;
                runWalk(params, 2.0, phase, time);
            }
            else{
                params.enabledGain = 1.0;
                params.stepGain = task.step;
                params.lateralGain = task.lateral;
                params.turnGain = task.turn;
                runWalk(params, 2.0, phase, time);
            }
        }
        else if(task.type == simulation::Task::TASK_KICK){
            if(isWalking){
                stopWalk(isWalking, phase, time);
            }
            ActionReady();
            wait(500);
            ActionRightKick(0, -0.05, 0, 0);
            wait(500);
            ActionReady();
            wait(500);
        }
        else{
            if(isWalking){
                stopWalk(isWalking, phase, time);
            }
            else{
                myStep();
            }
        }
    }
}

void UniRobot::stopWalk(bool &isWalking, double &phase, double &time)
{
    params.enabledGain = 0.0;
    params.stepGain = 0.0;
    params.lateralGain = 0.0;
    params.turnGain = 0.0;
    isWalking = false;
    runWalk(params, 2.0, phase, time);
}

void UniRobot::PublishImage()
{
    const unsigned char *colorImage = mCamera->getImage();
    if(colorImage!=NULL)
    {
        cv::Mat raw(mCamera->getHeight(), mCamera->getWidth(), CV_8UC4);
        memcpy(raw.data, colorImage, 4 * mCamera->getWidth() * mCamera->getHeight());
        std::vector<uchar> buf;
        cv::imencode(".jpg", raw, buf);
        sensor_msgs::CompressedImage image;
        image.header.stamp = ros::Time::now();
        image.header.frame_id = mName;
        image.format = "jpeg";
        image.data.resize(buf.size());
        memcpy(&image.data[0], &(buf[0]), buf.size());
        mImagePublisher.publish(image);
    }
}

void UniRobot::GetPositions(Rhoban::IKWalkOutputs &lOuts, UpperOutputs &uOuts)
{
    lOuts.left_hip_roll = getMotor(LeftLegJoint[0])->getPositionSensor()->getValue();
    lOuts.left_ankle_pitch = getMotor(LeftLegJoint[1])->getPositionSensor()->getValue();
    lOuts.left_hip_yaw = getMotor(LeftLegJoint[2])->getPositionSensor()->getValue();
    lOuts.left_knee = getMotor(LeftLegJoint[3])->getPositionSensor()->getValue();
    lOuts.left_ankle_roll = getMotor(LeftLegJoint[4])->getPositionSensor()->getValue();
    lOuts.left_ankle_pitch = getMotor(LeftLegJoint[5])->getPositionSensor()->getValue();    
    lOuts.right_hip_roll = getMotor(RightLegJoint[0])->getPositionSensor()->getValue();
    lOuts.right_ankle_pitch = getMotor(RightLegJoint[1])->getPositionSensor()->getValue();
    lOuts.right_hip_yaw = getMotor(RightLegJoint[2])->getPositionSensor()->getValue();
    lOuts.right_knee = getMotor(RightLegJoint[3])->getPositionSensor()->getValue();
    lOuts.right_ankle_roll = getMotor(RightLegJoint[4])->getPositionSensor()->getValue();
    lOuts.right_ankle_pitch = getMotor(RightLegJoint[5])->getPositionSensor()->getValue();  

    uOuts.left_shoulder = getMotor(LeftArmJoint[0])->getPositionSensor()->getValue(); 
    uOuts.left_elbow = getMotor(LeftArmJoint[1])->getPositionSensor()->getValue(); 
    uOuts.right_shoulder = getMotor(RightArmJoint[0])->getPositionSensor()->getValue(); 
    uOuts.right_elbow = getMotor(RightArmJoint[1])->getPositionSensor()->getValue(); 

    uOuts.head_yaw = getMotor(Neck[0])->getPositionSensor()->getValue(); 
    uOuts.head_pitch = getMotor(Neck[1])->getPositionSensor()->getValue(); 
}

void UniRobot::SetPositions()
{
    Motor *motor;
    motor = getMotor(LeftLegJoint[0]);
    motor->setPosition(mLOuts.left_hip_roll);
    motor = getMotor(LeftLegJoint[1]);
    motor->setPosition(mLOuts.left_hip_pitch);
    motor = getMotor(LeftLegJoint[2]);
    motor->setPosition(mLOuts.left_hip_yaw);
    motor = getMotor(LeftLegJoint[3]);
    motor->setPosition(mLOuts.left_knee);
    motor = getMotor(LeftLegJoint[4]);
    motor->setPosition(mLOuts.left_ankle_roll);
    motor = getMotor(LeftLegJoint[5]);
    motor->setPosition(mLOuts.left_ankle_pitch);

    motor = getMotor(RightLegJoint[0]);
    motor->setPosition(mLOuts.right_hip_roll);
    motor = getMotor(RightLegJoint[1]);
    motor->setPosition(mLOuts.right_hip_pitch);
    motor = getMotor(RightLegJoint[2]);
    motor->setPosition(mLOuts.right_hip_yaw);
    motor = getMotor(RightLegJoint[3]);
    motor->setPosition(mLOuts.right_knee);
    motor = getMotor(RightLegJoint[4]);
    motor->setPosition(mLOuts.right_ankle_roll);
    motor = getMotor(RightLegJoint[5]);
    motor->setPosition(mLOuts.right_ankle_pitch);

    motor = getMotor(LeftArmJoint[0]);
    motor->setPosition(mUOuts.left_shoulder);
    motor = getMotor(LeftArmJoint[1]);
    motor->setPosition(mUOuts.left_elbow);
    motor = getMotor(RightArmJoint[0]);
    motor->setPosition(mUOuts.right_shoulder);
    motor = getMotor(RightArmJoint[1]);
    motor->setPosition(mUOuts.right_elbow);

    motor = getMotor(Neck[1]);
    motor->setPosition(mUOuts.head_pitch);
    motor = getMotor(Neck[0]);
    motor->setPosition(mUOuts.head_yaw);
}


void UniRobot::runWalk(const Rhoban::IKWalkParameters& params, 
    double timeLength, double& phase, double& time)
{
    //Walk engine frequency
    for (double t=0.0;t<=timeLength;t+=1.0/engineFrequency) {
        time += 1.0/engineFrequency;
        bool success = Rhoban::IKWalk::walk(
            params, //Walk parameters
            1.0/engineFrequency, //Time step
            phase, //Current walk phase -will be updated)
            mLOuts); //Result target position (updated)
        if (!success) {
            //The requested position for left or right foot is not feasible
            //(phase is not updated)
            std::cout << time << " Inverse Kinematics error. Position not reachable." << std::endl;
        } else {
            myStep();
        }
    }
}


float UniRobot::ComputeVtorso(float theta)//Compute vector of torso
{
    float trans_y = (bone_lowerleg + bone_upperleg) * cos(roll) * sin(theta);
    //float trans_z = (bone_lowerleg + bone_upperleg) * cos(roll) * cos(theta);
    float vtorso = mx106 * bone_lowerleg * cos(roll) * sin(theta) + torso * (trans_y - bone_hip);
    return vtorso;
}

float UniRobot::ComputeVrightleg(float theta, float x, float y, float z)
{
    float trans = (bone_lowerleg + bone_upperleg) * cos(roll) * sin(theta);
    float legy = - y - bone_hip + trans;
    float legx = x;
    float legz = (bone_lowerleg + bone_upperleg) * cos(roll) * cos(theta) - z;
    
    float A = bone_upperleg;
    float B = bone_lowerleg;
    float C = pow((pow(legx, 2) + pow(legy, 2) + pow(legz, 2)), 0.5);
    
    if(A + B < C)
    {
       std::cout << "error: A + B < C" << std::endl;
       return 100000000;
    }
      
    float rky = (pow(A, 2) + pow(B, 2) - pow(C, 2)) / (2 * A * B);
    float rh = (pow(A, 2) + pow(C, 2) - pow(B, 2)) / (2 * A * C);
  
    rightkneey = M_PI - acos(rky);
    rightlegx = atan2(legy, legz);
    rightlegy = atan2(legx, legz);
    rightlegz = atan2(legx, legy);
    righthipx = -rightlegx;
    righthipy = -(rightlegy + acos(rh));
    rightanklex = rightlegx;
    rightankley = -rightkneey - righthipy;
    
    float vrightleg = mx106 * (-bone_upperleg * sin(fabs(righthipy)) * sin(fabs(righthipx)) + trans -  2 * bone_hip ) + mfoot * (trans -  2 * bone_hip  + y);
    return vrightleg;
}

void UniRobot::ActionRightFootPos(float x, float y, float z)
{
    Motor *motor;
    float theta = 0.0;
    float delta = 0.0;
    float mindelta = 100.0;
    float mintheta = 0.0;
    for(theta = 0.0; theta < M_PI / 4; theta += M_PI/180)
    {
      delta = fabs(ComputeVtorso(theta) + ComputeVrightleg(theta, x, y, z));
      if(delta < mindelta)
      {
        mindelta = delta;
        mintheta = theta;
      }
    }
    theta = mintheta;
    delta = fabs(ComputeVtorso(mintheta) + ComputeVrightleg(mintheta, x, y, z));

    mLOuts.left_hip_roll = -theta;
    mLOuts.left_hip_pitch = -roll;
    mLOuts.left_hip_yaw = 0.0;
    mLOuts.left_knee = 2*roll;
    mLOuts.left_ankle_roll = theta;
    mLOuts.left_ankle_pitch = -roll;

    mLOuts.right_hip_roll = righthipx;
    mLOuts.right_hip_pitch = righthipy;
    mLOuts.right_hip_yaw = righthipz;
    mLOuts.right_knee = rightkneey;
    mLOuts.right_ankle_roll = rightanklex;
    mLOuts.right_ankle_pitch = rightankley;

    myStep();
}

void UniRobot::ActionRightKick(float x, float y, float z, float alpha)
{
    int i = 0;
    for(i = 0; i < 10; i++)
    {
      ActionRightFootPos(0, -0.04 * i / 10, 0.01 * i / 10);
    }
    wait(100);
    for(i = 0; i < times; i++)
    {
      ActionRightFootPos((p1[0] * cos(alpha) + x) * i / times, 
      (p1[0] * sin(alpha) + y)* i / times + y *  (1 - i / times), 
      p1[1] * i / times + 0.08 * (1 - i / times));
    }
    wait(100);
    for(i = 0; i < times; i++)
    {
      float xx = pow((1 - i/times), 2) * p1[0] 
              + 2 * (1 - i/times) * i/times * p2[0]
              + pow(i/times, 2) * p3[0];
      float yy = y;
      float zz = pow((1 - i/times), 2) * p1[1] 
              + 2 * (1 - i/times) * i/times * p2[1]
              + pow(i/times, 2) * p3[1];
      float xxx = x + xx * cos(alpha); 
      float yyy = yy + xx * sin(alpha);
      float zzz = zz;
      ActionRightFootPos(xxx, yyy, zzz);
    }
    wait(100);
    for(i = 0; i < times; i++)
    {
      ActionRightFootPos((p3[0] * cos(alpha) + x) * (1 - i / times), 
      (p3[0] * sin(alpha) + y) * (1 - i / times)  + y *  i / times, 
      p3[1] * (1 - i / times) + 0.08 *  i / times);
    }
}

void UniRobot::ActionReady()
{
    wait(100);
    for(int i = 0; i < 10; i++)
    {
      ActionRightFootPos(0, -0.04 * i / 10, 0.01 * i / 10);
    }
}