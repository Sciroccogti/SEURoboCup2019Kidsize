#include <mutex>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <simulation/ImageInfo.h>
#include <simulation/Task.h>
#include <simulation/GameData.h>
#include <game_ctrl/TeamInfo.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

extern void GdataUpdate(const simulation::GameData::ConstPtr &p);
extern void PositionUpdate(const geometry_msgs::Point::ConstPtr &pos);
extern void ImageUpdate(const sensor_msgs::CompressedImage::ConstPtr &image);
extern void ImagePublish(const cv::Mat &image, string encoding="bgr8");
extern void TaskPublish(const simulation::Task &task);
extern void HeadPublish(double yaw, double pitch);
extern void GetImageInfo();
extern string GetRobot(string tname);
extern cv::Mat GetImage();
extern geometry_msgs::Point GetPosition();
extern simulation::GameData GetGdata();

std::shared_ptr<ros::NodeHandle> node;
int image_width, image_height, image_channels;
ros::Publisher imagePublisher, taskPublisher, headPublisher;
string robot_name="robot";

int main(int argc, char **argv)
{
    string teamname, hostname="localhost";
    if(argc>1) teamname = string(argv[1]);
    if(argc>2) {
        ifstream ifs(argv[2]);
        if(ifs){
            ifs>>hostname;
            ifs.close();
        }
    }
    string uri = "http://"+hostname+":11311";
    setenv("ROS_MASTER_URI", uri.c_str(), 1);
    ros::init(argc, argv, teamname);
    node = std::make_shared<ros::NodeHandle>();
    robot_name = GetRobot(teamname);
    if(robot_name.size()==0){
        ROS_WARN("can not find this team in game_ctrl");
        return 0;
    }

    GetImageInfo();
    ros::Subscriber imageSub = node->subscribe(robot_name+"/image/compressed", 1, ImageUpdate);
    ros::Subscriber posSub = node->subscribe(robot_name+"/position", 1, PositionUpdate);
    ros::Subscriber gdataSub = node->subscribe("/gamedata", 1, GdataUpdate);
    imagePublisher = node->advertise<sensor_msgs::Image>(teamname+"/image", 1);
    taskPublisher = node->advertise<simulation::Task>(robot_name+"/task", 1);
    headPublisher = node->advertise<geometry_msgs::Point>(robot_name+"/head", 1);

    ros::Rate rate(10);
    double yaw=-M_PI/2;
    while(ros::ok()){
        cv::Mat image_tmp = GetImage();
        geometry_msgs::Point mypos = GetPosition();
        simulation::GameData gdata = GetGdata();
        if(gdata.state == simulation::GameData::STATE_PLAY)
        {
            simulation::Task task;
            task.type = simulation::Task::TASK_WALK; // 任务类型 
            task.step = 0.03;  // 行走前进量 米
            task.lateral = 0.0; // 行走横移量 米
            task.turn = 0.0; // 行走转角量 弧度
            double head_yaw=0.0; //头部航向角
            double head_pitch=-M_PI/2; //头部俯仰角
            
            if(!image_tmp.empty()){
                //在这里进行图像处理
                cv::Mat grey;
                cv::cvtColor(image_tmp, grey, CV_BGR2GRAY);
                //发布处理后的图像，便于使用rqt_image_view查看，不建议使用imshow函数
                ImagePublish(grey, "mono8"); 
            }
            HeadPublish(head_yaw, head_pitch);
            TaskPublish(task);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

simulation::GameData mGdata;
mutex gdata_mutex;
mutex image_mutex, pos_mutex;
cv::Mat image_data;
geometry_msgs::Point my_position;
void ImageUpdate(const sensor_msgs::CompressedImage::ConstPtr &image)
{
    vector<uint8_t> buf(image->data.size());
    memcpy(&(buf[0]), &(image->data[0]), image->data.size());
    try
    {
        std::lock_guard<mutex> lk(image_mutex);
        image_data = imdecode(buf, cv::IMREAD_COLOR);
    }
    catch(std::exception &e){
        ROS_INFO("%s", e.what());
    }
}

void PositionUpdate(const geometry_msgs::Point::ConstPtr &pos)
{
    std::lock_guard<mutex> lk(pos_mutex);
    my_position = *(pos.get());
}

void ImagePublish(const cv::Mat &image, string encoding)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();
    imagePublisher.publish(msg);
}

void TaskPublish(const simulation::Task &task)
{
    taskPublisher.publish(task);
}

void HeadPublish(double yaw, double pitch)
{
    geometry_msgs::Point p;
    p.z = yaw;
    p.y = pitch;
    headPublisher.publish(p);
}

void GetImageInfo()
{
    ros::service::waitForService(robot_name+"/imageinfo");
    ros::ServiceClient infoClient = node->serviceClient<simulation::ImageInfo>(robot_name+"/imageinfo");
    simulation::ImageInfo info;
    infoClient.call(info);
    image_width = info.response.width;
    image_height = info.response.height;
    image_channels = info.response.channels;
    infoClient.shutdown();
}


void GdataUpdate(const simulation::GameData::ConstPtr &p)
{
    gdata_mutex.lock();
    mGdata = *(p.get());
    gdata_mutex.unlock();
}

string GetRobot(string tname)
{
    ros::service::waitForService("/teaminfo");
    ros::ServiceClient infoClient = node->serviceClient<game_ctrl::TeamInfo>("/teaminfo");
    game_ctrl::TeamInfo info;
    info.request.teamname=tname;
    infoClient.call(info);
    infoClient.shutdown();
    return info.response.team;
}

cv::Mat GetImage()
{
    std::lock_guard<mutex> lk(image_mutex);
    return image_data.clone();
}

geometry_msgs::Point GetPosition()
{
    std::lock_guard<mutex> lk(pos_mutex);
    return my_position;
}

simulation::GameData GetGdata()
{
    std::lock_guard<mutex> lk(gdata_mutex);
    return mGdata;
}