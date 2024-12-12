#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <ros/package.h>


bool getStart = false; 
ros::Publisher phone_pub;


void stopCallBack(const std_msgs::String::ConstPtr &msg)
{   
    ROS_INFO("STOP Message arrived");
    
    std_msgs::String msg1;
    msg1.data = "MainNodeRecived Message";
    phone_pub.publish(msg1);

    ROS_INFO("CallBackMessage has beed sent");

    ros::NodeHandle private_nh("~");

    //获取sh文件路径
    std::string packagePath = ros::package::getPath("phone_controller");
    ROS_INFO("run:%s",packagePath.c_str());

    const char* sh = "sh ";
    //shell脚本路径
    std::string SHPath = packagePath+"/sh/stop.sh";
    
    std::string shCmd  =sh+SHPath;
    
    
    if(system(shCmd.c_str())==0){
        ROS_INFO("shell run");
    }else{
        ROS_ERROR("shell not run");
    }


}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"order_stop");
    ROS_INFO("Node:order_stop,started");
    ros::NodeHandle h;
    ros::Subscriber startListener = h.subscribe<std_msgs::String>("/phoneControler/stop",10,stopCallBack);
    phone_pub = h.advertise<std_msgs::String>("/phoneStopRecive",10);
    ros::spin();
    return 0;
}
