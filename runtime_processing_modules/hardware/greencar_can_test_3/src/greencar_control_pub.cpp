#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "greencar_can_test_3/control_cmd.h"
#include "std_msgs/UInt8MultiArray.h"

int main (int argc,char * argv[]){
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"greencar_control_pub");
    ros::NodeHandle nh;

    //ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("greencar_control",100);   // 这里用来控制车的速度的
    ros::Publisher other_control_pub = nh.advertise<greencar_can_test_3::control_cmd>("greencar_control_other",100);  // 这里是用来发布控制车的避障距离等七七八八的东西的指令的

    ros::Rate rate(5);      // 每隔0.2秒发送一次

    geometry_msgs::Twist velocity;
    velocity.linear.x = 0.0;
    velocity.angular.z = 0.0;

    // 用来发布前后左右四个超声波模块检测到的距离，用来避障停车
    ros::Publisher obstacle_avoid_distance = nh.advertise<greencar_can_test_3::control_cmd>("obstacle_avoid_distance",100);

    /*----------下面这一大段是用来读取can信息，用来设置避障距离的----------*/ 
    FILE* can_dump_pipe = popen("candump can0", "r");   //popen函数可用于启动一个新进程并打开一个管道用于输入或输出。它的参数包括要执行的命令和执行模式。
    ros::Publisher can_pub = nh.advertise<std_msgs::UInt8MultiArray>("can_rx_msg",100);
    if(can_dump_pipe == nullptr)
    {
        ROS_ERROR("未能执行 candump 指令");
        return 1;
    }

    // 循环读取candump输出并发布数据
    char can_frame_line[100];   //声明一个字符数组can_frame_line，用于存储从管道中读取的每一行CAN数据帧的文本信息。    

    while(fgets(can_frame_line, sizeof(can_frame_line), can_dump_pipe) != nullptr)
    {   /*  
            while循环使用fgets()函数从管道中读取一行CAN数据帧文本，如果读取成功则执行循环内部代码块。
            fgets()函数的第二个参数sizeof(can_frame_line)表示最多读取can_frame_line数组长度的数据。
        */

        ROS_INFO("Received CAN frame: %s", can_frame_line); // 输出调试信息

        // 解析CAN帧数据并构造ROS消息
        std_msgs::UInt8MultiArray can_rx_msg;       //声明一个名为can_rx_msg的std_msgs::UInt8MultiArray类型的变量，该变量用于存储解析出的CAN数据帧信息。
        can_rx_msg.data.resize(22);                  // CAN帧数据为8个字节 但还得加上其它内容  注意数组长度要确定，不然会出错！！！
        std::stringstream ss(can_frame_line);       //std::stringstream ss(can_frame_line); 使用字符串流对象ss对can_frame_line进行解析。
        std::string field;

        /* 使用getline函数从ss中读取一个字段，以空格符作为分隔符。由于前三个字段不是CAN数据帧的内容，因此对它们进行跳过。*/
        for(int i=0; i<3; i++) std::getline(ss, field, ' ');        
        
        for(int i=0; i<22; i++) ss >> std::hex >> can_rx_msg.data[i]; // 解析CAN帧数据      注意这里的can_rx_msg的数据长度在前面被定死了！！
        
        // 用来挑选要的值
        std::stringstream sss;
        sss << std::hex << can_rx_msg.data[0] << std::hex << can_rx_msg.data[1]
            << std::hex << can_rx_msg.data[2];
 
        // 只发布超声波传感器测的距离值
        if(sss.str() == "188"){
            can_pub.publish(can_rx_msg); // 发布ROS消息
        }

        // 将这个也放到这个循环里算了。。。
        // velocity_pub.publish(velocity);     // 如果改变其参数的话， 就默认发布0，0 使车停止
        // rate.sleep();
    }

    pclose(can_dump_pipe); // 关闭管道

    return 0;
}
