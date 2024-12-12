#include "ros/ros.h"
#include "greencar_can_test_3/control_cmd.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8MultiArray.h"

#include <iostream>
#include <thread>
#include <mutex>
/*
！！！说明：
        在使用这个包进行控制前，得装好 这两个模块 安装指令如下
            sudo modprobe gs_usb
            sudo apt install can-utils
        装好这两个模块后 使用这个指令查看 连接的can模块
            ifconfig -a          // 如果提示没装 net-tools 那就装上 sudo apt install net-tools
        装好之后，要连接can得设置好波特率 在命令行输入下面这个指令即可
            sudo ip link set can0 up type can bitrate 250000    
        注意 每回绿车重新开机或者 an和主机断开连接，都要重新使用 sudo ip link xxx 上面这个指令来重新连接一下
        sudo ip link xxx 这个指令也可以包含在 ros包中，但要给权限才行
*/

#define safe_distance  40  // 避障距离，如果前后左右四个超声波检测到的距离低于这个值，则导致停车
int cmd_lost=1;
std::mutex mu; 

// 将ascii值转成其对应的数值，仅对 0～10和A～E有效
int asciiToDecimal(int ascii_value) {
    int decimal_value = 0;
    if (ascii_value >= '0' && ascii_value <= '9') {
        decimal_value = ascii_value - '0';
    } else if (ascii_value >= 'A' && ascii_value <= 'E') {
        decimal_value = ascii_value - 'A' + 10;
    }
    return decimal_value;
}

// 四舍五入取整函数
int round_double(double number)
{
  // 若为负数，则先化为正数再进行四舍五入
  if (number > 0)
    return number - int(number) >= 0.5 ? int(number)+1 : int(number);
  else
    return -number - int(-number) >= 0.5 ? (int(number) - 1) : int(number);
}

// 将速度值转为控制的can指令
std::string speedvalue_to_command(std::string motorx,int val){
    std::string temp;   // 定义一个临时字符串变量 用来装指令
    std::stringstream high_bits;
    std::stringstream low_bits; 

    // 先判断传进来的是哪个电机的速度
    if(motorx != "motor1" && motorx != "motor2"){
        ROS_INFO("输入电机名称有误");
        return "false";
    }
    else if (motorx == "motor1")
    {
        temp = "cansend can0 601#2b003000";     // 电机1的部分控制指令
    }
    else {
        temp = "cansend can0 601#2b013000";     // 电机2的部分控制指令
    }

    val = -val;                             // 因为这个绿车默认是发送负的往前走。。。虽然可以改通过设置电机方向来改，但改过后绿车重新开关机又没了。。。所以直接换个正负号吧。。。。
    if(val >= -1000 && val < 0){            // 输入1000～-1之间的值电机反转，即向退
        val = 0x10000 + val;                  
    }   
    // 否则的话，就正常进行咯
    low_bits  << std::setw(2) << std::setfill('0') << std::hex << (0xff & val);     // 取传进来的数字的低2个字节的内容
    high_bits << std::setw(2) << std::setfill('0') << std::hex << (val >> 8);       // 注意是移8位呀。。。不是移16位。。。1个16进制数对应4位的二进制数。。。

    // 构造cansend指令
    temp += low_bits.str();     // 先传输低位，再传输高位
    temp += high_bits.str();
    temp += "0000";

    ROS_INFO("将要发送给绿车的指令是：%s",temp.c_str());
    return temp;
    
}

// 控制车的速度
void control_car(const geometry_msgs::Twist::ConstPtr &msg){
    mu.try_lock();
    cmd_lost = 0;
    mu.unlock();
    // 从消息中获取速度值和角速度值
    double linear_vel = msg->linear.x;      // 单位：米/秒
    double angular_vel = msg->angular.z;

    // 计算左右两个电机的转速（假设轮子半径为r，左轮和右轮的距离d）
    double r = 0.2;     // 轮子半径，单位：米       我粗略测量的轮子半径和左右车轮间距。。。。
    double d = 0.72;    // 左车轮和右车轮的距离，单位：米

    double v_left  = 60*(2*linear_vel - d*angular_vel) / (4*r*M_PI);   //得到的是轮子的转速，多少转每分钟
    double v_right = 60*(2*linear_vel + d*angular_vel) / (4*r*M_PI);

    /*
        根据厂家给的表和问厂家得知，输入1000对应电机3000转/分钟的转速，然后还要经过1：35的减速比才到实际轮子的转速
        所以 算出来的 v_left 和 v_right 还要 ＊35/3 才能变成输入值
    */ 
   v_left  = v_left  * 35 / 3;
   v_right = v_right * 35 / 3;

    // 将转速值四舍五入转化为整值。因为厂家给的指令表那里，只能取整数
    int left_v  = round_double(v_left);
    int right_v = round_double(v_right);

    // 判断传进来的速度值是否在合理范围内   其实-1000~1000 是额定范围，实际还能更高
    if(left_v<-1000 || left_v >1000 || right_v <-1000 || right_v > 1000){
        ROS_INFO("输入电机速度值过大或过小，请重新输入");
    }
    else
    {
        ROS_INFO("motor1转速：%d 转/分钟", left_v);
        ROS_INFO("motor2转速：%d 转/分钟",right_v);
        std::string command1 = speedvalue_to_command("motor1",left_v);
        std::string command2 = speedvalue_to_command("motor2",right_v);

        if(std::system(command1.c_str()) != 0)
        {      // 执行指令 输入给电机1的速度    system()函数 执行成功返回0，不执行成功返回非0值
            ROS_INFO("发送命令执行失败");
        } 
        if(std::system(command2.c_str()) != 0)
        {      // 执行指令 输入给电机2的速度    system()函数 执行成功返回0，不执行成功返回非0值
            ROS_INFO("发送命令执行失败");
        } 
    }

}

void stop_car()
{
    // 从消息中获取速度值和角速度值
    double linear_vel = 0;      // 单位：米/秒
    double angular_vel = 0;

    // 计算左右两个电机的转速（假设轮子半径为r，左轮和右轮的距离d）
    double r = 0.2;     // 轮子半径，单位：米       我粗略测量的轮子半径和左右车轮间距。。。。
    double d = 0.72;    // 左车轮和右车轮的距离，单位：米

    double v_left  = 60*(2*linear_vel - d*angular_vel) / (4*r*M_PI);   //得到的是轮子的转速，多少转每分钟
    double v_right = 60*(2*linear_vel + d*angular_vel) / (4*r*M_PI);

    /*
        根据厂家给的表和问厂家得知，输入1000对应电机3000转/分钟的转速，然后还要经过1：35的减速比才到实际轮子的转速
        所以 算出来的 v_left 和 v_right 还要 ＊35/3 才能变成输入值
    */ 
   v_left  = v_left  * 35 / 3;
   v_right = v_right * 35 / 3;

    // 将转速值四舍五入转化为整值。因为厂家给的指令表那里，只能取整数
    int left_v  = round_double(v_left);
    int right_v = round_double(v_right);

    // 判断传进来的速度值是否在合理范围内   其实-1000~1000 是额定范围，实际还能更高
    if(left_v<-1000 || left_v >1000 || right_v <-1000 || right_v > 1000){
        ROS_INFO("输入电机速度值过大或过小，请重新输入");
    }
    else{
        ROS_INFO("motor1转速：%d 转/分钟", left_v);
        ROS_INFO("motor2转速：%d 转/分钟",right_v);
        std::string command1 = speedvalue_to_command("motor1",left_v);
        std::string command2 = speedvalue_to_command("motor2",right_v);

        if(std::system(command1.c_str()) != 0){      // 执行指令 输入给电机1的速度    system()函数 执行成功返回0，不执行成功返回非0值
            ROS_INFO("发送命令执行失败");
        } 
        if(std::system(command2.c_str()) != 0){      // 执行指令 输入给电机2的速度    system()函数 执行成功返回0，不执行成功返回非0值
            ROS_INFO("发送命令执行失败");
        } 
    }

}



// 控制绿车的避障距离等参数...没啥用。。。绿车开关机变回原样。。。
void control_car_other(const greencar_can_test_3::control_cmd::ConstPtr & msg){
    // 发送一次改变一次电机1方向，不建议使用。。。。因为绿车开关机会又变回原样了。。。
    if(msg->cmd == "change_motor1_direction"){
        if (msg->value == 1){
            std::string cmd = "cansend can0 601#2b293000ffff0000";
            if(std::system(cmd.c_str()) != 0){      // 执行指令 改变电机1的转速方向，每执行一次改变一次。 所以如果说 输入的速度为正值，但电机反转，车子后退，则要改变一下方向
                ROS_INFO("发送命令执行失败");
            } 
            else{
                ROS_INFO("成功发送指令：%s",cmd.c_str()); 
            }
        }
        else {
            ROS_INFO("输入的值有误，只有输入数字 1 才能生效，请重新输入");
        }
    }
    else if(msg->cmd == "change_motor2_direction"){
        if (msg->value == 1){
            std::string cmd = "cansend can0 601#2b2a3000ffff0000";
            if(std::system(cmd.c_str()) != 0){      // 执行指令 改变电机2的转速方向，每执行一次改变一次。 
                ROS_INFO("发送命令执行失败");
            } 
            else{
                ROS_INFO("成功发送指令：%s",cmd.c_str()); 
            }
        }
        else {
            ROS_INFO("输入的值有误，只有输入数字 1 才能生效，请重新输入");
        }
    }
    // 设置前避障距离   经测试无效，。。厂家给的指令没有用。。。 
    else if(msg->cmd == "set_forward_avoid_obstacle_distance"){  
        // 避障距离是0～500cm 
        if(msg->value <0 || msg->value >500){
            ROS_INFO("输入的避障距离超出0～500 单位cm 范围值，请重新输入");
        }
        else{
            std::string temp = "cansend can0 601#2b0B3000";   // 定义一个临时字符串变量 用来装指令
            std::stringstream high_bits;
            std::stringstream low_bits; 
            low_bits  << std::setw(2) << std::setfill('0') << std::hex << (0xff & msg->value);     // 取传进来的数字的低2个字节的内容
            high_bits << std::setw(2) << std::setfill('0') << std::hex << (msg->value >> 8);       // 注意是移8位呀。。。不是移16位。。。1个16进制数对应4位的二进制数。。。

            temp += low_bits.str();
            temp += high_bits.str();
            temp += "0000";
            if(std::system(temp.c_str()) != 0){                 // 执行指令 
                ROS_INFO("发送命令执行失败");
            } 
            else{
                ROS_INFO("成功发送指令：%s",temp.c_str()); 
            }
        }
    }
    // 设置后避障距离   经测试无效，。。厂家给的指令没有用。。。
    else if(msg->cmd == "set_backward_avoid_obstacle_distance"){
        // 避障距离是0～500cm 
        if(msg->value <0 || msg->value >500){
            ROS_INFO("输入的避障距离超出0～500 单位cm 范围值，请重新输入");
        }

        else{
            std::string temp = "cansend can0 601#2b0C3000";   // 定义一个临时字符串变量 用来装指令
            std::stringstream high_bits;
            std::stringstream low_bits; 
            low_bits  << std::setw(2) << std::setfill('0') << std::hex << (0xff & msg->value);     // 取传进来的数字的低2个字节的内容
            high_bits << std::setw(2) << std::setfill('0') << std::hex << (msg->value >> 8);       // 注意是移8位呀。。。不是移16位。。。1个16进制数对应4位的二进制数。。。

            temp += low_bits.str();
            temp += high_bits.str();
            temp += "0000";
            if(std::system(temp.c_str()) != 0){                 // 执行指令 
                ROS_INFO("发送命令执行失败");
            } 
            else{
                ROS_INFO("成功发送指令：%s",temp.c_str()); 
            }
        }
    }
    // 设置是否避障使能 经测试无效，。。厂家给的指令没有用。。。
    else if(msg->cmd == "set_avoid_enable"){
        /*
            0为不使能避障功能         1为使能前避障
            2为使能后避障            3为使能前后避障
        */
        // 避障距离是0～500cm 
        if(msg->value !=0 && msg->value !=1 && msg->value != 2 && msg->value != 3){
            ROS_INFO("输入的避障模式有误 避障模式只有4种，分别是 0，1，2，3 请重新输入");
        }
        else{
            std::string temp = "cansend can0 601#2b0D3000";   // 定义一个临时字符串变量 用来装指令
            std::stringstream high_bits;
            std::stringstream low_bits; 
            low_bits  << std::setw(2) << std::setfill('0') << std::hex << (0xff & msg->value);     // 取传进来的数字的低2个字节的内容
            high_bits << std::setw(2) << std::setfill('0') << std::hex << (msg->value >> 8);       // 注意是移8位呀。。。不是移16位。。。1个16进制数对应4位的二进制数。。。

            temp += low_bits.str();
            temp += high_bits.str();
            temp += "0000";
            if(std::system(temp.c_str()) != 0){                 // 执行指令 
                ROS_INFO("发送命令执行失败");
            } 
            else{
                ROS_INFO("成功发送指令：%s",temp.c_str()); 
            }
        }
    }
    else {
        ROS_INFO("输入的控制命令有误，请检查输入的控制命令");
    }
}

// 用来控制避障距离的回调函数
void control_avoid_obstacle_distance(const std_msgs::UInt8MultiArray::ConstPtr & msg){
    // 构造cansend命令并执行
    /*  暂时不需要它们了
        std::stringstream ss;
    ss << std::hex << msg->data[0] << std::hex << msg->data[1]
       << std::hex << msg->data[2] << std::hex << msg->data[3]
       << std::hex << msg->data[4] << std::hex << msg->data[5]
       << std::hex << msg->data[6] << std::hex << msg->data[7]
       << std::hex << msg->data[8] << std::hex << msg->data[9]
       << std::hex << msg->data[10] << std::hex << msg->data[11]
       << std::hex << msg->data[12] << std::hex << msg->data[13]
       << std::hex << msg->data[14] << std::hex << msg->data[15]
       << std::hex << msg->data[16] << std::hex << msg->data[17]
       << std::hex << msg->data[18] << std::hex << msg->data[19]
       << std::hex << msg->data[20] << std::hex << msg->data[21];
    ROS_INFO("接收到的数据是：%s",ss.str().c_str());
    */

    // 靠！msg->data[]里面存的是那些数字或字符的ascii值。。。。。。。。
    // 左前方超声波距离
    int left_forward_distance = asciiToDecimal(msg->data[6])*16+asciiToDecimal(msg->data[7])+asciiToDecimal(msg->data[9]*16*16);
    // 右前方超声波距离
    int right_forward_distance = asciiToDecimal(msg->data[10])*16+asciiToDecimal(msg->data[11])+asciiToDecimal(msg->data[13]*16*16);
    // 左后方超声波距离
    int left_backward_distance = asciiToDecimal(msg->data[14])*16+asciiToDecimal(msg->data[15])+asciiToDecimal(msg->data[17]*16*16);
    // 右后方超声波距离
    int right_backward_distance = asciiToDecimal(msg->data[18])*16+asciiToDecimal(msg->data[19])+asciiToDecimal(msg->data[21]*16*16);

    // 如果小于避障距离avoid_distance 就触发停车
#if 0
    if(left_forward_distance <safe_distance || right_forward_distance <safe_distance || 
        left_backward_distance <safe_distance || right_backward_distance <safe_distance){
        if(std::system("cansend can0 601#2b00300000000000") != 0){      // 执行指令 输入给电机1的速度    system()函数 执行成功返回0，不执行成功返回非0值
            ROS_INFO("发送命令执行失败");
        } 
        if(std::system("cansend can0 601#2b01300000000000") != 0){      // 执行指令 输入给电机2的速度    system()函数 执行成功返回0，不执行成功返回非0值
            ROS_INFO("发送命令执行失败");
        } 
    }
    if(left_forward_distance <safe_distance){
        ROS_INFO("左前方与物体相距低于安全距离！！！要撞上啦！！！ ");
    }
    if(right_forward_distance <safe_distance){
        ROS_INFO("右前方与物体相距低于安全距离！！！要撞上啦！！！ ");
    }
    if(left_backward_distance <safe_distance){
        ROS_INFO("左后方与物体相距低于安全距离！！！要撞上啦！！！ ");
    }
    if(right_backward_distance <safe_distance){
        ROS_INFO("右后方与物体相距低于安全距离！！！要撞上啦！！！ ");
    }
#endif
if(left_forward_distance < safe_distance || right_forward_distance < safe_distance || 
    left_backward_distance < safe_distance || right_backward_distance < safe_distance) {
    char cmd[100];
    std::sprintf(cmd, "cansend can0 601#2b00300000000000 && cansend can0 601#2b01300000000000");
    if(std::system(cmd) != 0) {
        ROS_INFO("发送命令执行失败");
    }
    if(left_forward_distance < safe_distance) {
        ROS_INFO("左前方与物体相距低于安全距离！！！要撞上啦！！！ ");
    }
    if(right_forward_distance < safe_distance) {
        ROS_INFO("右前方与物体相距低于安全距离！！！要撞上啦！！！ ");
    }
    if(left_backward_distance < safe_distance) {
        ROS_INFO("左后方与物体相距低于安全距离！！！要撞上啦！！！ ");
    }
    if(right_backward_distance < safe_distance) {
        ROS_INFO("右后方与物体相距低于安全距离！！！要撞上啦！！！ ");
    }
}
#if 0
    // 左前方超声波距离
    std::stringstream left_forward_distance;
    left_forward_distance   << std::hex << msg->data[8] << std::hex<<msg->data[9]
                            << std::hex << msg->data[6] << std::hex<<msg->data[7];

    // 右前方超声波距离
    std::stringstream right_forward_distance; 
    right_forward_distance  << std::hex << msg->data[12] << std::hex<<msg->data[13]
                            << std::hex << msg->data[10] << std::hex<<msg->data[11];

    // 左后方超声波距离
    std::stringstream left_backward_distance;
    left_backward_distance  << std::hex << msg->data[16] << std::hex<<msg->data[17]
                            << std::hex << msg->data[14] << std::hex<<msg->data[15];
    // 右后方超声波距离
    std::stringstream right_backward_distance;
    right_backward_distance << std::hex << msg->data[20] << std::hex<<msg->data[21]
                            << std::hex << msg->data[18] << std::hex<<msg->data[19];
   ROS_INFO("\n左前方超声波距离：%s cm\n右前方超声波距离：%s cm\n左后方超声波距离：%s cm\n右后方超声波距离：%s cm",left_forward_distance.str().c_str(),
   right_forward_distance.str().c_str(),left_backward_distance.str().c_str(),right_backward_distance.str().c_str());

    int distance = msg->data[6]*16 + msg->data[7];
    ROS_INFO("数字是：%d %d",msg->data[6],msg->data[7]);
    if (distance < 25){
        ROS_INFO("快撞到东西啦！！！");
    }
#endif

}

int main(int argc, char *argv[])
{   

    // 使输出到终端的文字不乱码，正常显示
    setlocale(LC_ALL,"");
    
    //初始化节点
    ros::init(argc, argv, "greencar_control");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cmd_vel", 100, control_car);    // 订阅用来控制车的速度的

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // 输出信息，表示订阅服务端已经起动了
    ROS_INFO("服务端开始工作");


    // ros::Subscriber other_control_sub = nh.subscribe("greencar_control_other",100,control_car_other);   // 订阅用来控制车的其它东西的，但没啥用。。。
    // 创建发布者 用来接can传输来的信息，并将其解析出来，用来设置避障
    // ros::Subscriber sub_can_msg = nh.subscribe("can_rx_msg",100,control_avoid_obstacle_distance);

    // 设置循环频率
    ros::Rate loop_rate(1); // 每秒循环10次
    int count_lost = 0;
    
    while (ros::ok())
    {
        mu.try_lock();
        cmd_lost=1;
        mu.unlock();
        // 创建发布者 用来接收控制信息
        if (cmd_lost==1)
        {
            count_lost++;
            if(count_lost>3){
                stop_car();
            }
        }
        else{
            count_lost=0;
        }

        // ros::spinOnce();
        loop_rate.sleep();
    }

    







    
    return 0;
}
