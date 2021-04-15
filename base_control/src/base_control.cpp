#include <iostream>                   // C++标准库头文件
#include <boost/asio.hpp>             // boost库头文件
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace boost::asio;

struct speed_info
{
    double x;
    double y;
    double th;
};

struct speed_info_int
{
    int x;
    int y;
    int th;
};

speed_info *position,*True_Speed; //小车的位置、真实编码器速度
speed_info *com_vel_speed;        //发送给小车的速度

const uint8_t buffer_length=14;
char speed_recive_buf[buffer_length],speed_sent_buf[buffer_length];

/******************************  配置串口参数 ***********************************/
bool UART_Init(boost::asio::serial_port &sp1)
{ 
    sp1.set_option(serial_port::baud_rate(115200));                              // 设置波特率
    sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));  // 设置控制方式
    sp1.set_option(serial_port::parity(serial_port::parity::none));              // 设置奇偶校验
    sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));         // 设置停止位
    sp1.set_option(serial_port::character_size(8));                              // 设置字母位数为8位

    return true;
}

//将数组数据转换成float数据
void buff_to_float(char *buff,speed_info* const sp_info)
{
    int32_t a,b,c;
    a = (buff[1] <<24 | buff[2] <<16 | buff[3] <<8 | buff[4] <<0);
    b = (buff[5] <<24 | buff[6] <<16 | buff[7] <<8 | buff[8] <<0);
    c = (buff[9] <<24 | buff[10] <<16 | buff[11] <<8 | buff[12] <<0);
    sp_info->x  = *(float*)(&a);
    sp_info->y  = *(float*)(&b);
    sp_info->th = *(float*)(&c);
}

//将float型数据转换成8字节数组
void float_to_buff(const speed_info* sp_info,char *buff)
{
    speed_info_int *sent_data_int;
    
    buff[0] = 0x01;
    buff[13]= 0x02;

    sent_data_int->x  = *(int* )(&sp_info->x);
    sent_data_int->y  = *(int* )(&sp_info->y);    
    sent_data_int->th = *(int* )(&sp_info->th);

    buff[1] = (sent_data_int->x)  >> 24;
    buff[2] = (sent_data_int->x & 0x00ff0000)>>16;
    buff[3] = (sent_data_int->x & 0x0000ff00)>>8;
    buff[4] = (sent_data_int->x & 0x000000ff);

    buff[5] = (sent_data_int->y)  >> 24;
    buff[6] = (sent_data_int->y & 0x00ff0000)>>16;
    buff[7] = (sent_data_int->y & 0x0000ff00)>>8;
    buff[8] = (sent_data_int->y & 0x000000ff);

    buff[9]  = (sent_data_int->th)  >> 24;
    buff[10] = (sent_data_int->th & 0x00ff0000)>>16;
    buff[11] = (sent_data_int->th & 0x0000ff00)>>8;
    buff[12] = (sent_data_int->th & 0x000000ff);
}

//校验buff数组的帧头帧尾数据
bool CRC_verify(char* buff)
{
    if(buff[0] == 0x00 && buff[13] == 0x00)
        return true;
    return false;
}
/******************************  以上为串口相关 ***********************************/


// 在回调函数中将接收到的cmd_vel速度消息转换为自定义的结构体（或者union）类型的数据
void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    //填充发送速度信息的结构体数据
    com_vel_speed->x  = twist_aux.linear.x;
    com_vel_speed->y  = twist_aux.linear.y;
    com_vel_speed->th = twist_aux.angular.z;
    //将数据格式转换
    float_to_buff(com_vel_speed,speed_sent_buf);
}


int main(int argc ,char** argv)
{
    //创建IO服务对象 io_service  serial_port 都是boost::asio下面的类
    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");   
    UART_Init(sp);                           // 设置字母位数为8位
    //串口接收和发送数组

    ros::init(argc,argv,"/odometry_publisher");
    ros::NodeHandle n;

    // 订阅cmd_vel topic,当有话题的时候到回调函数
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 100, cmd_velCallback); 
    //创建发布者，消息类型为nav_msgs::Odometry  ,topic->odom,队列长度 50
    ros::Publisher odom_pub  =  n.advertise<nav_msgs::Odometry>("odom", 20);
    //发布base_link->odom的tf转换
    tf::TransformBroadcaster odom_broadcaster;  

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    //设置循环频率hz
    ros::Rate loop_rate(250);
    while(ros::ok())
    {
        //处理回调函数
        ros::spinOnce();
        current_time = ros::Time::now();
        // 从串口读取机器人速度数据存入Speed_buf_rev   如果串口下设备未开启，会卡在这儿
        read(sp, buffer(speed_recive_buf));   
        // 对接收到的数据进行校验

        if(CRC_verify(speed_recive_buf))
        {
            //将speed_recive数据处理后传给com_vel_speed结构体 即将串口接收的数据转换成float型速度数据
            buff_to_float(speed_recive_buf,True_Speed); 
            // 打印接收到的机器人速度信息
            ROS_INFO("Recive ok");
            /**compute odometry in a typical way given the velocities of the robot**/
            double dt = (current_time - last_time).toSec();
            double delta_x = (True_Speed->x * cos(True_Speed->th) - True_Speed->y * sin(True_Speed->th)) * dt;
            double delta_y = (True_Speed->x * sin(True_Speed->th) + True_Speed->y * cos(True_Speed->th)) * dt;
            double delta_th = True_Speed->th * dt;

            position->x  += delta_x;
            position->y  += delta_y;
            position->th += delta_th;
            //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(position->th);
    
            //first, we'll publish the transform over tf
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
        
            odom_trans.transform.translation.x = position->x;
            odom_trans.transform.translation.y = position->y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;
        
            //send the transform
            odom_broadcaster.sendTransform(odom_trans);
        
            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
        
            //set the position
            odom.pose.pose.position.x = position->x;
            odom.pose.pose.position.y = position->y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
        
            //填充线速度  角速度由陀螺仪提供
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x  = True_Speed->x;
            odom.twist.twist.linear.y  = True_Speed->y;
            odom.twist.twist.angular.z = True_Speed->th;
        
            //publish the message
            odom_pub.publish(odom);
        }
        else
            ROS_INFO("Serial port communication failed!");

        last_time = current_time;
        // 串口写入由回调函数中接收的cml_vel话题消息,即向小车发送速度指令
        write(sp, buffer(speed_sent_buf, buffer_length));
        loop_rate.sleep();
        
    }
    iosev.run();
    return 0;
}
