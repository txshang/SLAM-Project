#include <ros/ros.h>
#include <tf/tf.h>
#include <serial/serial.h>
#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <math.h>
#include "sensor_msgs/Imu.h"
#include <imu_test/math/Vector3.h>
#include <imu_test/math/Quaternion.h>
#include <imu_test/math/Utils.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
using namespace std;
using namespace octomath;
serial::Serial ser;
//数据解析8->32 
uint32_t comb32(uint8_t first ,uint8_t second ,uint8_t third ,uint8_t fouth) 
{ 
    uint32_t temp; 
    temp = ((uint32_t)first<<24); 
    temp |= ((uint32_t)second<<16); 
    temp |= ((uint32_t)third<<8); 
    temp |= ((uint32_t)fouth); 
    return temp; 
}
//数据解析8->16 
uint16_t comb16(uint8_t first,uint8_t second)
{ 
    uint16_t temp=0; 
    temp = ((uint16_t)first<<8); 
    temp |=((uint16_t)second); 
    return temp; 
}
//数据转换uint32_t -> float
float uint32_float(uint32_t f)
{
    static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");
    float ret;
    std::memcpy(&ret, &f, sizeof(float));
    return ret;
}
class Imu
{
public:
    Imu();
    ~Imu();
    void update(float linear_acc_x, float linear_acc_y, float linear_acc_z, ros::Time last_time,Vector3 &tmp_P,Vector3 &tmp_V);
    float linear_acc_x;
    float linear_acc_y;
    float linear_acc_z;//线加速度
    float angular_acc_x;
    float angular_acc_y;
    float angular_acc_z;//角加速度
    float pitch;
    float roll;
    float yaw;//欧拉角
private:
    ros::Publisher imu_pub;
    ros::Publisher odom_pub;
};
Imu::Imu(){
    ros::NodeHandle ph_nh_("~");
    string ttyusb_port;
    ph_nh_.param<string>("ttyusb_port",ttyusb_port,string("/dev/ttyUSB0"));
    imu_pub = ph_nh_.advertise<sensor_msgs::Imu>("/imu", 1000, false);
    odom_pub = ph_nh_.advertise<nav_msgs::Odometry>("/odom", 1000, false);
    try
    {
        ser.setPort(ttyusb_port.c_str());
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM(string("Unable to open port ")+ttyusb_port);
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM(string("Serial Port initialized")+ttyusb_port);
    }
    else
      ROS_INFO_STREAM("Serial Port Error!!!");
}
Imu::~Imu(){
    ser.close();
}
void Imu::update(float linear_acc_x, float linear_acc_y, float linear_acc_z, ros::Time last_time,Vector3 &tmp_P,Vector3 &tmp_V)
{
    //Fill the IMU message
    //欧拉角转四元数
    geometry_msgs::Quaternion odom_quat;
    pitch = DEG2RAD(-pitch);
    roll = DEG2RAD(-roll);
    yaw = DEG2RAD(-yaw);
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);//返回四元数
    sensor_msgs::Imu imu_data;
    // Fill orientation quaternion
    imu_data.orientation = odom_quat;

    // Fill the header
    ros::Time current_time_ros,last_time_ros;
    current_time_ros = ros::Time::now();
    imu_data.header.stamp = current_time_ros;
    imu_data.header.frame_id = "imu";


    // Fill angular velocity data
    //暂时无此数据
    /*
    imu_msg.angular_velocity.x = angular_acc_x;
    imu_msg.angular_velocity.y = angular_acc_y;
    imu_msg.angular_velocity.z = angular_acc_z;
    */

    // Fill linear acceleration data
    imu_data.linear_acceleration.x = linear_acc_x;
    imu_data.linear_acceleration.y = linear_acc_y;
    imu_data.linear_acceleration.z = linear_acc_z;
    //Fill covariance matrices(TODO)
    double e=1e-7;
    imu_data.orientation_covariance = {e,0,0, 0,e,0, 0,0,e};
    e=1e-6;
    imu_data.angular_velocity_covariance = {e,0,0, 0,e,0, 0,0,e};
    imu_data.linear_acceleration_covariance = {e,0,0, 0,e,0, 0,0,e};
        
    // Publish the messages
    imu_pub.publish(imu_data);  

    //imu模拟里程计发布
    Quaternion tmp_Q = Quaternion(odom_quat.w,odom_quat.x,odom_quat.y,odom_quat.z);
    Vector3 imu_linear_acc = Vector3(linear_acc_x,linear_acc_y,linear_acc_z);

    double dt =(double) (current_time_ros - last_time).toSec();

    
    tmp_V(0)=tmp_V(0)+linear_acc_x*dt;
    tmp_V(1)=tmp_V(1)+linear_acc_y*dt;

    //Vector3 acc= tmp_Q*imu_linear_acc; 

    tmp_P(0) = tmp_P(0) + tmp_V(0)*dt+0.5*dt*dt*linear_acc_x; 
    tmp_P(1) = tmp_P(1) + tmp_V(1)*dt+0.5*dt*dt*linear_acc_y; 
    

    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_ros;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = tmp_P(0);
    odom_trans.transform.translation.y = tmp_P(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_ros;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = tmp_P(0);
    odom.pose.pose.position.y = tmp_P(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    //暂时无速度信息
    //odom.twist.twist.linear.x = vx;
    //odom.twist.twist.linear.y = vy;
    //odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

}
int main(int argc, char** argv) 
{
    ros::init(argc,argv,"imu_read");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
    Imu my_imu;
    uint32_t last_time ;
    uint8_t imu_data[100]; //定义串口数据存放数组
    Vector3 tmp_P = Vector3(0,0,0);//位姿
    Vector3 tmp_V = Vector3(0,0,0);//速度
    while(ros::ok()){
        if(ser.available()){
            ROS_INFO_STREAM("serial available: "<<ser.available());
            ser.read(imu_data,ser.available());
            for(int i =0;i<100;i++){
                if(imu_data[i]==0x03)
                {
                    if(imu_data[i+1]==0x50)
                    {
                        ROS_INFO_STREAM("read data Success");
                        uint16_t start = comb16(imu_data[i],imu_data[i+1]);
                        //ROS_INFO_STREAM("start: "<<start);
                        uint32_t current_time = comb32(imu_data[i+2],imu_data[i+3],imu_data[i+4],imu_data[i+5]);
                        ROS_INFO_STREAM("time: "<<current_time);
                        uint32_t time_delay = current_time - last_time;
                        ROS_INFO_STREAM("time_delay: "<<time_delay);
                        last_time = current_time;
                        uint32_t linear_x = comb32(imu_data[i+6],imu_data[i+7],imu_data[i+8],imu_data[i+9]);
                        my_imu.linear_acc_x = uint32_float(linear_x);
                        //ROS_INFO_STREAM("linear_acc_x: "<<my_imu.linear_acc_x);
                        uint32_t linear_y = comb32(imu_data[i+10],imu_data[i+11],imu_data[i+12],imu_data[i+13]);
                        my_imu.linear_acc_y = uint32_float(linear_y);
                        //ROS_INFO_STREAM("linear_acc_y: "<<my_imu.linear_acc_y);
                        uint32_t linear_z = comb32(imu_data[i+14],imu_data[i+15],imu_data[i+16],imu_data[i+17]);
                        my_imu.linear_acc_z = uint32_float(linear_z);
                        ROS_INFO_STREAM("linear_acc_z: "<<my_imu.linear_acc_z);
                        /*
                        my_imu.angular_acc_x = uint32_float((imu_data[i+18],imu_data[i+19],imu_data[i+20],imu_data[i+21]));
                        ROS_INFO_STREAM("angular_acc_x: "<<my_imu.angular_acc_x);
                        my_imu.angular_acc_x = uint32_float(comb32(imu_data[i+22],imu_data[i+23],imu_data[i+24],imu_data[i+25]));
                        ROS_INFO_STREAM("angular_acc_y: "<<my_imu.angular_acc_y);
                        my_imu.angular_acc_x = uint32_float(comb32(imu_data[i+26],imu_data[i+27],imu_data[i+28],imu_data[i+29]));
                        ROS_INFO_STREAM("angular_acc_z: "<<my_imu.angular_acc_z);
                        */
                        uint32_t pitch_32 = comb32(imu_data[i+30],imu_data[i+31],imu_data[i+32],imu_data[i+33]);
                        my_imu.pitch = uint32_float(pitch_32);
                        //ROS_INFO_STREAM("pitch: "<<my_imu.pitch);
                        uint32_t roll_32 = comb32(imu_data[i+34],imu_data[i+35],imu_data[i+36],imu_data[i+37]);
                        my_imu.roll = uint32_float(roll_32);
                        //ROS_INFO_STREAM("roll: "<<my_imu.roll);
                        uint32_t yaw_32 = comb32(imu_data[i+38],imu_data[i+39],imu_data[i+40],imu_data[i+41]);
                        my_imu.yaw = uint32_float(yaw_32);
                        //ROS_INFO_STREAM("yaw: "<<my_imu.yaw);
                        ros::Time current_time_ros,last_time_ros;
                        current_time_ros = ros::Time::now();
                        if(time_delay <= 10)my_imu.update(my_imu.linear_acc_x,my_imu.linear_acc_y,my_imu.linear_acc_z,current_time_ros,tmp_P,tmp_V);
                    }
                }
            }
        }
        //读取数据 
        ros::spinOnce();
        loop_rate.sleep();
    }
}














