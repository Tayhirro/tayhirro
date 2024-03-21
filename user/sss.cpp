#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
int main(int argc,char **argv)
{
        //ROS初始化
        ros::init(argc, argv, "talker");
        //创建句柄
        ros::NodeHandle n;
        //使用n句柄发布节点话题 chatter_pub
        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
        //轮询率
        ros::Rate loop_rate(10);
        int count = 0;
        while(ros::ok())
        {
                //初始化信息
                std_msgs::String msg;
                std::stringstream ss;
                ss<<"hello world"<<count;
                msg.data = ss.str();
                //发布信息
                ROS_INFO("%s", msg.data.c_str());
                chatter_pub.publish(msg);
                //处理ROS的信息，比如订阅消息,并调用回调函数
                ros::spinOnce();
                //按照循环频率延时
                loop_rate.sleep();
                ++count;
        }
        return 0;
}
