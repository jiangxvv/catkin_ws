#include"ros/ros.h"//导入ros核心头文件
#include"std_msgs/String.h"//导入std_msgs/String消息头文件，由std_msgs包的string.msg文件自动生成
#include<sstream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker1");//初始化ROS，指定节点名为talker
    ros::NodeHandle n;//实例化节点
    ros::Publisher chatter_pub=n.advertise<std_msgs::String>("chatter",1000);//发布一个名为chatter的的话题的消息，消息类型为std_msgs/String，消息队列大小为1000，超过后旧的消息丢弃
    ros::Rate loop_rate(10);//发消息的频率，这里指每秒10次，通过 Rate::sleep()来处理睡眠的时间来控制对应的发布频率。
    int count=0;
    while(ros::ok())//默认roscpp会植入一个SIGINT处理机制，当按下Ctrl-C，就会让ros::ok()返回false,那循环就会结束
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss<<"hello world"<<count;
        msg.data=ss.str();//实例化消息msg, 定义字符串流“hello world”并赋值给ss, 最后转成为字符串赋值给msg.data
        ROS_INFO("%s",msg.data.c_str());//输出调试信息
        chatter_pub.publish(msg);//实际发布的函数
        ros::spinOnce();//不是必需的，但是保持增加这个调用，是好习惯。如果程序里也有订阅话题，就必需，否则回调函数不能起作用。
        loop_rate.sleep();
        ++count;
    }
    return 0;
}



/*ros::ok() 返回false的几种情况：

    SIGINT收到(Ctrl-C)信号
    另一个同名节点启动，会先中止之前的同名节点
    ros::shutdown()被调用
    所有的ros::NodeHandles被销毁
*/