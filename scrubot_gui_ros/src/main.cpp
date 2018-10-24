#include "scrubot_gui_ros/scrubot_gui.h"
#include <QApplication>
#include <ros/ros.h>
#include <QThread>
#include <boost/thread.hpp>
#include <signal.h>

QApplication* b;

void spin_f()
{
    ros::spin();
}
void quit(int a)
{
    printf("SIGINT \n");
    b->quit();
}
void show_log(ros::NodeHandle nh){

    int i = 0;
    //while( nh.ok() ){
    //    if(i++ % 1000 == 0 )ROS_INFO_STREAM("scrubot bot log for test..." << i );
    //    if(i > 10000000) i = 0;
    //}

}
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"scrubot_gui_ros");
    ros::NodeHandle nh;

    QApplication a(argc, argv);
    b = &a;
    QTextCodec *codec = QTextCodec::codecForName("UTF-8");//情况2 LINUX 下 中文字符不乱码
    QTextCodec::setCodecForTr(codec);
    QTextCodec::setCodecForLocale(codec);
    QTextCodec::setCodecForCStrings(codec);
    scrubot_gui w;
    w.show();
    boost::thread t1(spin_f);//manage ros msg
    //boost::thread th2(boost::bind(&ReceiveMessage, 1, retdata, 1));
    boost::thread test( boost::bind(&show_log,boost::ref(nh)) );
    signal(SIGINT, quit);//ctrl + c to quit qt gui

    return a.exec();
}
