#include "rosThread.h"
#include <QDebug>

RosThread::RosThread()
{
    shutdown = false;

    //  connect(poseUpdateTimer,SIGNAL(timeout()),this,SLOT(poseUpdate()));

    networkUpdateTimer = new QTimer(this);

    connect(networkUpdateTimer,SIGNAL(timeout()),this,SLOT(networkUpdate()));


}

/*RosThread::RosThread(int argc, char **argv, std::string nodeName){

    //  ros::init(argc, argv, nodeName);

 //   ros::init(argc,argv,nodeName);
}*/

void RosThread::work(){

    //  int argc; // Player Main() method does not take argument
    //  char **argv; // What to do with argc and argv??

    // const M_string nnn;

    //   ros::init(argc,argv,"ISLFramework");

    if(!ros::ok()){

        emit rosStartFailed();

        return;
    }

    emit rosStarted();

    this->amclSub = n.subscribe("amcl_pose",2,&RosThread::amclPoseCallback,this);

    this->robotinfoPublisher = n.advertise<navigationISL::robotInfo>("navigationISL/robotInfo",1);

    this->coordinatorUpdatePublisher = n.advertise<navigationISL::robotInfo>("navigationISL/coordinatorUpdate",1);

    this->neighborInfoSubscriber = n.subscribe("communicationISL/neighborInfo",1,&RosThread::neighborInfoCallback,this);
    //  ros::AsyncSpinner spinner(2);

    ros::Timer timer = n.createTimer(ros::Duration(2), &RosThread::poseUpdate,this);

    // spinner.start();

    // poseUpdateTimer->start();

    ros::Rate loop(10);

    while(ros::ok()){

        NavigationController::robotContoller(vel, numOfRobots, bin, bt, b_rs, ro, kkLimits);

        //   ros::spinOnce();

        //   loop.sleep();
        ros::spinOnce();
        loop.sleep();


    }

    qDebug()<<"I am quitting";

    ros::shutdown();

    emit rosFinished();


}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;


}
void RosThread::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    bin[1][1] = msg->pose.pose.position.x;
    bin[1][2] = msg->pose.pose.position.y;
    bin[1][3] = 0.33;

   // ROS_INFO("position x %4.2f position y %4.2f orientation %4.2f",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.orientation.z*180/3.14);

}
// Received neighbor info
void RosThread::neighborInfoCallback(navigationISL::neighborInfo neighborInfo)
{
    QString str = QString::fromStdString(neighborInfo.name);

    str.remove("IRobot");

    int num = str.toInt();

    if(num > 1 && num < numOfRobots){
        bin[num][1] = neighborInfo.posX;
        bin[num][2] = neighborInfo.posY;
        bin[num][3] = neighborInfo.radius;
       // qDebug()<<"robot number "<<num;
    }
    else qDebug()<<"Unknown robot id number";
}

// Tc saniyede Komsulara kendi bilgisini gonderiyor
void RosThread::poseUpdate(const ros::TimerEvent&)
{
    navigationISL::robotInfo info;

    info.neighbors.resize(2);
    info.neighbors[0] = "IRobot3";
    info.neighbors[1] = "mehmet";

    info.posX = 5;

    info.posY = 5;

    info.targetX = 10;

    info.targetY = 40;

    info.radius = 0.33;


    robotinfoPublisher.publish(info);



}

// Tg saniyede Coordinator a kendi konum bilgisini gonderiyor
void RosThread::coordinatorUpdate(const ros::TimerEvent&)
{
    navigationISL::robotInfo info;

    info.posX = bin[1][1];

    info.posY = bin[1][2];

    this->coordinatorUpdatePublisher.publish(info);



}
