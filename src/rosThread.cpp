#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>

RosThread::RosThread()
{
    shutdown = false;

    //  connect(poseUpdateTimer,SIGNAL(timeout()),this,SLOT(poseUpdate()));

  // networkUpdateTimer = new QTimer(this);

   // connect(networkUpdateTimer,SIGNAL(timeout()),this,SLOT(networkUpdate()));


}

/*RosThread::RosThread(int argc, char **argv, std::string nodeName){

    //  ros::init(argc, argv, nodeName);

 //   ros::init(argc,argv,nodeName);
}*/

void RosThread::work(){

    QString path = QDir::homePath();
    path.append("/fuerte_workspace/sandbox/configISL.json");

    if(!readConfigFile(path)){

        qDebug()<< "Read Config File Failed!!!";

        ros::shutdown();

        emit rosFinished();

        return;
    }


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

        NavigationController::robotContoller(vel, numOfRobots, bin, bt, b_rs, ro, kkLimits, robot.robotID);

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
    bin[robot.robotID][1] = msg->pose.pose.position.x;
    bin[robot.robotID][2] = msg->pose.pose.position.y;


    bin[1][3] = 0.33;

   // ROS_INFO("position x %4.2f position y %4.2f orientation %4.2f",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.orientation.z*180/3.14);

}
// Received neighbor info
void RosThread::neighborInfoCallback(navigationISL::neighborInfo neighborInfo)
{
    QString str = QString::fromStdString(neighborInfo.name);

    str.remove("IRobot");

    int num = str.toInt();

    if(num > 0 && num < numOfRobots){
        bin[num][1] = neighborInfo.posX;
        bin[num][2] = neighborInfo.posY;
        bin[num][3] = neighborInfo.radius;
        qDebug()<<"robot number "<<num;
    }
    else qDebug()<<"Unknown robot id number";
}

// Tc saniyede Komsulara kendi bilgisini gonderiyor
void RosThread::poseUpdate(const ros::TimerEvent&)
{
    navigationISL::robotInfo info;

    info.neighbors.resize(2);
    info.neighbors[0] = "IRobot1";
    info.neighbors[1] = "IRobot3";

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
bool RosThread::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {
       // qDebug()<<result["numrobots"].toString();

        int numrobots = result["numrobots"].toInt();

        poseUpdatePeriod = result["Tc"].toInt();

        coordinatorUpdatePeriod = result["Tg"].toInt();

        robot.robotID = result["robotID"].toInt();

        int iscoord =   result["iscoordinator"].toInt();
        if(iscoord == 1) this->robot.isCoordinator = true;

        this->robot.radius = result["radius"].toDouble();

    }
    file.close();
    return true;



}
