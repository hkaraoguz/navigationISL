#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

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
    this->turtlebotVelPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

    this->amclInitialPosePublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1,true);

    this->coordinatorUpdatePublisher = n.advertise<navigationISL::robotInfo>("navigationISL/coordinatorUpdate",1);

    this->neighborInfoSubscriber = n.subscribe("communicationISL/neighborInfo",1,&RosThread::neighborInfoCallback,this);
    //  ros::AsyncSpinner spinner(2);

    n.createTimer(ros::Duration(poseUpdatePeriod), &RosThread::poseUpdate,this);

    // spinner.start();

    // poseUpdateTimer->start();

    ros::Rate loop(10);

    geometry_msgs::PoseWithCovarianceStamped initialpose;

    initialpose.pose.pose.position.x = 0;

    initialpose.pose.pose.position.y = 0;

   // geometry_msgs::Quaternion k();

    initialpose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    amclInitialPosePublisher.publish(initialpose);

    while(ros::ok())
    {


        NavigationController::robotContoller(vel, numOfRobots, 4, 150, bin, bt, b_rs, bp, ro, kkLimits, robot.robotID);

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
    bin[robot.robotID][3] = robot.radius;

    double radYaw = tf::getYaw(msg->pose.pose.orientation);

    double calYaw = atan2(vel[1],vel[0]);

    double diffYaw = radYaw-calYaw;

    geometry_msgs::Twist twist;

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    if(fabs(diffYaw) > 0.09)
    {
        if(diffYaw > 0)
        {

            twist.angular.z = 0.1;

            //turtlebotVelPublisher.publish(twist);

        }
        else
        {

            twist.angular.z = -0.1;



        }

        turtlebotVelPublisher.publish(twist);

    }
    else
    {

        if(fabs(robot.targetX-bin[robot.robotID][1]) > 10 && fabs(robot.targetY-bin[robot.robotID][2] > 10))

            twist.linear.x = 0.2;
        else
            twist.linear.x = 0;

        turtlebotVelPublisher.publish(twist);


    }

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

    info.posX = bin[robot.robotID][1];

    info.posY = bin[robot.robotID][2];

    info.targetX = robot.targetX;

    info.targetY = robot.targetY;

    info.radius = robot.radius;


    robotinfoPublisher.publish(info);



}

// Tg saniyede Coordinator a kendi konum bilgisini gonderiyor
void RosThread::coordinatorUpdate(const ros::TimerEvent&)
{
    navigationISL::robotInfo info;

    info.posX = bin[robot.robotID][1];

    info.posY = bin[robot.robotID][2];

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


        int numrobots = result["numrobots"].toInt();

        qDebug()<<result["numrobots"].toString();

        poseUpdatePeriod = result["Tc"].toInt();

        qDebug()<<result["Tc"].toString();

        coordinatorUpdatePeriod = result["Tg"].toInt();

        qDebug()<<result["Tg"].toString();

        robot.robotID = result["robotID"].toInt();

        qDebug()<<result["robotID"].toString();

        int iscoord =   result["iscoordinator"].toInt();
        if(iscoord == 1) this->robot.isCoordinator = true;

        this->robot.radius = result["radius"].toDouble();

        qDebug()<<result["radius"].toString();

        this->robot.targetX = result["targetX"].toDouble();

        qDebug()<<result["targetX"].toString();

        this->robot.targetY = result["targetY"].toDouble();

        qDebug()<<result["targetY"].toString();
        QVariantMap nestedMap = result["Obstacles"].toMap();

        this->obstacles.resize(nestedMap.size());

        int count = 0;
        foreach (QVariant plugin, nestedMap["Obstacle"].toList()) {

            Obstacle obstacle;

            obstacle.id = plugin.toMap()["id"].toInt();

            qDebug()<<obstacle.id;

            obstacle.radius = plugin.toMap()["radius"].toDouble();

            obstacle.x= plugin.toMap()["x"].toDouble();

            obstacle.y = plugin.toMap()["y"].toDouble();

          //  if(coord == 1) robot->setCoordinator(true);

            obstacles[obstacle.id] = obstacle;

            count++;
           // qDebug() << "\t-" << plugin.toMap()["ip"].toString();
        }
        for(int i = 0 ; i < obstacles.size(); i++)
        {

            bp[i+1][1] = obstacles[i].x;
            bp[i+1][2] = obstacles[i].y;
            bp[i+1][3] = obstacles[i].radius;

        }


    }
    file.close();
    return true;



}
