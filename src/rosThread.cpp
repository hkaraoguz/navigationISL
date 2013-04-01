#include <tf/tf.h>


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
    this->neighborInfoSubscriber = n.subscribe("communicationISL/neighborInfo",1,&RosThread::neighborInfoCallback,this);
   // this->turtlebotOdometrySubscriber = n.subscribe("odom",2,&RosThread::turtlebotOdometryCallback,this);



    this->turtlebotVelPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    this->amclInitialPosePublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1,true);

    this->coordinatorUpdatePublisher = n.advertise<navigationISL::robotInfo>("navigationISL/coordinatorUpdate",1);
    this->robotinfoPublisher = n.advertise<navigationISL::robotInfo>("navigationISL/robotInfo",1);
    //  ros::AsyncSpinner spinner(2);

    pt = n.createTimer(ros::Duration(poseUpdatePeriod), &RosThread::poseUpdate,this);

    ct = n.createTimer(ros::Duration(coordinatorUpdatePeriod),&RosThread::coordinatorUpdate,this);

    ros::Rate loop(10);

    pt.start();
    ct.start();

    bin[robot.robotID][1] = 0;
    bin[robot.robotID][2] = 0;

    geometry_msgs::PoseWithCovarianceStamped initialpose;

    initialpose.pose.pose.position.x = 0;

    initialpose.pose.pose.position.y = 0;

    initialpose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

 //   amclInitialPosePublisher.publish(initialpose);

    velocityVector.linear.x = 0.1;

    while(ros::ok())
    {


       NavigationController::robotContoller(vel, numOfRobots, obstacles.size(), partDist, bin, bt, b_rs, bp, ro, kkLimits, robot.robotID);

       qDebug()<<"Velocities: "<<vel[0]<<vel[1];

       this->sendVelocityCommand();

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
    bin[robot.robotID][1] = msg->pose.pose.position.x*100;
    bin[robot.robotID][2] = msg->pose.pose.position.y*100;
    bin[robot.robotID][3] = robot.radius;

    btQuaternion odomquat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

    double d1,d2,radYaw;
    btMatrix3x3(odomquat).getEulerYPR(radYaw,d1,d2);
   // double radYaw = tf::getYaw(odomquat);

    qDebug()<<"Rad yaw: "<<radYaw;

    double calYaw = atan2(vel[1],vel[0]);

    qDebug()<<"Bin: "<<bin[robot.robotID][1]<<"Bin 2: "<<bin[robot.robotID][2];
    qDebug()<<"Cal yaw: "<<calYaw;

    if(calYaw < 0)
    {
        calYaw += M_PI*2;
    }
    if(radYaw < 0)
    {
        radYaw += M_PI*2;
    }


    geometry_msgs::Twist twist;

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    velocityVector = twist;


    if(fabs(radYaw-calYaw) > angleThreshold*M_PI/180)
    {

        calculateTurn(calYaw,radYaw);

        return;
    }
    else
    {


        if(fabs(robot.targetX-bin[robot.robotID][1]) > 10 || fabs(robot.targetY-bin[robot.robotID][2]) > 10){
            qDebug()<<"Linear";
            velocityVector.linear.x = 0.15;
        }
        else
            velocityVector.linear.x = 0;

     //   turtlebotVelPublisher.publish(twist);


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

        bt[num][1] = neighborInfo.targetX;
        bt[num][2] = neighborInfo.targetY;
        qDebug()<<"robot number "<<num;
    }
    else qDebug()<<"Unknown robot id number";
}

// Tc saniyede Komsulara kendi bilgisini gonderiyor
void RosThread::poseUpdate(const ros::TimerEvent&)
{

    navigationISL::robotInfo info;

    info.neighbors.resize(2);
    info.neighbors[0] = "IRobot2";
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
    pt.stop();

    navigationISL::robotInfo info;

    info.neighbors.resize(1);
    info.neighbors[0] = "";
    info.posX = bin[robot.robotID][1];

    info.posY = bin[robot.robotID][2];

    info.radius = 0;
    info.targetX = 0;
    info.targetY = 0;

    this->coordinatorUpdatePublisher.publish(info);

    pt.start();

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

        numrobots = result["numrobots"].toInt();

        qDebug()<<result["numrobots"].toString();

        coordinatorUpdatePeriod = result["Tg"].toInt();

        qDebug()<<result["Tg"].toString();

        poseUpdatePeriod = result["Tc"].toInt();

        qDebug()<<result["Tc"].toString();

        this->robot.radius = result["radius"].toDouble();

        qDebug()<<result["radius"].toString();

        this->linearVelocity = result["linearVelocity"].toDouble();

        this->angularVelocity = result["angularVelocity"].toDouble();

        this->angleThreshold = result["angleThreshold"].toInt();

        this->distanceThreshold = result["distanceThreshold"].toInt();

        qDebug()<<distanceThreshold;

        ro = result["ro"].toInt();

        qDebug()<<ro;

        kkLimits[0] = result["kMin"].toInt();

        qDebug()<<kkLimits[0];

        kkLimits[1] = result["kMax"].toInt();

        qDebug()<<kkLimits[1];

        partDist = result["partDist"].toInt();

        qDebug()<<partDist;

        robot.robotID = result["robotID"].toInt();

        qDebug()<<result["robotID"].toString();

        int iscoord =   result["iscoordinator"].toInt();
        if(iscoord == 1) this->robot.isCoordinator = true;



        this->robot.targetX = result["targetX"].toDouble();

        qDebug()<<result["targetX"].toString();

        this->robot.targetY = result["targetY"].toDouble();

        qDebug()<<result["targetY"].toString();
        QVariantMap nestedMap = result["Obstacles"].toMap();

      //  this->obstacles.resize(4);

        int count = 0;
        foreach (QVariant plugin, nestedMap["Obstacle"].toList()) {

            Obstacle obstacle;

            obstacle.id = plugin.toMap()["id"].toInt();

            qDebug()<<obstacle.id;

            obstacle.radius = plugin.toMap()["radius"].toDouble();

            obstacle.x= plugin.toMap()["x"].toDouble();

            obstacle.y = plugin.toMap()["y"].toDouble();

          //  if(coord == 1) robot->setCoordinator(true);

            obstacles.push_back(obstacle);

            count++;
           // qDebug() << "\t-" << plugin.toMap()["ip"].toString();
        }
        for(int i = 0 ; i < obstacles.size(); i++)
        {

            bp[i+1][1] = obstacles[i].x;
            bp[i+1][2] = obstacles[i].y;
            bp[i+1][3] = obstacles[i].radius;

        }
        for(int i = 1; i <= numOfRobots; i++)
        {
            if(i  != robot.robotID){

                bin[i][1] = 0;
                bin[i][2] = 0;
                bin[i][3] = 0;

                bt[i][1] = 0;
                bt[i][2] = 0;




            }
            else{

                bt[i][1] = robot.targetX;
                bt[i][2] = robot.targetY;
            }

            b_rs[i][1] = 0;
            b_rs[i][2] = 0;
            b_rs[i][3] = 0;

        }


    }
    file.close();
    return true;



}
void RosThread::sendVelocityCommand()
{

    turtlebotVelPublisher.publish(velocityVector);
}
void RosThread::turtlebotOdometryCallback(const nav_msgs::Odometry &msg)
{

    bin[robot.robotID][1] = msg.pose.pose.position.x*100;
    bin[robot.robotID][2] = msg.pose.pose.position.y*100;
    bin[robot.robotID][3] = robot.radius;

    btQuaternion odomquat(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);

    double d1,d2,radYaw;
    btMatrix3x3(odomquat).getEulerYPR(radYaw,d1,d2);
   // double radYaw = tf::getYaw(odomquat);

    qDebug()<<"Rad yaw: "<<radYaw;

    double cradyaw = cos(radYaw);
    double sradyaw = sin(radYaw);

   //vel[1] = -0.5;

   // vel[0] = 0.5;

    double calYaw = atan2((robot.targetY-bin[robot.robotID][2]),(robot.targetX-bin[robot.robotID][1]));

    qDebug()<<"Bin: "<<bin[robot.robotID][1]<<"Bin 2: "<<bin[robot.robotID][2];
    qDebug()<<"Cal yaw: "<<calYaw;

  /*  double ccalyaw = cos(calYaw);
    double scalyaw = sin(calYaw);

    double diffYaw = radYaw-calYaw;

    qDebug()<<"Diff yaw: "<<diffYaw;*/

    geometry_msgs::Twist twist;

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    velocityVector = twist;

   /* if((fabs(ccalyaw - cradyaw) > (double)0.5 && ccalyaw*cradyaw < 0 )|| (fabs(scalyaw-sradyaw) > (double)0.5 && scalyaw*sradyaw < 0))
    {

        if(calYaw >= 0)
        {

            twist.angular.z = 0.3;

            //turtlebotVelPublisher.publish(twist);

        }
        else
        {

            twist.angular.z = -0.3;



        }

      //  turtlebotVelPublisher.publish(twist);

    }*/
    if(fabs(radYaw-calYaw) >= angleThreshold*M_PI/180)
    {

        calculateTurn(calYaw,radYaw);

        return;
    }
    else
    {


        if(fabs(robot.targetX-bin[robot.robotID][1]) > distanceThreshold || fabs(robot.targetY-bin[robot.robotID][2]) > distanceThreshold){
            qDebug()<<"Linear";
            velocityVector.linear.x = linearVelocity;
        }
        else
            velocityVector.linear.x = 0;

     //   turtlebotVelPublisher.publish(twist);


    }

  //  velocityVector = twist;


}
void RosThread::calculateTurn(double desired, double current)
{
    int dir = 0;

   /* if(desired < 0)
    {
        desired += M_PI*2;
    }

    if(current < 0)
    {
        current += M_PI*2;
    }*/

    double diff = desired-current;

    if(diff > 0 )
    {
           dir = 1;
    }
    else
    {
        dir = -1;
    }

    if(fabs(M_PI*2 - fabs(diff)) < fabs(diff))
    {
        dir = dir*-1;
    }

    velocityVector.angular.z = dir*angularVelocity;

}
