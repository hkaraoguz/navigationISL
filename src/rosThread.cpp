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

   // this->amclSub = n.subscribe("amcl_pose",2,&RosThread::amclPoseCallback,this);
    this->neighborInfoSubscriber = n.subscribe("communicationISL/neighborInfo",1,&RosThread::neighborInfoCallback,this);
    this->turtlebotOdometrySubscriber = n.subscribe("odom",2,&RosThread::turtlebotOdometryCallback,this);


    this->robotinfoPublisher = n.advertise<navigationISL::robotInfo>("navigationISL/robotInfo",1);
    this->turtlebotVelPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    this->amclInitialPosePublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1,true);

    this->coordinatorUpdatePublisher = n.advertise<navigationISL::robotInfo>("navigationISL/coordinatorUpdate",1);

    //  ros::AsyncSpinner spinner(2);

    n.createTimer(ros::Duration(poseUpdatePeriod), &RosThread::poseUpdate,this);

    // spinner.start();

    // poseUpdateTimer->start();

    ros::Rate loop(20);

    bin[robot.robotID][1] = 0;
    bin[robot.robotID][2] = 0;

    geometry_msgs::PoseWithCovarianceStamped initialpose;

    initialpose.pose.pose.position.x = 0;

    initialpose.pose.pose.position.y = 0;

   // geometry_msgs::Quaternion k();

    initialpose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    amclInitialPosePublisher.publish(initialpose);

    ro = 800;

    while(ros::ok())
    {


      // NavigationController::robotContoller(vel, numOfRobots, 4, 150, bin, bt, b_rs, bp, ro, kkLimits, robot.robotID);

       //this->sendVelocityCommand();
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

    qDebug()<<"Rad yaw: "<<radYaw;

   //vel[1] = -0.5;

   // vel[0] = 0.5;

    double calYaw = atan2(-0.5,0.5);

    qDebug()<<"Cal yaw: "<<calYaw;

    double diffYaw = radYaw-calYaw;

    qDebug()<<"Diff yaw: "<<diffYaw;

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

      //  turtlebotVelPublisher.publish(twist);

    }
    else
    {

        if(fabs(robot.targetX-bin[robot.robotID][1]) > 10 && fabs(robot.targetY-bin[robot.robotID][2] > 10))

            twist.linear.x = 0.2;
        else
            twist.linear.x = 0;

     //   turtlebotVelPublisher.publish(twist);


    }

    velocityVector = twist;
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

        kkLimits[0] = 2;
        kkLimits[1] = 20;
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
    if(fabs(radYaw-calYaw) >= 3*M_PI/180)
    {

        calculateTurn(calYaw,radYaw);

        return;
    }
    else
    {


        if(fabs(robot.targetX-bin[robot.robotID][1]) > 10 || fabs(robot.targetY-bin[robot.robotID][2]) > 10){
            qDebug()<<"Linear";
            velocityVector.linear.x = 0.2;
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

    if(desired < 0)
    {
        desired += M_PI*2;
    }

    if(current < 0)
    {
        current += M_PI*2;
    }

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

    velocityVector.angular.z = dir*0.4;

}
