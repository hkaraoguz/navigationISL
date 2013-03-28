
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include "navigationController.h"
#include "navigationISL/robotInfo.h"
#include "navigationISL/neighborInfo.h"
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>





#define numOfRobots 5

class Robot
{
public:
    int robotID;
    bool isCoordinator;
    double radius;
    double targetX;
    double targetY;

};
class Obstacle
{
public:
    int id;
    double radius;
    double x;
    double y;


};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

    Robot robot;

    QVector<Obstacle> obstacles;

   // RosThread(int argc, char **argv, std::string nodeName);

public:

     bool readConfigFile(QString filename);

private:
     bool shutdown;

  //   navigationISL::robotInfo currentStatus;

     ros::NodeHandle n;

     ros::Subscriber amclSub;

     ros::Subscriber neighborInfoSubscriber;

     ros::Publisher robotinfoPublisher;

     ros::Publisher coordinatorUpdatePublisher;

     ros::Publisher turtlebotVelPublisher;

     ros::Publisher amclInitialPosePublisher;

     void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

     void neighborInfoCallback(navigationISL::neighborInfo neighborInfo);

     void poseUpdate(const ros::TimerEvent&);

     void coordinatorUpdate(const ros::TimerEvent&);

     void robotContoller(double [], int , double [][4], double [][3], double [][4], double, double []);

    // int numOfRobots;

     double vel[2]; // velocity vector
     double bin[numOfRobots+1][4];// positions including itself
     double bt[numOfRobots+1][3]; // goal positions
     double rr[numOfRobots+1]; // radii of the robots
     double b_rs[numOfRobots+1][4]; // robots' positions within sensing range
     double ro;
     double kkLimits[2]; // upper and lower bounds of parameters in navigation function
     double bp[5][4];
     int poseUpdatePeriod;
     int coordinatorUpdatePeriod;


public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void  rosStarted();
   void  rosStartFailed();

};
