#include "navigationController.h"
#include "navigationISL/robotInfo.h"
#include "navigationISL/neighborInfo.h"
#include <QThread>
#include <QObject>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

#include <QTimer>

#define numOfRobots 5

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

   // RosThread(int argc, char **argv, std::string nodeName);

public:



private:
     bool shutdown;

  //   navigationISL::robotInfo currentStatus;

     QTimer* poseUpdateTimer;

     QTimer* networkUpdateTimer;

     ros::NodeHandle n;

     ros::Subscriber amclSub;

     ros::Subscriber neighborInfoSubscriber;

     ros::Publisher robotinfoPublisher;

     ros::Publisher coordinatorUpdatePublisher;

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

     int poseUpdatePeriod;
     int networkUpdatePeriod;

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void  rosStarted();
   void  rosStartFailed();

};
