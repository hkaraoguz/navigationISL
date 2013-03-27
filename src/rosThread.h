#include "navigationController.h"
#include <QThread>
#include <QObject>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <QTimer>
#include "navigationISL/robotInfo.h"
#define numOfRobots 5

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

   // RosThread(int argc, char **argv, std::string nodeName);

public:

     void shutdownROS();

private:
     bool shutdown;

     QTimer* poseUpdateTimer;

     QTimer* networkUpdateTimer;

     ros::NodeHandle n;

     ros::Subscriber amclSub;

     ros::Publisher robotinfoPublisher;

     void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

     void poseUpdate(const ros::TimerEvent&);
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

     void networkUpdate();
signals:

   void  rosStarted();
   void  rosStartFailed();

};
