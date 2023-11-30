#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
// #include <nav_msgs/Odometry.h>
#include <nmea_msgs/Gpgga.h>
#include <unitree_legged_msgs/HighState.h>

using namespace std;

namespace unitree_diagnostics
{

    struct DiagnosticInfo
    {
        int navSatFixGpsStatus = -1;
        ros::Time navSatFixTs;
        int gpggaGpsStatus;
        ros::Time gpggaTs;
        double cmdVelocity;
        double cmdYawSpeed;
        ros::Time cmdVelTs;
        int batterySoc;
        ros::Time highStateTs;
        float odomVelocity;
        double odomYawSpeed;
        // double odomTs;
    };

    class Diagnostics
    {
    private:
        DiagnosticInfo info;
        double frequency;

        ros::NodeHandle node;
        ros::Subscriber navSatFixSub;
        ros::Subscriber gpggaSub;
        ros::Subscriber highStateSub;
        ros::Subscriber cmdVelSub;
        // ros::Subscriber odomSub;

        ros::Publisher diagnosticsPub;
        ros::Timer periodicUpdateTimer_;
        
    public:
        Diagnostics(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~Diagnostics();

        void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void gpggaCallback(const nmea_msgs::Gpgga::ConstPtr &msg);
        void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg);
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
        // void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

        void periodicUpdate(const ros::TimerEvent& event);

    };
}
