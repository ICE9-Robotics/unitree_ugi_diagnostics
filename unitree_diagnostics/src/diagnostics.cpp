#include "diagnostics.h"
#include "unitree_diagnostics_msgs/Diagnostics.h"

using namespace unitree_diagnostics;

Diagnostics::Diagnostics(ros::NodeHandle nh, ros::NodeHandle nh_priv)
:info(DiagnosticInfo())
{
    nh_priv.param("frequency", frequency, 10.0);

    navSatFixSub = nh.subscribe("fix", 10, &Diagnostics::navSatFixCallback, this);
    gpggaSub = nh.subscribe("nmea/gpgga", 10, &Diagnostics::gpggaCallback, this);
    gpsVelSub = nh.subscribe("reach/vel", 10, &Diagnostics::gpsVelCallback, this);
    highStateSub = nh.subscribe("high_state", 10, &Diagnostics::highStateCallback, this);
    cmdVelSub = nh.subscribe("cmd_vel", 10, &Diagnostics::cmdVelCallback, this);
    vfrSub = nh.subscribe("mavros/vfr_hud", 10, &Diagnostics::vfrCallback, this);

    diagnosticsPub = nh.advertise<unitree_diagnostics_msgs::Diagnostics>("diagnostics", 10);

    periodicUpdateTimer_ = nh.createTimer(ros::Duration(1./frequency), &Diagnostics::periodicUpdate, this);
}

Diagnostics::~Diagnostics()
{}

void Diagnostics::navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    info.navSatFixGpsStatus = msg->status.status;
    info.navSatFixTs = msg->header.stamp;
}

void Diagnostics::gpggaCallback(const nmea_msgs::Gpgga::ConstPtr &msg)
{
    info.gpggaGpsStatus = msg->gps_qual;
    info.gpggaTs = msg->header.stamp;
}

void Diagnostics::gpsVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    info.gpsVelocity = sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2) + pow(msg->linear.z, 2));
}

void Diagnostics::highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{
    info.batterySoc = msg->bms.SOC;
    info.odomVelocity = sqrt(pow(msg->velocity[0], 2) + pow(msg->velocity[1], 2) + pow(msg->velocity[2], 2));
    info.odomYawSpeed = msg->yawSpeed;
    info.highStateTs = msg->header.stamp;
    info.mode = msg->mode;
}

void Diagnostics::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    info.cmdVelocity = sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2) + pow(msg->linear.z, 2));
    info.cmdYawSpeed = msg->angular.z;
}

void Diagnostics::vfrCallback(const mavros_msgs::VFR_HUD::ConstPtr &msg)
{
    info.heading = msg->heading;
}

void Diagnostics::periodicUpdate(const ros::TimerEvent& event)
{
    unitree_diagnostics_msgs::Diagnostics msg;
    msg.header.stamp = ros::Time::now();
    msg.batterySoC = info.batterySoc;
    msg.commandVelocity = info.cmdVelocity;
    msg.gpsVelocity = info.gpsVelocity;
    msg.commandYawSpeed = info.cmdYawSpeed;
    msg.velocity = info.odomVelocity;
    msg.yawSpeed = info.odomYawSpeed;
    msg.highStateTimestamp = info.highStateTs;
    msg.heading = info.heading;
    msg.mode = info.mode;


    if (info.gpggaTs.isValid() && !info.gpggaTs.isZero())
    {
        msg.gpsStatusTimestamp = info.gpggaTs;
        if (info.gpggaGpsStatus == 0)
        {
            msg.gpsStatusDescription = "No fix";
        } 
        else if (info.gpggaGpsStatus == 1)
        {
            msg.gpsStatusDescription = "Fixed";
        }
        else if (info.gpggaGpsStatus == 2)
        {
            msg.gpsStatusDescription = "Differential";
        }
        else if (info.gpggaGpsStatus == 4)
        {
            msg.gpsStatusDescription = "RTK fixed";
        }
        else if (info.gpggaGpsStatus == 5)
        {
            msg.gpsStatusDescription = "RTK float";
        }
        else if (info.gpggaGpsStatus == 6)
        {
            msg.gpsStatusDescription = "Dead reckoning";
        }
        else if (info.gpggaGpsStatus == 9)
        {
            msg.gpsStatusDescription = "SBAS fixed";
        }
    }
    else
    {
        msg.gpsStatusTimestamp = info.navSatFixTs;
        if (info.navSatFixGpsStatus == -1)
        {
            msg.gpsStatusDescription = "No fix";
        }
        else if (info.navSatFixGpsStatus == 0)
        {
            msg.gpsStatusDescription = "Fixed";
        }
        else if (info.navSatFixGpsStatus == 1)
        {
            msg.gpsStatusDescription = "SBAS fixed";
        }
        else if (info.navSatFixGpsStatus == 2)
        {
            msg.gpsStatusDescription = "RTK fixed/float";
        }
    }
    diagnosticsPub.publish(msg);
}
