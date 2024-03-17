/*
This file subscribe the position and velocity of the UAV, and then compute the fake movement(pos, vel) to activate the avoidance algorithm. 
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/PositionTarget.h>
#include "llh_conv.h"

#include <fstream>
#include <string>

using namespace std;

FILE *fp;
ofstream fout;
const double o_lat = 47.3977423;
const double o_lon = 8.5455930;
const double o_alt = 535.4;
const double ltogx = 180.0 / (M_PI) / (6371e3 * cos(o_lat / 180.0 * M_PI));
const double ltogy = 180.0 / (M_PI) / 6371e3;

sensor_msgs::NavSatFix current_pos;
void mypos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_pos = *msg;
}

geometry_msgs::TwistStamped current_lvel;
void local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_lvel = *msg;
}

double norm(double vx,double vy)
{
    return sqrt(pow(vx,2)+pow(vy,2));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_neighbor_node");
    ros::NodeHandle nh;

    // Subscribers
    
    ros::Subscriber pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav0/mavros/global_position/global", 10, mypos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("uav0/mavros/local_position/velocity_local", 10, local_velocity_cb);
    

    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("uav1/mavros/local_position/velocity_local", 10);

    ros::Publisher local_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
            ("uav2/mavros/local_position/local", 10);
    ros::Publisher local_vel_pub2 = nh.advertise<geometry_msgs::TwistStamped>
            ("uav2/mavros/local_position/velocity_local", 10);            
    ros::Publisher neighbor_pub = nh.advertise<sensor_msgs::NavSatFix>("uav1/mavros/global_position/global", 10);

    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    
    // wait for local position connection
    while(ros::ok() && current_lvel.header.frame_id.empty()){
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    double o_lat = 47.3977423;
    double o_lon = 8.5455930;
    double o_alt = 535.4;

    double t1,t2;
    double v0,v1,v2;
    v1=15.0;
    v2=15.0;

    t1=9.0;
    t2=11.2;

    initialiseReference(o_lat, o_lon, o_alt);

    double n, e, d;
    geodetic2Ned((double)current_pos.latitude, (double)current_pos.longitude, current_pos.altitude, &n,&e, &d);

    // crash after 10sec. approach from right of target
    float cx, cy, cz, cx2,cy2,cz2;
    cx = (float)(e + t1*current_lvel.twist.linear.x);
    cy = (float)(n + t1*current_lvel.twist.linear.y);
    cz = -(float)d;

    cx2 = (float) (e + t2*current_lvel.twist.linear.x);
    cy2 = (float)(n + t2*current_lvel.twist.linear.y);
    cz2 = -(float)d;

    printf("vel : %.2f, %.2f\n", current_lvel.twist.linear.x, current_lvel.twist.linear.y);
    printf("crsh : %.2f, %.2f\n", cx, cy);
    float vx0,vy0,vz0,vx1, vy1, vz1, vx2,vy2, vz2; //velocity
    vx0 = -current_lvel.twist.linear.y;
    vy0 = current_lvel.twist.linear.x;
    vz0 = 0;

    v0=norm(vx0,vy0);
    double th1=0.0;

    vx1 = cos(th1)*vx0-sin(th1)*vy0;
    vy1 = sin(th1)*vx0+cos(th1)*vy0;
    vz1 = vz0;

    vx1 *= v1/v0;
    vy1 *= v1/v0;
    vz1 *= v1/v0;

    double th2=0.0*M_PI/4.0;
    vx2 = cos(th2)*vx0-sin(th2)*vy0;
    vy2 = -sin(th2)*vx0+cos(th2)*vy0;
    vz2 = vz1;

    vx2 *= v2/v0;
    vy2 *= v2/v0;
    vz2 *= v2/v0;

    float spx, spy, spz;
    spx = cx - t1*vx1; //initial position.
    spy = cy - t1*vy1;
    spz = cz;

    float spx2, spy2, spz2;
    spx2 = cx2 - (t2)*vx2;
    spy2 = cy2 - (t2+0.8)*vy2;
    spz2 = cz2;

    geometry_msgs::PoseStamped pos;
    geometry_msgs::PoseStamped pos2;
    
    geometry_msgs::TwistStamped vel;
    geometry_msgs::TwistStamped vel2;
    pos.pose.position.x = spx;
    pos.pose.position.y = spy;
    pos.pose.position.z = spz;
    pos2.pose.position.x = spx2;
    pos2.pose.position.y = spy2;
    pos2.pose.position.z = spz2;
    vel.twist.linear.x = vx1;
    vel.twist.linear.y = vy1;
    vel.twist.linear.z = vz1;
    vel2.twist.linear.x = vx2;
    vel2.twist.linear.y = vy2;
    vel2.twist.linear.z = vz2;
    

    float movex = spx;
    float movey = spy;
    float movez = spz;

    float movex2 = spx2;
    float movey2 = spy2;
    float movez2 = spz2;
    sensor_msgs::NavSatFix global_pose;
    global_pose.altitude = movez;
    global_pose.latitude = o_lat+movey*ltogy;
    global_pose.longitude = o_lon+movex*ltogx;
        
    printf("move init: %.2f, %.2f, %.2f\n", movex, movey, movez);

    string filename_landing;
    filename_landing = "/home/escl/log/";
    filename_landing += to_string(ros::Time::now().toSec());
    filename_landing += ".txt";
    
    while(ros::ok()){

        ros::spinOnce();

        geodetic2Ned((double)current_pos.latitude, (double)current_pos.longitude, (double)current_pos.altitude, &n, &e, &d);
        fout.open(filename_landing, ios::app);
        fout << ros::Time::now().toSec() << "\t" << pos.pose.position.x << "\t" << pos.pose.position.y << "\t" << pos.pose.position.z << "\t" << e << "\t" << n << "\t" << -d << "\t" << cx << "\t" << cy << "\t" << cz << endl;
        if (fout.is_open() == true) {
            fout.close();
        }
        
        movex += 0.05*vx1;
        movey += 0.05*vy1;
        movez += 0.05*vz1;
        movex2 += 0.05*vx2;
        movey2 += 0.05*vy2;
        movez2 += 0.05*vz2;
        pos.pose.position.x = movex;
        pos.pose.position.y = movey;
        pos.pose.position.z = movez;
        pos2.pose.position.x = movex2;
        pos2.pose.position.y = movey2;
        pos2.pose.position.z = movez2;
        
        //printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", movex, movey, movez, vx1, vy1, vz1);
        local_pos_pub.publish(pos);
        local_vel_pub.publish(vel);

        global_pose.longitude += 0.05*vx1*ltogx;
        global_pose.latitude += 0.05*vy1*ltogy;
        global_pose.altitude +=0.05*vz1;
        
        neighbor_pub.publish(global_pose);
        //local_pos_pub2.publish(pos2);
        //local_vel_pub2.publish(vel2);
        
        rate.sleep();
    }

    return 0;
}
