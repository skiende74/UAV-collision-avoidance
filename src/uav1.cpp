#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/WaypointList.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <iostream>
#include <random>
#include <stdio.h>
#include <string>

#include "avoidance.h"
#include "state_machine.h"

using namespace std;

const int my_num = 1;
const int type = 1; // 1: 회전익 2: 고정익.
const int N = 3;
const double o_lat = 47.3977423;
const double o_lon = 8.5455930;
const double o_alt = 535.4;
const double d_m = 5.0;
const double bank_angle = M_PI / 6.0;
const double RATE = 20.0;

const double ltogx = 180.0 / (M_PI) / (6371e3 * cos(o_lat / 180.0 * M_PI));
const double ltogy = 180.0 / (M_PI) / 6371e3;

double V2 = 10.0; // 고정익 속도
double V1 = 5.0;  //회전익 속도

void llh2ned_local_update(int i); //섭스크라이브해온 llh2ned 글로벌을 로컬로 변환해서 저장!

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// Callback function for Getting my Position
sensor_msgs::NavSatFix current_pos;
geometry_msgs::PoseStamped current_lpos;

void mypos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    current_pos = *msg;
    //current_lpos.pose.position.x = (current_pos.latitude-o_lat)/ltogx;
    //current_lpos.pose.position.y = (current_pos.longitude-o_lon)/ltogy;
    //current_lpos.pose.position.z = current_pos.altitude-o_alt;
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_lpos = *msg;
}

geometry_msgs::TwistStamped current_lvel;
void local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_lvel = *msg;
}

mavros_msgs::WaypointList wp;
void waypoint_cb(const mavros_msgs::WaypointList::ConstPtr &msg)
{
    wp = *msg;
}

geometry_msgs::TwistStamped current_nlvel[2];
void local_nvelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_nlvel[0] = *msg;
}
void local_nvelocity_cb2(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_nlvel[1] = *msg;
}

geometry_msgs::PoseStamped llh2ned[2];
sensor_msgs::NavSatFix llh2ned_g[2];
void llh2ned_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) //글로벌좌표를 받아와서, 로컬좌표(uav0기준)으로 바꿔서 llh2ned에도 저장.
{
    llh2ned_g[0] = *msg;
    llh2ned_local_update(0);
}
void llh2ned_cb2(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    llh2ned_g[1] = *msg;
    llh2ned_local_update(1);
}
/*local_position/pose

sensor_msgs::NavSatFix current_npos;
void npos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_npos = *msg;
}
geometry_msgs::PoseStamped local_npos;
void local_npos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_npos = *msg;
}
*/

int main(int argc, char **argv)
{
    double v0;
    if (type == 1)
        v0 = V1;
    else if (type == 2)
        v0 = V2;
    string str0, str1, str2;
    str0 = to_string(my_num);
    str1 = to_string((my_num + 1) % N);
    str2 = to_string((my_num + 2) % N);

    ros::init(argc, argv, "uav" + str0 + "_node");
    ros::NodeHandle nh;
    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("uav" + str0 + "/mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("uav" + str0 + "/mavros/global_position/global", 10, mypos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav" + str0 + "/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("uav" + str0 + "/mavros/local_position/velocity_local", 10, local_velocity_cb);
    ros::Subscriber waypoint_sub = nh.subscribe<mavros_msgs::WaypointList>("uav" + str0 + "/mavros/mission/waypoints", 10, waypoint_cb);

    //neighbor
    ros::Subscriber llh2ned_neighbor_sub = nh.subscribe<sensor_msgs::NavSatFix>("uav" + str1 + "/mavros/global_position/global", 10, llh2ned_cb);
    ros::Subscriber llh2ned_neighbor_sub2 = nh.subscribe<sensor_msgs::NavSatFix>("uav" + str2 + "/mavros/global_position/global", 10, llh2ned_cb2);
    ros::Subscriber local_vel1_sub = nh.subscribe<geometry_msgs::TwistStamped>("uav" + str1 + "/mavros/local_position/velocity_local", 10, local_nvelocity_cb);
    ros::Subscriber local_vel1_sub2 = nh.subscribe<geometry_msgs::TwistStamped>("uav" + str2 + "/mavros/local_position/velocity_local", 10, local_nvelocity_cb2);

    //ros::Subscriber pos1_sub = nh.subscribe<sensor_msgs::NavSatFix>  //I think needless.
    //        ("uav1/mavros/global_position/global", 10, npos_cb);
    //ros::Subscriber local_npos_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //        ("uav1/mavros/local_position/pose", 10, local_npos_cb);

    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("uav" + str0 + "/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("uav" + str0 + "/mavros/setpoint_raw/local", 10);

    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("uav" + str0 + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("uav" + str0 + "/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(RATE);

    /* CONNECTION CHECK */
    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // wait for local position connection
    while (ros::ok() && current_lpos.header.frame_id.empty())
    {
        ros::spinOnce();
        rate.sleep();
    }
    string filename_str = "/home/escl2/log/" + to_string(ros::Time::now().toSec()) + "_" + to_string(my_num) + ".dat";
    const char *filename = filename_str.c_str();

    if (my_num == 0)
    {
        FILE *f = fopen(filename, "w");
        fprintf(f, " ");
        fclose(f);
    }

    /* Takeoff and hold for start */
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::PositionTarget vel;
    vel.coordinate_frame = 1;
    vel.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 1024 + 2048;
    vel.velocity.x = current_lvel.twist.linear.x;
    vel.velocity.y = current_lvel.twist.linear.y;
    vel.velocity.z = 0.0;

    //send a few setpoints before starting
    for (int i = 10; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    StateMachine SM;
    Avoidance AV;
    printf("v0: %.4f\n",v0);
    AV.initializeSpecification(v0, bank_angle, d_m, RATE);

    ros::Time last_request = ros::Time::now();
    printf("loop start\n");
    while (ros::ok())
    {

        ros::spinOnce();

        vel.coordinate_frame = 1;
        vel.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 1024 + 2048;
        vel.velocity.x = -0.6;
        vel.velocity.y = -0.6;
        vel.velocity.z = 0.0;

        printf("----------------------------------------------\n");
        // Neighbor update : excute this whenever msg income
        if (!wp.waypoints.empty())
        {
            int ii = 0;
            while (true)
            {
                //printf("wp %d : %d\n", i, wp.waypoints[i].is_current);
                if (wp.waypoints[ii].is_current == true)
                {
                    //printf("lat : %f, lon : %f\n", wp.waypoints[i].x_lat, wp.waypoints[i].y_long);
                    if (ii >= 1)
                        AV.myinfo_wp_update(wp.waypoints[ii - 1], wp.waypoints[ii]);
                    else
                        AV.myinfo_wp_update(wp.waypoints[ii], wp.waypoints[ii]);
                    break;
                }
                ii++;
            }
        }
        AV.myinfo_update(current_lvel.header.stamp.toSec(), 1, (double)current_pos.latitude, (double)current_pos.longitude, (double)current_pos.altitude, current_lvel.twist.linear.x, current_lvel.twist.linear.y, current_lvel.twist.linear.z);

        for (int i = 0; i < N - 1; i++)
        {
            if (llh2ned_g[i].header.seq != 0)
            {
                AV.neighbor_update(2 + i, 1, current_nlvel[i].header.stamp.toSec(), llh2ned_g[i].latitude, llh2ned_g[i].longitude, llh2ned_g[i].altitude, current_nlvel[i].twist.linear.x, current_nlvel[i].twist.linear.y, current_nlvel[i].twist.linear.z);
            }
        }
        // Threat checker.
        SM.condition.threat = AV.threat_checker();
        //printf("threat = %d\n", SM.condition.threat);
        // Mode checker.
        SM.condition.offboard = (current_state.mode == "OFFBOARD");
        // State machine.
        SM.state_shifter();

        Vector_t pose_data;
        pose_data = AV.getdata_myinfo();
        double r_u0[2];

        r_u0[0] = pose_data.x;
        r_u0[1] = pose_data.y;

        printf("\n[UAV0] state:%d, SM.condition.priority:%d, r_u0[0]:%.2f,r_u0[1]:%.2f\n", SM.condition.state, SM.condition.priority, r_u0[0], r_u0[1]);

        SM.condition.priority = AV.priority_check();
        
        switch (SM.condition.state)
        {
        case 1: // Idle: Auto mode. Do nothing.
            break;
        case 2: // Priority check.
            //printf("[uav0] SM.condition.priority:%d",SM.condition.priority);
            break;
        case 3:
        { // Mode changer.
            //SM.condition.mode_changed = mode_change();
            mavros_msgs::SetMode offb_set_mode;
            if (current_state.mode == "OFFBOARD")
            {
                offb_set_mode.request.custom_mode = "AUTO.MISSION";
                if (current_state.mode != "AUTO.MISSION" && (ros::Time::now() - last_request > ros::Duration(1.0))) // 5.0 ->1.0
                {
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    {
                        ROS_INFO("Auto enabled");
                        SM.condition.mode_changed = true;
                    }
                    last_request = ros::Time::now();
                }
            }
            else
            {
                offb_set_mode.request.custom_mode = "OFFBOARD";
                if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
                {
                    if (set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent)
                    {
                        ROS_INFO("Offboard enabled");
                        SM.condition.mode_changed = true;
                    }
                    last_request = ros::Time::now();
                }
            }
            //SM.condition.mode_changed = true;
            break;
        }
        case 4: // Avoidance.
            Vector_t vel_vec;
            vel_vec = AV.avoid();
            vel.velocity.x = vel_vec.x;
            vel.velocity.y = vel_vec.y;
            pose.pose.position.x = (current_pos.longitude - o_lon) / ltogx + 100.0 * vel_vec.x;
            pose.pose.position.y = (current_pos.latitude - o_lat) / ltogy + 100.0 * vel_vec.y;
            break;
        }
        if (type == 1)
            local_vel_pub.publish(vel);
        else if (type == 2)
            local_pos_pub.publish(pose);

        double r_u1[2], r_u2[2];
        r_u0[0] = current_lpos.pose.position.x;
        r_u0[1] = current_lpos.pose.position.y;
        r_u1[0] = llh2ned[0].pose.position.x;
        r_u1[1] = llh2ned[0].pose.position.y;
        r_u2[0] = llh2ned[1].pose.position.x;
        r_u2[1] = llh2ned[1].pose.position.y;

        if (my_num == 0)
        {
            FILE *f = fopen(filename, "a");
            fprintf(f, "%f %f %f %f %f %f\n", r_u0[0], r_u0[1], r_u1[0], r_u1[1], r_u2[0], r_u2[1]);
            fclose(f);
        }
        AV.neighbor_managing();

        rate.sleep();
    }

    return 0;
}

void llh2ned_local_update(int i) //섭스크라이브해온 llh2ned 글로벌을 로컬로 변환해서 저장!
{
    llh2ned[i].header.stamp = llh2ned_g[i].header.stamp;
    llh2ned[i].header.seq = llh2ned_g[i].header.seq;
    llh2ned[i].pose.position.x = (llh2ned_g[i].longitude - o_lon) / ltogx;
    llh2ned[i].pose.position.y = (llh2ned_g[i].latitude - o_lat) / ltogy;
    llh2ned[i].pose.position.z = (llh2ned_g[i].latitude - o_alt);
}
