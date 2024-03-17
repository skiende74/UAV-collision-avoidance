#include <ros/ros.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h> //wp
#include <mavros_msgs/WaypointList.h>  //wp
#include <mavros_msgs/WaypointPush.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <iostream>
#include <stdio.h>

using namespace std;
const int N = 3;

const double x00 = 8.5455942;
const double y00 = 47.3977432;
const double z00 = 0.0;

const double ltogy = 180.0 / M_PI / 6371e3;
const double ltogx = 180.0 / M_PI / (6371e3 * cos(y00 / 180.0 * M_PI));

mavros_msgs::State state[N];

void state_cb0(const mavros_msgs::State::ConstPtr &msg)
{
    state[0] = *msg;
}
void state_cb1(const mavros_msgs::State::ConstPtr &msg)
{
    state[1] = *msg;
}
void state_cb2(const mavros_msgs::State::ConstPtr &msg)
{
    state[2] = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "start_node");

    ros::NodeHandle nh;
    ros::Subscriber state_sub[N];
    ros::ServiceClient wp_client[N], arming_client[N], set_mode_client[N];

    // Publishers

    state_sub[0] = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, state_cb0);
    wp_client[0] = nh.serviceClient<mavros_msgs::WaypointPush>("/uav0/mavros/mission/push");
    arming_client[0] = nh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    set_mode_client[0] = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");

    state_sub[1] = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, state_cb1);
    wp_client[1] = nh.serviceClient<mavros_msgs::WaypointPush>("/uav1/mavros/mission/push");
    arming_client[1] = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    set_mode_client[1] = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");

    if (N >= 3)
    {
        state_sub[2] = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state", 10, state_cb2);
        wp_client[2] = nh.serviceClient<mavros_msgs::WaypointPush>("/uav2/mavros/mission/push");
        arming_client[2] = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
        set_mode_client[2] = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
    }

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    mavros_msgs::CommandBool arming[N];//기동 (서비스 인풋)
    mavros_msgs::SetMode set_mode[N];// 모드변경(서비스 인풋)
    mavros_msgs::WaypointPush wp[N]; //wp 푸시하기위한 변수 (서비스 인풋)
    mavros_msgs::Waypoint msg[N]; //Waypoint. 
    double L = 100.0; //시작지점에서 충돌지점까지거리(로컬)
    double Z = 20.0; // 충돌높이
    double th[N],type[N]; //to set direction.
    th[0] = M_PI;
    th[1] = 3.0 / 2.0 * M_PI;
    type[0] = 1;
    type[1] = 1;
    if (N>=3)
    {
        th[2] = 5.0/4.0*M_PI;
        type[2] = 1;
    }
    double dx, dy;//충돌지점과 시작지접사이 거리(로컬)

    double x_s[N], y_s[N], z_s[N]; //시작점 in global
    double x_wp[N], y_wp[N], z_wp[N]; //wp in global
    double x_col, y_col, z_col;//충돌지점 in global

    x_col = x00 + L * ltogx;
    y_col = y00 + 0.0;
    z_col = z00 + 6.0;

    int i;
    for (i = 0; i < N; i++)
    {
        dx = L * cos(th[i]) * ltogx;
        dy = L * sin(th[i]) * ltogy;

        x_s[i] = x_col + dx;
        y_s[i] = y_col + dy;
        z_s[i] = z_col;

        x_wp[i] = x_col - dx;
        y_wp[i] = y_col - dy;
        z_wp[i] = z_col;

        printf("r_s of %d = (%.3f,%.3f) \n", i, (x_s[i]-x00)/ltogx,(y_s[i]-y00)/ltogy);
    }
    for (i = 0; i < N; i++)
    {
        while (ros::ok() && !state[i].connected)
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
    ros::Time last_request[N];
    for (i = 0; i < N; i++)
        last_request[i] = ros::Time::now();

    for (i = 0; i < N && ros::ok(); i++)
    {

        wp[i].request.waypoints.clear();
        if (type[i]==1)
        {
        msg[i].frame = 3; //TAKEOFF
        msg[i].command = 22;
        msg[i].is_current = 1;
        msg[i].autocontinue = 1;
        msg[i].param1 = 0.0;
        msg[i].param2 = 0.0;
        msg[i].param3 = 0.0;
        msg[i].x_lat = y_s[i];
        msg[i].y_long = x_s[i];
        msg[i].z_alt = z_s[i];

        wp[i].request.waypoints.push_back(msg[i]);

        msg[i].frame = 3; //WP
        msg[i].command = 16;
        msg[i].is_current = 0;
        msg[i].autocontinue = 1;
        msg[i].param1 = 0.0;
        msg[i].param2 = 0.0;
        msg[i].param3 = 0.0;
        msg[i].x_lat = y_wp[i];
        msg[i].y_long = x_wp[i];
        msg[i].z_alt = z_wp[i];

        wp[i].request.waypoints.push_back(msg[i]);
        
            msg[i].frame=6; //LAND
            msg[i].command=21;
            msg[i].is_current=0;
            msg[i].autocontinue=1;
            msg[i].param1=0.0;
            msg[i].param2=0.0;
            msg[i].param3=0.0;
            msg[i].x_lat=y_wp[i];
            msg[i].y_long=x_wp[i];
            msg[i].z_alt=0.0;

        wp[i].request.waypoints.push_back(msg[i]);
        }
        else if (type[i]==2)
        {

        }
        
        wp[i].request.start_index = 0;
        if (!wp[i].response.success)
        {
            if (wp_client[i].call(wp[i]) && wp[i].response.success)
            {
                ROS_INFO("Vehicle[%d] wp updated.\n", i);
                last_request[i] = ros::Time::now();
            }
        }

        //wp_client[i].call(wp[i]);
        //if (wp[i].response.success)
        //    ROS_INFO("wp of uav[%d] updated\n",i);
    }
    printf("loop start\n");
    printf("----------------------------------------------\n");
    while (ros::ok())
    {

        ros::spinOnce();
        for (i = 0; i < N; i++)
        {
            arming[i].request.value = true;
            set_mode[i].request.custom_mode = "AUTO.MISSION";

            bool cond = ((ros::Time::now() - last_request[i]) > ros::Duration(1.0));
            if (!state[i].armed && cond)
            {
                if (arming_client[i].call(arming[i]) && arming[i].response.success)
                {
                    ROS_INFO("Vehicle[%d] armed\n", i);
                    last_request[i] = ros::Time::now();
                }
            }
            if (state[i].mode != "AUTO.MISSION" && state[i].mode != "OFFBOARD" && cond)
            {
                if (set_mode_client[i].call(set_mode[i]) && set_mode[i].response.mode_sent)
                {
                    ROS_INFO("Vehicle[%d] auto mode enabled.\n", i);
                    last_request[i] = ros::Time::now();
                }
            } /*
            if (!wp[i].response.success && cond){
                if (wp_client[i].call(wp[i]) && wp[i].response.success)
                {
                    ROS_INFO("Vehicle[%d] wp updated.\n",i);
                    last_request[i]=ros::Time::now();
                }
            }*/
        }

        rate.sleep();
    }
    return 0;
}
