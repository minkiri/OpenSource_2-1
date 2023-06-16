#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

bool front_cargo = false;
bool back_cargo = false;
bool platoon_go = false;
std_msgs::Int32 color_index;

void frontCargoCallback(const std_msgs::Bool::ConstPtr& msg)
{
    front_cargo = msg->data;
}

void backCargoCallback(const std_msgs::Bool::ConstPtr& msg)
{
    back_cargo = msg->data;
    //std::cout<<back_cargo<<std::endl;
}

void colorIndexCallback(const std_msgs::Int32::ConstPtr& msg)
{
    color_index = *msg;
}

void checkState(ros::Publisher& platoon_go_pub)
{

    if (color_index.data == 2 && front_cargo == false && back_cargo == false) {
        platoon_go = false;
    } else if (color_index.data == 2 && front_cargo == true && back_cargo == true) {
        platoon_go = true;
    } else if (color_index.data == 3 && front_cargo == true && back_cargo == true) {
        platoon_go = false;
    } else if (color_index.data == 3 && front_cargo == false && back_cargo == false) {
        platoon_go = true;
    }else if(color_index.data == 0)
    {
    platoon_go = true;
    }
    std::cout<<"platoon_go : " <<platoon_go<<std::endl;
    std_msgs::Bool platoon_data;
    platoon_data.data = platoon_go;
    platoon_go_pub.publish(platoon_data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "decision");
    ros::NodeHandle nh;

    // Subscribe
    ros::Subscriber front_cargo_sub = nh.subscribe("/cargo", 10, frontCargoCallback);
    ros::Subscriber back_cargo_sub = nh.subscribe("/back_cargo", 10, backCargoCallback);
    ros::Subscriber color_index_sub = nh.subscribe("/color_image", 10, colorIndexCallback);

    // Publish
    ros::Publisher platoon_go_pub = nh.advertise<std_msgs::Bool>("/platoon_go", 10);

    while (ros::ok()) {
        checkState(platoon_go_pub);
        ros::spinOnce();
    }

    return 0;
}
