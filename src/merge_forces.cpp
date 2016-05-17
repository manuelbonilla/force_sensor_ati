#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>

#include <unistd.h>
#include <string>

std_msgs::Float64MultiArray leftforces, rightforces;

void callbackleft(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    leftforces.data = msg->data;
}

void callbackright(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    rightforces.data = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "merge_forces");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("[MergeForces] Node is ready");

    double spin_rate = 1000;
    ros::param::get("~spin_rate", spin_rate);
    ROS_INFO( "Spin Rate %lf", spin_rate);

    ros::Rate rate(spin_rate);

    XmlRpc::XmlRpcValue my_list;
    nh.getParam("topics", my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    ros::Subscriber sub_leftforces = nh.subscribe("/left/allforces", 1, callbackleft);
    ros::Subscriber sub_rightforces = nh.subscribe("/right/allforces", 1, callbackright);

    ros::Publisher pub_allforces = nh.advertise<std_msgs::Float64MultiArray>( "allforces", 1 );

    while ( nh.ok() )
    {
        std_msgs::Float64MultiArray allforces;
        allforces.data = leftforces.data;
        allforces.data.insert(allforces.data.end(), rightforces.data.begin(), rightforces.data.end());
        pub_allforces.publish(allforces);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}