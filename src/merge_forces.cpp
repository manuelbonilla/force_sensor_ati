#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>

#include <unistd.h>
#include <string>

std_msgs::Float64MultiArray leftforces, rightforces;
std::vector<std_msgs::Float64MultiArray> vec_allforces;

void callbackforce(const std_msgs::Float64MultiArray::ConstPtr& msg, int i)
{
    vec_allforces[i].data = msg->data;
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

    ros::Publisher pub_allforces = nh.advertise<std_msgs::Float64MultiArray>( "allforces", 1 );

    XmlRpc::XmlRpcValue my_list;
    nh.getParam("topics", my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    std::vector<ros::Subscriber> list_subs;
    for ( int i = 0; i < my_list.size(); ++i)
    {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        ROS_INFO_STREAM("Mering topic: " <<  static_cast<std::string>(my_list[i]).c_str());
        std_msgs::Float64MultiArray new_forces;
        vec_allforces.push_back( new_forces );
        list_subs.push_back( nh.subscribe<std_msgs::Float64MultiArray>( static_cast<std::string>(my_list[i]).c_str(), 1, boost::bind(&callbackforce, _1, i) ) );

    }

    while ( nh.ok() )
    {
        std_msgs::Float64MultiArray allforces;
        if (vec_allforces.size() > 0)
        {
            allforces.data = vec_allforces[0].data;
            for (unsigned int i = 1; i < vec_allforces.size(); ++i)
            {
                allforces.data.insert(allforces.data.end(), vec_allforces[i].data.begin(), vec_allforces[i].data.end());
            }
        }

        pub_allforces.publish(allforces);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}