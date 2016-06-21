#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <fstream>


std::vector<std_msgs::Float64MultiArray> vec_allforces;
std_msgs::Float64MultiArray matlab_data;

int matlab_enabled;

void callbackforce(const std_msgs::Float64MultiArray::ConstPtr& msg, int i)
{
    vec_allforces[i].data = msg->data;
}

void callbackTimeMatlab(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    matlab_data.data = msg->data;
    matlab_enabled = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "merge_forces");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("[MergeForces] Node is ready");

    double spin_rate = 1000;
    ros::param::get("~spin_rate", spin_rate);
    ROS_INFO_STREAM( "Spin Rate: " << spin_rate);

    ros::Rate rate(spin_rate);

    ros::Subscriber sub_matlab_data = nh.subscribe<std_msgs::Float64MultiArray>( "/matlab_data", 1, callbackTimeMatlab);

    XmlRpc::XmlRpcValue my_list;
    nh.getParam("topics", my_list);

    std::string file_results;
    nh.param<std::string>("file_results", file_results, "results.txt");
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO_STREAM ("Writing of file: " << file_results.c_str());

    matlab_enabled = 0;
    std::vector<ros::Subscriber> list_subs;
    for ( int i = 0; i < my_list.size(); ++i)
    {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        ROS_INFO_STREAM("Merging topic: " <<  static_cast<std::string>(my_list[i]).c_str());
        std_msgs::Float64MultiArray new_forces;
        vec_allforces.push_back( new_forces );
        list_subs.push_back( nh.subscribe<std_msgs::Float64MultiArray>( static_cast<std::string>(my_list[i]).c_str(), 1, boost::bind(&callbackforce, _1, i) ) );
    }
    ros::Time begin = ros::Time::now();
    ros::Duration elapsed_time;



    std::ofstream myfile;
    myfile.open(file_results.c_str());

    if (vec_allforces.size() > 0)
    {
        std::string on_topic;
        nh.param<std::string>("on_topic", on_topic, "allforces");
        ros::Publisher pub_allforces = nh.advertise<std_msgs::Float64MultiArray>( "allforces", 100 );

        while ( nh.ok() )
        {
            std_msgs::Float64MultiArray allforces;
            elapsed_time = ros::Time::now() - begin;
            allforces.data = matlab_data.data;
            allforces.data.push_back(elapsed_time.toSec());
            // allforces.data.push_back(time_matlab.data);
            for (unsigned int i = 0; i < vec_allforces.size(); ++i)
            {
                allforces.data.insert(allforces.data.end(), vec_allforces[i].data.begin(), vec_allforces[i].data.end());
            }
            
            if (allforces.data.size() > 1 && matlab_enabled == 1)
            {
                for (unsigned int i = 0; i < allforces.data.size(); i++)
                {
                    myfile << allforces.data[i] << "\t";
                }
                myfile << std::endl;
            }

            pub_allforces.publish(allforces);
            ros::spinOnce();
            rate.sleep();
        }
    }

    myfile.close();

    return 0;
}