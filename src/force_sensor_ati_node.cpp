#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include "force_sensor_ati/force_sensor_ati.h"
#include <unistd.h>
#include <string>
#include <std_srvs/Empty.h>

OptoDAQ daq;

bool setZerosCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    daq.zeroAll();
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_sensor_ati");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("[ForceFensorATI] Node is ready");

    double spin_rate = 1000;
    ros::param::get("~spin_rate", spin_rate);
    ROS_DEBUG( "Spin Rate %lf", spin_rate);

    ros::Rate rate(spin_rate);

    std::vector<ros::Publisher> pubs_forces;

    int iSpeed = (int) spin_rate; // Speed in Hz
    int iFilter = 15;  // Filter in Hz
    int port_id = 0;

    std::string sensor_port, on_topic;
    nh.param<int>("port_id", port_id, 0);
    sensor_port = std::string("/dev/ttyACM") + std::to_string(port_id);
    nh.param<std::string>("on_topic", on_topic, "wrench");
    
    nh.param<int>("filter", iFilter, 15);

    OptoPorts ports;

    sleep(1.0);

    OPort* portlist = ports.listPorts(true);
    std::string port_names(portlist[0].name);


    ROS_INFO_STREAM("Opening sensor on port: " << sensor_port.c_str());
    ROS_INFO_STREAM("Publishing force measurements on topic: " << on_topic.c_str());

    if (daq.open(portlist[ port_id -1 ]))
    {
        ROS_INFO_STREAM("Sensor Initialized on port " << sensor_port.c_str());
    }
    else
    {
        ROS_ERROR_STREAM("No sensor available on port " << sensor_port.c_str() );
        return 0;
        ros::shutdown();
    }

    geometry_msgs::Wrench force_measurements;


    SensorConfig sensorConfig;
    sensorConfig.setSpeed(iSpeed);
    sensorConfig.setFilter(iFilter);

    bool bConfig = false;
    bConfig = daq.sendConfig(sensorConfig);
    sleep(1.0);
    if (bConfig == false) {
        ROS_ERROR_STREAM("Could not set config");
        daq.close();
        ros::shutdown();
        return 0;
    }
    ROS_INFO_STREAM("Sensor Initialized with Speed: " << iSpeed << " and Filter: " << iFilter);

    OptoPackage* pack3D = 0;
    int size = daq.readAll(pack3D, false);
    ROS_INFO_STREAM("Number of Sensor: " << daq.getSensorSize());

    ros::ServiceServer srv_setZeros = nh.advertiseService("setZeros", &setZerosCallback);

    int number_of_sensor = daq.getSensorSize();
    for (unsigned i = 0; i < number_of_sensor; ++i) {
        pubs_forces.push_back( nh.advertise<geometry_msgs::WrenchStamped>( on_topic.c_str() + std::to_string(i), 1 ));
    }

    ros::Publisher pub_allforces = nh.advertise<std_msgs::Float64MultiArray>( "allforces", 1 );

    while (nh.ok())
    {
        geometry_msgs::WrenchStamped force_readings;
        std_msgs::Float64MultiArray allforces;
        size = daq.readAll(pack3D, false);
        if (size ==  0)
            continue;

        for (int k = 0; k < number_of_sensor; k++)
        {
            for (int i = 0; i < size; i++)
            {
                force_readings.wrench.force.x  = pack3D[k * size + i].x;
                force_readings.wrench.force.y  = pack3D[k * size + i].y;
                force_readings.wrench.force.z  = pack3D[k * size + i].z;
                pubs_forces[k].publish(force_readings);
                allforces.data.push_back(pack3D[k * size + i].x);
                allforces.data.push_back(pack3D[k * size + i].y);
                allforces.data.push_back(pack3D[k * size + i].z);
            }
        }
        pub_allforces.publish(allforces);
        ros::spinOnce();
        rate.sleep();

    }

    sleep(1.0);
    daq.close();

    return 0;
}