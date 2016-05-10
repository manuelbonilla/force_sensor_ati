#include <force_sensor_ati/omd/opto.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <string.h>

#include <ros/ros.h>
#include <ros/console.h>

using namespace std;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "force_sensor_ati");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("[ForceFensorATI] Node is ready");

    double spin_rate = 100;
    ros::param::get("~spin_rate", spin_rate);
    ROS_DEBUG( "Spin Rate %lf", spin_rate);

    ros::Rate rate(spin_rate);
    OptoDAQ daq;
    OptoPorts ports;

    OPort* portlist = ports.listPorts(true);

    //std::cout << "Test!" << std::endl;

    if (ports.getLastSize() == 0)
    {
        std::cout << "No sensor available!" << std::endl;
        return 0;
    }


    daq.open(portlist[0]);
    daq.zeroAll();

    OptoPackage* pa = 0;

    int size = daq.readAll(pa, false);

    while (nh.ok())
    {

        std::cout << "test" << std::endl;
        std::cout << "daq.getSensorSize(): " << daq.getSensorSize() << std::endl;
        int size = daq.readAll(pa, false);

        ROS_INFO_STREAM("***** Size: " << size);

        for (int k = 0; k < daq.getSensorSize(); k++)
        {
            std::cout << "Sensor " << k << " datas:" << endl;;
            std::cout << "(Size: " << size << ")" << endl;;
            for (int i = 0; i < size; i++)
            {
                std::cout << "x: " << pa[k * size + i].x << " y: " << pa[k * size + i].y << " z: " << pa[k * size + i].z << " s1: " << pa[k * size + i].s1 << " s2: " << pa[k * size + i].s2 << " s3: " << pa[k * size + i].s3 << " s4: " << pa[k * size + i].s4 << " TEMP: " << pa[k * size + i].temp << std::endl;
            }
        }

        ros::spinOnce();
        rate.sleep();

    }



    daq.close();


    return 0;

}
