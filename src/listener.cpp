#include <ros/ros.h>
#include <irobotcreate2/RoombaIR.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <time.h>
#include <string>
#include <cmath>

int irdata[150] = {0}; 
double intensity[150];
double ranges[150];
const int sub_laser = 2; 
const double laser_freq = 10; 
const unsigned int num_readings = 6; 
const char *frames[6] = {"base_irbumper_left", 
                      "base_irbumper_front_left", 
                      "base_irbumper_center_left", 
                      "base_irbumper_center_right", 
                      "base_irbumper_front_right", 
                      "base_irbumper_right"}; 

void sensorListenerCallback
     (const irobotcreate2::RoombaIR::ConstPtr& ir_data)
{
    // ROS_INFO("Header.frame_id = %s", ir_data->header.frame_id.c_str()); 
    // ROS_INFO("Signal = %d", ir_data->signal); 

    std::string frm_id = ir_data->header.frame_id; 
    for (int i = 0; i < 6; i++)
        if (frm_id == frames[i])
        {
            for (int j = sub_laser * (i); j < sub_laser * (i+1); j++)
                irdata[j] = ir_data->signal; 
            break; 
        }
}

double calc_range(double intensity){
    const double a = -0.0374444;
    const double b = 0.2836288;
    if (intensity < 1e-8)
        intensity = 1e-8; 
    double ans = a * log(intensity) + b;
    if (ans <= 0.2) return 0.2 + ans; 
    else return 0.4 + (ans - 0.2) * 3; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_listener"); 
    ros::NodeHandle n; 
    ros::Subscriber sub = n.subscribe("ir_bumper", 1000, sensorListenerCallback);
    ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("base_scan", 50);
    ros::Rate r(10); 
    int count = 0; 
    ROS_INFO("Sensor broadcasting..."); 

    while (ros::ok())
    {
        ros::spinOnce(); 
        
        for (int i = 0; i < num_readings * sub_laser; i++)
        {
            intensity[i] = irdata[i]; 
            ranges[i] = calc_range(intensity[i]); 
            // ROS_INFO("i = %d, range = %.3f, intens = %.3f", i, ranges[i], intensity[i]);  
        }
    
        ros::Time scan_time = ros::Time::now();

        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "base_laser_link";
        scan.angle_min = 3.1416 / 2;
        scan.angle_max = 3 * 3.1416 / 2;
        scan.angle_increment = -3.1416 / 6 / sub_laser;
        scan.time_increment = (1 / laser_freq) / (num_readings * sub_laser);
        scan.range_min = 0.0;
        scan.range_max = 0.4;
        scan.scan_time = scan_time.sec + scan_time.nsec; 

        scan.ranges.resize(num_readings * sub_laser);
        scan.intensities.resize(num_readings * sub_laser);

        for (int tot = 0, i = 0; i < num_readings; i++)
        {
            for (int j = 0; j < sub_laser; j++)
            {
                scan.ranges[tot] = ranges[tot];
                scan.intensities[tot] = intensity[tot];
                // ROS_INFO("TOT = %d, range = %.3f, intens = %.3f", tot, ranges[tot], intensity[tot]);  
                tot++;
            }
        }

        pub.publish(scan); 
        ++count; 
        r.sleep(); 
    }
    return 0; 
}

