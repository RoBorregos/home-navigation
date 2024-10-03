
// Setting goal: Frame:map, Position(-2.789, -0.124, 0.000), Orientation(0.000, 0.000, 0.774, 0.633) = Angle: 1.772
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

#define RAD2DEG(x) ((x)*180./M_PI)

ros::Publisher scan_fixed;
std::vector<double> ignore_array;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan new_scan;
    new_scan = *scan;
    int count = scan->scan_time / scan->time_increment;
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if(ignore_array.size() != 0){
	            for(uint16_t j = 0; j < ignore_array.size();j = j+2){
                    if((ignore_array[j] < degree) && (degree <= ignore_array[j+1])){
		                new_scan.ranges[i] = 0.0;
		                break;
		            }

	            }
	        }
       

    }
    scan_fixed.publish(new_scan);
}
std::vector<double> split(const std::string &s, char delim) {
    std::vector<double> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}

int main(int argc, char **argv)
{
    std::string list;
    ros::init(argc, argv, "ignore_laser");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_input", 1000, scanCallback);
    scan_fixed = nh.advertise<sensor_msgs::LaserScan>("scan_fixed", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("ignore_array",list,"");
    ignore_array = split(list ,',');

    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            ROS_ERROR_STREAM("ignore array should be between -180 and 180");
        }
    }

    ros::spin();

    return 0;
}
