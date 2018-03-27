#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
unsigned cpu_counter = 0;
unsigned gpu_counter = 0;

void cloudCPUCallback1 (const sensor_msgs::PointCloud2& msg){
    cpu_counter++;
    ROS_INFO("\n\033[1;32mCPU messages: %d\033[0m\n\033[1;34mGPU messages: %d\033[0m\n\033[1;35mGPU/CPU ration: %f\033[0m\n", cpu_counter, gpu_counter, (float)gpu_counter/cpu_counter);
}

void cloudGPUCallback2 (const sensor_msgs::PointCloud2& msg){
    gpu_counter++;
}

int main (int argc, char** argv){
    ros::init (argc, argv, "rosgpu_tests_sum_of_pointcloud2_comparison");
    ros::NodeHandle nh;
    string result_topic1;
    string result_topic2;

    nh.param("rosgpu_tests_sum_of_pointcloud2_comparison/cpu_topic", result_topic1, string("rosgpu_tests_sum_of_pointcloud2_cpu/result"));
    nh.param("rosgpu_tests_sum_of_pointcloud2_comparison/gpu_topic2", result_topic2, string("rosgpu_tests_sum_of_pointcloud2/result"));

    ros::Subscriber s1 = nh.subscribe (result_topic1, 1, cloudCPUCallback1);
    ros::Subscriber s2 = nh.subscribe (result_topic2, 1, cloudGPUCallback2);

    while(ros::ok()){
        ros::spin();
    }
}
