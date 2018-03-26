#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <string>


using namespace std;
ros::Publisher pub;

void cloudCallback (const sensor_msgs::PointCloud2& msg){
    float result = 1;
    for(unsigned i = 0; i < msg.data.size(); i++){
        result += msg.data[i] * 0.3f;
    }
    std_msgs::Float32 pmsg;
    pmsg.data = result;
    pub.publish(pmsg);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "rosgpu_tests_sum_of_pointcloud2_cpu");
    ros::NodeHandle nh;
    string cloud_topic;
    string result_topic;

    nh.param("rosgpu_tests_sum_of_pointcloud2_cpu/cloud_topic", cloud_topic, string("/zed/point_cloud/cloud_registered"));
    nh.param("rosgpu_tests_sum_of_pointcloud2_cpu/result_topic", result_topic, string("rosgpu_tests_sum_of_pointcloud2_cpu/result"));

    pub = nh.advertise<std_msgs::Float32>(result_topic, 1);
    ros::Subscriber s = nh.subscribe (cloud_topic, 1, cloudCallback);

    while(ros::ok()){
        ros::spin();
    }
}
