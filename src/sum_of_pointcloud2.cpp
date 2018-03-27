#include <vector>
#include <string>
#include <fstream>
#include <iterator>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#ifdef __APPLE__
    #include "OpenCL/opencl.h"
#else
    #include "CL/cl.h"
#endif

using namespace std;

cl_kernel kernel;
cl_context context;
cl_uint deviceIdCount;
vector<cl_device_id> deviceIds;
ros::Publisher pub;

string getPlatformName (cl_platform_id id){
    size_t size = 0;
    clGetPlatformInfo (id, CL_PLATFORM_NAME, 0, NULL, &size);

    string result;
    result.resize (size);
    clGetPlatformInfo (id, CL_PLATFORM_NAME, size, const_cast<char*> (result.data ()), NULL);

    return result;
}

string getDeviceName (cl_device_id id){
    size_t size = 0;
    clGetDeviceInfo (id, CL_DEVICE_NAME, 0, NULL, &size);

    string result;
    result.resize (size);
    clGetDeviceInfo (id, CL_DEVICE_NAME, size, const_cast<char*> (result.data ()), NULL);

    return result;
}

void checkError (cl_int error){
    if (error != CL_SUCCESS) {
        ROS_ERROR("OpenCL call failed with error: %d", error);
        exit (1);
    }
}

string LoadKernel (const char* name){
    ifstream in (name);
    string result((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());
    return result;
}

cl_program createProgram (const string& source, cl_context context){
    size_t lengths [1] = { source.size () };
    const char* sources [1] = { source.data () };

    cl_int error = 0;
    cl_program program_ = clCreateProgramWithSource(context, 1, sources, lengths, &error);
    checkError (error);

    return program_;
}

void cloudCallback (const sensor_msgs::PointCloud2& msg){
    cl_int sz = msg.data.size();
    cl_uint8 *in = (cl_uint8 *) malloc(sizeof(cl_uint8)*sz);
    cl_int error = 0;

    cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(cl_uint8) * sz, NULL, &error);
    checkError(error);

    clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);

    cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

    clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sizeof(cl_uint8) * sz, in, 0, NULL, NULL);
    checkError (error);

    size_t size = sz;

    cl_event gpuExec;

    checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

    clWaitForEvents(1, &gpuExec);

    uint8_t *result = (uint8_t *) malloc(sizeof(uint8_t) * sz);
    checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sizeof(uint8_t) * sz, result, 0, NULL, NULL));

    clReleaseCommandQueue (queue);
    clReleaseMemObject(buffer);

    sensor_msgs::PointCloud2 res = sensor_msgs::PointCloud2(msg);
    res.data.insert(res.data.end(), &result[0], &result[sz]);
    pub.publish(res);
    free(result);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "rosgpu_tests_sum_of_pointcloud2");
    ros::NodeHandle nh;
    string kernel_filename;
    string cloud_topic;
    string result_topic;

    nh.param("rosgpu_tests_sum_of_pointcloud2/kernel_filename", kernel_filename, string("sop2.cl"));
    nh.param("rosgpu_tests_sum_of_pointcloud2/cloud_topic", cloud_topic, string("/zed/point_cloud/cloud_registered"));
    nh.param("rosgpu_tests_sum_of_pointcloud2/result_topic", result_topic, string("rosgpu_tests_sum_of_pointcloud2/result"));

    string full_kernel_path = ros::package::getPath("rosgpu_tests") + "/kernels/" + kernel_filename;

    cl_uint platformIdCount = 0;
    clGetPlatformIDs (0, NULL, &platformIdCount);

    if (platformIdCount == 0) {
        ROS_ERROR("No OpenCL platform found");
        return 1;
    }
    else {
        ROS_INFO("Found %d platform(s)", platformIdCount);
    }

    vector<cl_platform_id> platformIds (platformIdCount);
    clGetPlatformIDs (platformIdCount, platformIds.data (), NULL);

    for (cl_uint i = 0; i < platformIdCount; ++i) {
        ROS_INFO("\t (%d) : %s", i+1, getPlatformName (platformIds [i]).c_str());
    }

    deviceIdCount = 0;
    clGetDeviceIDs (platformIds [0], CL_DEVICE_TYPE_ALL, 0, NULL, &deviceIdCount);

    if (deviceIdCount == 0) {
        ROS_ERROR("No OpenCL devices found");
        return 1;
    }
    else {
        ROS_INFO("Found %d device(s)", deviceIdCount);
    }

    deviceIds  = vector<cl_device_id>(deviceIdCount);
    clGetDeviceIDs (platformIds [0], CL_DEVICE_TYPE_ALL, deviceIdCount, deviceIds.data(), NULL);

    for (cl_uint i = 0; i < deviceIdCount; ++i) {
        ROS_INFO("\t (%d) : %s", i+1, getDeviceName (deviceIds [i]).c_str());
    }

    const cl_context_properties contextProperties [] = {CL_CONTEXT_PLATFORM, reinterpret_cast<cl_context_properties> (platformIds [0]), 0, 0};

    cl_int error = CL_SUCCESS;
    context = clCreateContext (contextProperties, deviceIdCount, deviceIds.data (), NULL, NULL, &error);
    checkError (error);

    ROS_INFO("Context created");

    cl_program program = createProgram (LoadKernel (full_kernel_path.c_str()), context);

    checkError (clBuildProgram (program, deviceIdCount, deviceIds.data (), "-D FILTER_SIZE=1", NULL, NULL));

    ROS_INFO("Program built");

    kernel = clCreateKernel (program, "sumPointCloud", &error);
    checkError (error);

    ROS_INFO("Kernel created");

    pub = nh.advertise<sensor_msgs::PointCloud2>(result_topic, 1);
    ros::Subscriber s = nh.subscribe (cloud_topic, 1, cloudCallback);

    while(ros::ok()){
        ros::spin();
    }

    clReleaseKernel (kernel);
    clReleaseProgram (program);

    clReleaseContext (context);
}
