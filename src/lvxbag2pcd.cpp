#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

#include "CustomMsg.h"
#include "common.h"

#include <boost/filesystem.hpp>
using namespace std;

struct pointData{
    float x;
    float y;
    float z;
    int i;
};
vector<pointData> vector_data;
livox_ros_driver::CustomMsg livox_cloud;
string input_bag_path, output_path;
int threshold_lidar, data_num;

bool getBagFilesAndTransfer(std::string inputPath);

void loadAndSavePointcloud(int index);
void loadAndSavePointcloud(std::string bagFile);

void writeTitle(const string filename, unsigned long point_num);
void writePointCloud(const string filename, const vector<pointData> singlePCD);

void dataSave(std::string fileName);
void dataSave(int index);


bool getBagFilesAndTransfer(std::string inputPath)
{
    if (!boost::filesystem::exists(inputPath)){
        std::cerr<<"###### ERROR: Cannot find input directory " << inputPath << "." << std::endl;
        return false;
    }
    else if (boost::filesystem::is_directory(inputPath)){//path
        std::cout<<"inputPath is a directory.\n";
        std::vector<std::string> bagFiles;

        boost::filesystem::recursive_directory_iterator it(inputPath);
        boost::filesystem::recursive_directory_iterator endit;
        while(it !=endit) {
            if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ".bag") {
                bagFiles.push_back(it->path().string());
            }
            ++it;
        }
        if(bagFiles.size() > 0 )
            for (size_t i=0;i<bagFiles.size();++i)
                loadAndSavePointcloud(bagFiles[i]);
    }
    else if(boost::filesystem::extension(inputPath) == ".bag")//.bag file
    {
        std::cout<<"inputPath is a '.bag' file.\n";
        loadAndSavePointcloud(inputPath);
    }
    return true;
}

void loadAndSavePointcloud(std::string bagFile)
{
    fstream file_;
    file_.open(bagFile, ios::in);
    if (!file_) {
        cout << "File " << bagFile << " does not exit" << endl;
        return;
    }
    ROS_INFO("Start to load the rosbag %s", bagFile.c_str());
    rosbag::Bag bag;
    try {
        bag.open(bagFile, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg"));
    rosbag::View view(bag, rosbag::TypeQuery(types));

    int cloudCount = 0;
    for (const rosbag::MessageInstance& m : view) {
        livox_cloud = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type

        for(uint i = 0; i < livox_cloud.point_num; ++i) {
            pointData myPoint;
            myPoint.x = livox_cloud.points[i].x;
            myPoint.y = livox_cloud.points[i].y;
            myPoint.z = livox_cloud.points[i].z;
            myPoint.i = livox_cloud.points[i].reflectivity;

            vector_data.push_back(myPoint);
        }
        ++cloudCount;
        if (cloudCount >= threshold_lidar) {
            break;
        }
    }

    dataSave(boost::filesystem::path(bagFile).stem().string());
    vector_data.clear();
}

//livox_camera_lidar_calibration
void loadAndSavePointcloud(int index) {
    string path = input_bag_path + int2str(index) + ".bag";
    fstream file_;
    file_.open(path, ios::in);
    if (!file_) {
        cout << "File " << path << " does not exit" << endl;
        return;
    }
    ROS_INFO("Start to load the rosbag %s", path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg")); 
    rosbag::View view(bag, rosbag::TypeQuery(types));

    int cloudCount = 0;
    for (const rosbag::MessageInstance& m : view) {
        livox_cloud = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type

        for(uint i = 0; i < livox_cloud.point_num; ++i) {
            pointData myPoint;
            myPoint.x = livox_cloud.points[i].x;
            myPoint.y = livox_cloud.points[i].y;
            myPoint.z = livox_cloud.points[i].z;
            myPoint.i = livox_cloud.points[i].reflectivity;

            vector_data.push_back(myPoint);
        }
        ++cloudCount;
        if (cloudCount >= threshold_lidar) {
            break;
        }
    }
    dataSave(index);
    vector_data.clear();
}

void writeTitle(const string filename, unsigned long point_num) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
        outfile << "VERSION .7" << endl;
        outfile << "FIELDS x y z intensity" << endl;
        outfile << "SIZE 4 4 4 4" << endl;
        outfile << "TYPE F F F F" << endl;
        outfile << "COUNT 1 1 1 1" << endl;
        outfile << "WIDTH " << long2str(point_num) << endl;
        outfile << "HEIGHT 1" << endl;
        outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
        outfile << "POINTS " << long2str(point_num) << endl;
        outfile << "DATA ascii" << endl;
    }
    ROS_INFO("Save file %s", filename.c_str());
}

void writePointCloud(const string filename, const vector<pointData> singlePCD) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        for (unsigned long i = 0; i < singlePCD.size(); ++i) {
            outfile << float2str(singlePCD[i].x) << " " << float2str(singlePCD[i].y) << " " << float2str(singlePCD[i].z) << " " << int2str(singlePCD[i].i) << endl;
        }
    }
}

//livox_camera_lidar_calibration
void dataSave(int index) {
    string outputName = output_path + int2str(index) + ".pcd";
    writeTitle(outputName, vector_data.size());
    writePointCloud(outputName, vector_data);
}

void dataSave(std::string fileName){
    string outputName = output_path + fileName + ".pcd";
    writeTitle(outputName, vector_data.size());
    writePointCloud(outputName, vector_data);
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    else {
        cout <<"input_bag_path: " << input_bag_path << endl;
    }
    if (!ros::param::get("output_pcd_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }

    if(output_path == "")
    {
        if(boost::filesystem::is_directory(input_bag_path))
            output_path=input_bag_path + "pcdFile/";
        else
            output_path=boost::filesystem::path(input_bag_path).parent_path().string() +"pcdFiles/";
        std::cout<<"output_path: "<<output_path<<std::endl;

        if(!boost::filesystem::exists(output_path))
            boost::filesystem::create_directory(output_path);
    }

    if (!ros::param::get("threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
    if (!ros::param::get("data_num", data_num)) {
        cout << "Can not get the value of data_num" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcdTransfer");
    getParameters();

    getBagFilesAndTransfer(input_bag_path);

//    for (int i = 0; i < data_num; ++i) {
//        loadAndSavePointcloud(i);
//    }
    ROS_INFO("Finish all!");
    return 0;
}

