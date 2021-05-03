// ==========================//
//   Get Aruco Marker Cloud
// ==========================//
// written by Shang-Wen, Wong.
// 2021.3.30

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //"pcl::fromROSMsg"

//realsense_msgs
//"~/realsense_ros/devel/include/realsense2_camera/Extrinsics.h"
#include <realsense2_camera/Extrinsics.h> 

// pcl
#include <pcl/io/pcd_io.h>

// C++
#include <vector>
#include <iostream>

using namespace std;

typedef pcl::PointXYZRGB PointTRGB;
typedef boost::shared_ptr<pcl::PointCloud<PointTRGB>> PointCloudTRGBPtr;

struct Center2D
{
    int x;
    int y;
};

struct Center3D
{
    float x;
    float y;
    float z;
};

struct ArucoMarker
{
    int id;
    std::vector<float> corner_pixel; //x1, y1, ..., x4, y4
    Center2D center_pixel;
    Center3D center_point;
    PointCloudTRGBPtr marker_cloud;
    // Eigen::Matrix4f marker_pose; #from aruco rvec,tvec/pointcloud normal
};

std::vector<ArucoMarker> marker_all{};

bool save_organized_cloud = false;

std::string file_path_cloud_organized = "organized_cloud_tmp.pcd";

pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);


void aruco_corners_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    cout << "aruco_corners_cb" << endl;
    // ROS_INFO("I heard: [%f],[%f],[%f],[%f]", corners_msg->data.at(0),corners_msg->data.at(1),corners_msg->data.at(2),corners_msg->data.at(3));
    // std::vector<float> corners = corners_msg->data;

    // int total_marker_num = corners.size() / 8; 
    // cout << "total_marker_num" << endl;
    // marker_all.resize(total_marker_num);

    // for(int k = 0; k < total_marker_num; ++k)
    // {
    //     int x1 = corners[8*k];
    //     int y1 = corners[8*k+1];
    //     int x3 = corners[8*k+4];       
    //     int y3 = corners[8*k+5];

    //     // marker_all[k].id = corners_msg->markers[k].ID;
        
    //     marker_all[k].corner_pixel.assign(corners.begin()+8*k, corners.begin()+8*k+7);
    //     marker_all[k].center_pixel.x = int((x1 +x3)/2.0);
    //     marker_all[k].center_pixel.y = int((y1 +y3)/2.0);
    // }

    // if(corners_msg->data.empty())
    //     total_marker_num = 0;
 
    // cout << "Total ArUco Markers = " << total_marker_num << endl;   //ERROR: display 1 even if no obj detected
}


void organized_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
{
    //==================================================//
    // 有序點雲 Organized Point Cloud; Depth Point Cloud
    // Subscribe "/camera/depth_registered/points" topic
    //==================================================//
    cout << "organized_cloud_cb" << endl;

    int height = organized_cloud_msg->height;
    int width = organized_cloud_msg->width;
    int points = height * width;

    if((points!=0) && (save_organized_cloud ==true))
    {
        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        organized_cloud_ori->clear();
        pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);

        cout << "organized_cloud_ori saved: " << file_path_cloud_organized << "; (width, height) = " << width << ", " << height << endl;
        pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_organized, *organized_cloud_ori); //savePCDFileASCII
        cout << "organized_cloud_ori saved: DONE! \n";
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_cloud");
    cout<<"inside\n";
    ros::NodeHandle nh;
    
    
    //有序點雲 Organized Point Cloud 
    // ros::Subscriber sub_organized_cloud = nh.subscribe("/camera/depth_registered/points", 1, organized_cloud_cb);
    
    //ArucoMarkers的角點 Corners of Aruco Markers
    ros::Subscriber sub_aruco_corners = nh.subscribe<std_msgs::Float64MultiArray>("/aruco_corners", 1, aruco_corners_cb);
    cout<<"try\n";
    ros::spin();

    return 0;
}
