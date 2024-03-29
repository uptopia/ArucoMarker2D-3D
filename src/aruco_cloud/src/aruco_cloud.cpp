// ==========================//
//   Get Aruco Marker Cloud
// ==========================//
// written by Shang-Wen, Wong.
// 2021.3.30

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

//tf
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //"pcl::fromROSMsg"

//realsense_msgs
//"~/realsense_ros/devel/include/realsense2_camera/Extrinsics.h"
#include <realsense2_camera/Extrinsics.h> 


#include <boost/make_shared.hpp>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include<pcl/kdtree/kdtree_flann.h>
// #include<pcl/kdtree/io.h>

// C++
#include <vector>
#include <iostream>
#include <algorithm>

#include <Eigen/Core>
// #include <pcl/common/transform.h>
// #include <pcl/transform.h>
#include <pcl/common/common.h>
// #include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf/tf.h>

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


bool save_organized_cloud = true;

std::string file_path_cloud_organized = "organized_cloud_tmp.pcd";

pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);

ros::Publisher top3_cloud_pub, target_sauce_center_pub;
sensor_msgs::PointCloud2 top3_clouds_msg;

// tf::TransformBroadcaster br;
// tf::Transform transform;

void get_3d_center(pcl::PointCloud<PointTRGB>::Ptr & cloud_ori, pcl::PointCloud<PointTRGB>::Ptr & cloud_obj, Center2D center_pixel, Center3D& center_point)
{
    //==========================================//
    // Get Center 3D Points
    // map 2D center_pixel to 3D center_point
    //==========================================//
    // int center_x = sauces_all[n].center_pixel.x;
    // int center_y = sauces_all[n].center_pixel.y;

    PointTRGB center_pt_3d = cloud_ori->at(center_pixel.x, center_pixel.y);
    cout << "\tCenter_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;

    // if Center_pt_3d is NAN, use all cluster's points
    if(!pcl_isfinite(center_pt_3d.x) || !pcl_isfinite(center_pt_3d.y) || !pcl_isfinite(center_pt_3d.z))
    {
        int total_points = cloud_obj->size();
        center_pt_3d.x = 0;
        center_pt_3d.y = 0;
        center_pt_3d.z = 0;

        for(int kk = 0; kk < total_points; ++kk)
        {
            PointTRGB pt = cloud_obj->points[kk];
            center_pt_3d.x += pt.x;
            center_pt_3d.y += pt.y;
            center_pt_3d.z += pt.z;
        }

        center_pt_3d.x /= total_points;
        center_pt_3d.y /= total_points;
        center_pt_3d.z /= total_points;

        cout << "\t**Center_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;
    }
    center_point.x = center_pt_3d.x;
    center_point.y = center_pt_3d.y;
    center_point.z = center_pt_3d.z;         
}


void aruco_corners_cb(const std_msgs::Float64MultiArray::ConstPtr& corners_msg)
{
    // cout << "aruco_corners_cb" << endl;
    // ROS_INFO("I heard: [%f],[%f],[%f],[%f]", corners_msg->data.at(0),corners_msg->data.at(1),corners_msg->data.at(2),corners_msg->data.at(3));
    
    int total_marker_num = 0;

    if(!corners_msg->data.empty())
    {
        total_marker_num = corners_msg->data.size() / 8; 
        marker_all.resize(total_marker_num);

        for(int k = 0; k < total_marker_num; ++k)
        {
            int x1 = corners_msg->data.at(8*k);
            int y1 = corners_msg->data.at(8*k+1);
            int x3 = corners_msg->data.at(8*k+4);       
            int y3 = corners_msg->data.at(8*k+5);

            // marker_all[k].id = corners_msg->markers[k].ID;
            
            marker_all[k].corner_pixel.assign(corners_msg->data.begin()+8*k, corners_msg->data.begin()+8*k+8);
            marker_all[k].center_pixel.x = int((x1 +x3)/2.0);
            marker_all[k].center_pixel.y = int((y1 +y3)/2.0);
        }
    }
    else
        total_marker_num = 0;
        marker_all.resize(total_marker_num);
 
    cout << "Total ArUco Markers Detected = " << total_marker_num << endl;   //ERROR: display 1 even if no obj detected
}


// void organized_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
// {
//     //==================================================//
//     // 有序點雲 Organized Point Cloud; Depth Point Cloud
//     // Subscribe "/camera/depth_registered/points" topic
//     //==================================================//
//     cout << "organized_cloud_cb" << endl;

//     int height = organized_cloud_msg->height;
//     int width = organized_cloud_msg->width;
//     int points = height * width;

//     if((points!=0) && (save_organized_cloud ==true))
//     {
//         // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
//         organized_cloud_ori->clear();
//         pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);

//         cout << "organized_cloud_ori saved: " << file_path_cloud_organized << "; (width, height) = " << width << ", " << height << endl;
//         pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_organized, *organized_cloud_ori); //savePCDFileASCII
//         cout << "organized_cloud_ori saved: DONE! \n";
//     }
// }
void CalculatePCA(pcl::PointCloud<PointTRGB>::Ptr & cloud, Eigen::Matrix3f eigenVectorsPCA)
{
    Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	// Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
 
	// std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	// std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	// std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
}

void aruco_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
{
    //==================================================//
    // 有序點雲 Organized Point Cloud; Depth Point Cloud
    // Subscribe "/camera/depth_registered/points" topic
    //==================================================//
    // cout << "organized_cloud_

    int height = organized_cloud_msg->height;
    int width = organized_cloud_msg->width;
    int points = height * width;

    // cout<<"(height, width) = "<<height<<", "<<width<<endl;
    if((points==0))// && (save_organized_cloud ==true))
    {
        cout<<"PointCloud No points!!!!!!\n";
        //break? pass?
    }
    else
    {
        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        organized_cloud_ori->clear();
        pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);

        // cout << "organized_cloud_ori saved: " << file_path_cloud_organized << "; (width, height) = " << organized_cloud_ori->width << ", " << organized_cloud_ori->height << endl;
        // pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_organized, *organized_cloud_ori); //savePCDFileASCII
        // cout << "organized_cloud_ori saved: DONE! \n";
    
        // pcl::PointCloud<PointTRGB>::Ptr top3_clouds(new pcl::PointCloud<PointTRGB>);            
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr top3_clouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);            

        for(int n = 0; n < marker_all.size(); ++n)
        {
            cout << "Marker #" << n << endl;
            
            //=========================================//
            // Extract Sauce's Depth Cloud(Orgainized)
            // 2D pixel mapping to 3D points
            //=========================================//
            marker_all[n].marker_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();
         
            int x1 = marker_all[n].corner_pixel[0];
            int x2 = marker_all[n].corner_pixel[2];
            int x3 = marker_all[n].corner_pixel[4];
            int x4 = marker_all[n].corner_pixel[6]; 
            int y1 = marker_all[n].corner_pixel[1];
            int y2 = marker_all[n].corner_pixel[3];
            int y3 = marker_all[n].corner_pixel[5];
            int y4 = marker_all[n].corner_pixel[7];
            cout<<x1<<","<<x2<<","<<x3<<","<<x4<<","<<y1<<","<<y2<<","<<y3<<","<<y4<<endl;
            // std::vector<int> x{x1, x2, x3, x4};
            // std::vector<int> y{y1, y2, y3, y4};
          
            // int xmax = *std::max_element(x.begin(), x.end());
            // int xmin = *std::min_element(x.begin(), x.end());
            // int ymax = *std::max_element(y.begin(), y.end());
            // int ymin = *std::min_element(y.begin(), y.end());

            int xmax = std::numeric_limits<int>::min();
            int xmin = std::numeric_limits<int>::max();
            int ymax = std::numeric_limits<int>::min();
            int ymin = std::numeric_limits<int>::max();

            if(x1>xmax) xmax = x1;
            if(x2>xmax) xmax = x2;
            if(x3>xmax) xmax = x3;
            if(x4>xmax) xmax = x4;

            if(y1>ymax) ymax = y1;
            if(y2>ymax) ymax = y2;
            if(y3>ymax) ymax = y3;
            if(y4>ymax) ymax = y4;

            if(x1<xmin) xmin = x1;
            if(x2<xmin) xmin = x2;
            if(x3<xmin) xmin = x3;
            if(x4<xmin) xmin = x4;

            if(y1<ymin) ymin = y1;
            if(y2<ymin) ymin = y2;
            if(y3<ymin) ymin = y3;
            if(y4<ymin) ymin = y4;

            // cout<< "\t!!!Pixel (xmin, xmax, ymin, ymax) = "<< xmin << ", " << xmax <<", " << ymin << ", " << ymax << endl;
            //Ensure the 2D pixels are inside image's max width, height
            if(xmin < 0) xmin = 0;//114;//186;//0;
            if(ymin < 0) ymin = 0;//40;//74;//0;
            int img_width = 640;
            int img_height = 480;
            if(xmax > img_width-1) xmax = 640;//723;//1085;//img_width-1;
            if(ymax > img_height-1) ymax = 480;//424;//648;//img_height-1;
            // cout<<"\timgwidth, imgHeight = "<< img_width <<",  "<< img_height<<endl;
            // cout<< "\tPixel (xmin, xmax, ymin, ymax) = "<< xmin << ", " << xmax <<", " << ymin << ", " << ymax << endl;

            //Map 2D pixel to 3D points
            for(int i = xmin; i <= xmax; i++)
            {
                for(int j = ymin; j<= ymax; j++)
                {
                    PointTRGB depth_pt = organized_cloud_ori->at(i, j);
                
                    if(pcl_isfinite(depth_pt.x) && pcl_isfinite(depth_pt.y) && pcl_isfinite(depth_pt.z))
                    {
                        marker_all[n].marker_cloud->push_back(depth_pt);
                        //tmp_cloud->push_back(depth_pt);
                    }
                }
            }
            cout << "\tExtract [depth_cloud] = " << marker_all[n].marker_cloud->size() << endl;
            // *top3_clouds = *top3_clouds + *(marker_all[n].marker_cloud);

            pcl::PointCloud<PointTRGB>::Ptr tmp(new pcl::PointCloud<PointTRGB>);
            float leaf = 0.005;
            pcl::VoxelGrid<PointTRGB> vg;
            vg.setInputCloud(marker_all[n].marker_cloud);
            vg.setLeafSize(leaf, leaf, leaf);
            vg.filter(*tmp);

            pcl::NormalEstimation<PointTRGB, pcl::Normal> nor;
            nor.setInputCloud(marker_all[n].marker_cloud);
            nor.setSearchSurface(tmp);

            //以kdtree作为索引方式
            pcl::search::KdTree<PointTRGB>::Ptr treeA(new pcl::search::KdTree<PointTRGB>);            
            nor.setSearchMethod(treeA);            
            
            //存储输出数据集
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);            
            
            nor.setRadiusSearch(0.03);                      
            nor.compute(*normals);

            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmpall(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::copyPointCloud<PointTRGB,pcl::PointXYZRGBNormal>(*marker_all[n].marker_cloud, *tmpall);
            for(int num = 0; num<tmpall->size(); num++)
            {
                tmpall->points[num].normal_x = normals->points[num].normal_x;
                tmpall->points[num].normal_y = normals->points[num].normal_y;
                tmpall->points[num].normal_z = normals->points[num].normal_z;
            }
            *top3_clouds = *top3_clouds + *tmpall;//*(marker_all[n].marker_cloud);


            get_3d_center(organized_cloud_ori, marker_all[n].marker_cloud, marker_all[n].center_pixel, marker_all[n].center_point);
            cout<<marker_all[n].center_point.x<<", "<<marker_all[n].center_point.y<<", "<<marker_all[n].center_point.z<<endl;
            std::string marker_coord_name = "marker_" + std::to_string(n);
            // cout<<"Marker_coord_name = "<< marker_coord_name <<endl;
            
            ////ROS tf
            // static tf::TransformBroadcaster br;
            // tf::Transform transform;
            
            // transform.setOrigin( tf::Vector3(marker_all[n].center_point.x, marker_all[n].center_point.y, marker_all[n].center_point.z));
            // transform.setRotation( tf::Quaternion(0, 0, 0, 1));             
            // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", marker_coord_name));//camera_depth_frame
            
            
            Eigen::Matrix3f vect;            
            CalculatePCA(marker_all[n].marker_cloud, vect);
            std::cout << "特征向量ve(3x3):\n" << vect << std::endl;
            tf2::Quaternion quat;
            tf2::Matrix3x3 mm;
            mm.setValue(vect(0),vect(1),vect(2),vect(3),vect(4),vect(5),vect(6),vect(7),vect(8));
            // mm.getRotation(quat);
            double r, p, y;
            
            // mm.to
            mm.getRPY(r,p,y); //0,1
            cout<<"rpy:"<<r<<", "<<p<<", "<<y<<endl;
            // r+=3.14159;
            // quat.normalize();
            // cout<<"quat:"<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl;

            ////ROS tf2
            static tf2_ros::StaticTransformBroadcaster sbr;
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = ros::Time::now();
            transform.header.frame_id = "camera_depth_optical_frame";
            transform.child_frame_id = marker_coord_name;
            transform.transform.translation.x = marker_all[n].center_point.x;
            transform.transform.translation.y = marker_all[n].center_point.y;
            transform.transform.translation.z = marker_all[n].center_point.z;

            // tf2::Quaternion quat;
            // quat.setRPY(0.0, 0.0, 0.0); //roll, pitch, yaw (around X, Y, Z)
            //quat.setRPY(tf::createQuaternionFromRPY (-1 * thetax, -1 * thetay, -1 * thetaz));
            cout<<"rpy:"<<r<<", "<<p<<", "<<y<<endl;
            cout<<"quat:"<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl;
            quat.setRPY(r,p,y);            
            transform.transform.rotation.x = quat.x();
            transform.transform.rotation.y = quat.y();
            transform.transform.rotation.z = quat.z();
            transform.transform.rotation.w = quat.w();

            sbr.sendTransform(transform);

            // cout<<normals->points[0].normal_x<<", "<<normals->points[0].normal_y<<", "<<normals->points[0].normal_z<<", "<<normals->points[0].curvature<<endl;
        }

        //Publish pcl::PointCloud to ROS sensor::PointCloud2, and to topic
        pcl::toROSMsg(*top3_clouds, top3_clouds_msg);        
        top3_clouds_msg.header.frame_id = "camera_depth_optical_frame";        
        top3_cloud_pub.publish(top3_clouds_msg);        
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_cloud");
    cout<<"inside\n";
    ros::NodeHandle nh;

    // tf::TransformBroadcaster br;
    
    
    // // 有序點雲 Organized Point Cloud 
    // ros::Subscriber sub_organized_cloud = nh.subscribe("/camera/depth_registered/points", 1, organized_cloud_cb);
    ros::Subscriber sub_aruco_cloud = nh.subscribe("/camera/depth_registered/points", 1, aruco_cloud_cb);

    // ArucoMarkers的角點 Corners of Aruco Markers
    ros::Subscriber sub_aruco_corners = nh.subscribe<std_msgs::Float64MultiArray>("/aruco_corners", 1, aruco_corners_cb);

    // ros::Subscriber sub_aruco_cloud = nh.subscribe("/camera/depth_registered/points", 1, aruco_cloud_cb);

    top3_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/top3_cloud_pub", 1);

    // ros::Rate rate(10.0);
    // tf::Transform transform;

    // while (nh.ok()){

    //     for(int n = 0; n< marker_all.size();n++)
    //     {
    //         std::string marker_coord_name = "marker_" + std::to_string(n);
    //         cout<<"Marker_coord_name = "<< marker_coord_name <<endl;

    //         transform.setOrigin( tf::Vector3(marker_all[n].center_point.x, marker_all[n].center_point.y, marker_all[n].center_point.z));
    //         transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    //         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_frame", "carrot1"));            
    //     }
    //     // rate.sleep();
    // }

    cout<<"try\n";
    ros::spin();

    return 0;
}
