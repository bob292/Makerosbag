//c++ lib
#include <iostream>
#include <string>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/voxel_grid.h>

//local lib
#include "createpointcloud.h"
#include "featureextraction.h"

ros::Publisher pubPointCloud;

void XYZI2FullCloudPtr(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &inputclouds, sad::FullCloudPtr &outputclouds) {
    for (int i=0; i<inputclouds.size(); i++) {
        for (auto point : *inputclouds[i]) {
            sad::FullPointType fp;
            fp.x = point.x;
            fp.y = point.y;
            fp.z = point.z;
            fp.intensity = 0.5;
            fp.ring = i;
            outputclouds->push_back(fp);
        }
    }
    return;
}

void TestFullCloud(const sad::FullCloudPtr &outputclouds, int ring) {
    int point_in_ring = 0;
    for (auto point : *outputclouds) {
        if (point.ring == ring) {
            point_in_ring++;
        }
    }
    ROS_INFO(std::to_string(point_in_ring).c_str());
    return;
}

// create a gray image to visualize where the edge points come from
void TestEdgePoint(const std::vector<std::vector<int>> &cloud_index,
                    const std::vector<std::vector<int>> &pixel_index) {
	int rows = 1024;
	int cols = 1344;
	cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC1);/* zeros可有可无 */
    for (int i=0; i<pixel_index.size(); i++) {
        for (auto index : cloud_index[i]) {
           img.ptr<u_char>(i)[pixel_index[i][index]] = 255;
        }
    }
	cv::imwrite("/floam/surface_cloud.png",img);//0质量最高，默认为1
}

void DownSample_Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_downsize,
                       double downsize) {
   pcl::VoxelGrid<pcl::PointXYZI> downsample_filter;
   downsample_filter.setLeafSize(downsize, downsize, downsize);
   downsample_filter.setDownsampleAllData(true);    //对全字段进行下采样;
   downsample_filter.setInputCloud(cloud_in);
   downsample_filter.filter(*cloud_downsize);
}

void PublishPointCloud(const std::string dir, const std::vector<std::string> &disparity_images) {
    MakePointCloud makepointcloud;
    int skip = 0;
    for (std::string img_name : disparity_images) {
        // use one image in every two
        if (skip++ % 2 == 1) {
            continue;
        }
        if (skip < 143) {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr entirepointcloud(new pcl::PointCloud<pcl::PointXYZI>()); 
        std::string img_path = dir + R"(/images_001/output_img/mpv_disp/)" + img_name;
        ROS_INFO(img_path.c_str());
        cv::Mat disparityimg = makepointcloud.load_image(img_path, MakePointCloud::ImageType::Disparity);
        // For testing purpose
        //cv::imwrite("/floam/disparity_img.jpeg",disparityimg);
        std::vector<std::vector<int>> pixel_index;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> depthringclouds;
        makepointcloud.Disparity2PointCloud(disparityimg, depthringclouds, entirepointcloud, pixel_index);
        // downsample
        DownSample_Filter(entirepointcloud, entirepointcloud, 0.02);
        // Feature extraction
        for (auto cloud : depthringclouds) {
            DownSample_Filter(cloud, cloud, 0.02);
        }
        sad::FullCloudPtr fullcloud(new sad::FullPointCloudType());
        XYZI2FullCloudPtr(depthringclouds,fullcloud);
        //TestFullCloud(fullcloud, 256);
        pcl::io::savePCDFileASCII("/floam/depth_cloudz26_y1_.pcd",*entirepointcloud);
        sad::FeatureExtraction featureextraction;
        sad::CloudPtr cloudedge(new sad::PointCloudType());
        sad::CloudPtr cloudsurface(new sad::PointCloudType());
        std::vector<std::vector<int>> cloud_index;
        featureextraction.Extract(fullcloud, cloudedge, cloudsurface, cloud_index);
        //TestEdgePoint(cloud_index, pixel_index);

        pcl::io::savePCDFileASCII("/floam/edge_cloud.pcd",*cloudedge);
        pcl::io::savePCDFileASCII("/floam/surface_cloud.pcd",*cloudsurface);
        // change coordinate frame
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        rotation << 1, 0, 0,
                    0, 0, -1,
                    0, 1, 0;
        Eigen::Vector3d t;
        t << 0, 0, 0;
        pose.rotate(rotation);
        pose.pretranslate(t);
        pcl::transformPointCloud(*entirepointcloud, *entirepointcloud, pose.matrix());
        sensor_msgs::PointCloud2 pubcloud;
        pcl::toROSMsg(*entirepointcloud,pubcloud);
        pubPointCloud.publish(pubcloud);
    }
    
    
    
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 100);
    std::string dir = R"(/media/bj_252_data/20230829)"; // disparity images
    MakePointCloud makepointcloud;
    std::vector<std::string> disparity_images;
    makepointcloud.ReadImage(dir, disparity_images);
    PublishPointCloud(dir, disparity_images);
    // Feature extraction
    /*sad::FullCloudPtr fullcloud(new sad::FullPointCloudType());
    XYZI2FullCloudPtr(depthringclouds,fullcloud);
    TestFullCloud(fullcloud, 256);
    pcl::io::savePCDFileASCII("/floam/depth_cloud.pcd",*entirepointcloud);
    sad::FeatureExtraction featureextraction;
    sad::CloudPtr cloudedge(new sad::PointCloudType());
    sad::CloudPtr cloudsurface(new sad::PointCloudType());
    std::vector<std::vector<int>> cloud_index;
    featureextraction.Extract(fullcloud, cloudedge, cloudsurface, cloud_index);
    TestEdgePoint(cloud_index, pixel_index);
    /*for (int i=0; i<pixel_index.size(); i++) {
        std::string row = "row " + std::to_string(i) + ": ";
        for (auto index : cloud_index[i]) {
           row = row + std::to_string(pixel_index[i][index]) + " ";    
        }
        if (cloud_index[i].size()) 
            ROS_INFO(row.c_str());
    }*/
    //ROS_INFO(cloudedge->size());
    //ROS_INFO(cloudsurface->size());
    /*pcl::io::savePCDFileASCII("/floam/edge_cloud.pcd",*cloudedge);
    pcl::io::savePCDFileASCII("/floam/surface_cloud.pcd",*cloudsurface);*/
    return 0;
}