#ifndef _INCLUDE_CREATE_POINTCLOUD_H_
#define _INCLUDE_CREATE_POINTCLOUD_H_

#include <string>
#include <vector>
#include <iostream>
#include <experimental/filesystem>
//OpenCV lib
#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//PCL lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

class MakePointCloud {  
public:
    MakePointCloud();
    ~MakePointCloud();
    enum class ImageType { Left, Right, Disparity, LeftColor, ImageParam, DisparityDSP };
    std::vector<std::string> get_collection_image_path(const std::string &collection_path,
                                                                ImageType image_type,
                                                                const std::vector<std::string> &patterns);
    cv::Mat load_image(const std::string &path, const ImageType type);
    void ReadImage(const std::string &path, std::vector<std::string> &disparity_images);
    void Disparity2PointCloud(const cv::Mat &disparityimg, 
                                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &outputclouds,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &entirepointcloud,
                                std::vector<std::vector<int>> &pixel_index);
    
};

#endif // _INCLUDE_CREATE_POINTCLOUD_H_