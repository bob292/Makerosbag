#pragma once

#include <string>
#include <vector>
#include <iostream>

//PCL lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

#include "point_types.h"

namespace sad {

class FeatureExtraction {
    /// 一个线ID+曲率的结构
    struct 
    IdAndValue {
        IdAndValue() {}
        IdAndValue(int id, double value) : id_(id), value_(value) {}
        int id_ = 0;
        double value_ = 0;  // 曲率
    };

    public:
    FeatureExtraction() {};
    ~FeatureExtraction() {};
    void Extract(FullCloudPtr pc_in, CloudPtr pc_out_edge, CloudPtr pc_out_surf, std::vector<std::vector<int>> &edge_pixel_index);
    void ExtractFromSector(const CloudPtr &pc_in, std::vector<IdAndValue> &
                            cloud_curvature, CloudPtr &pc_out_edge, CloudPtr &pc_out_surf, std::vector<int> &edge_row_index);

};
}