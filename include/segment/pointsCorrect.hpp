#ifndef LIO_LIVOX_POINTS_CORRECT_HPP
#define LIO_LIVOX_POINTS_CORRECT_HPP

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>

#include <vector>

using namespace std;

typedef struct
{
    Eigen::Matrix3f eigenVectorsPCA;
    Eigen::Vector3f eigenValuesPCA;
    std::vector<int> neibors;
} SNeiborPCA_cor;

int GetNeiborPCA_cor(SNeiborPCA_cor &npca, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree, pcl::PointXYZ searchPoint, float fSearchRadius);
int FilterGndForPos_cor(float* outPoints,float*inPoints,int inNum);
int CalGndPos_cor(float *gnd, float *fPoints,int pointNum,float fSearchRadius);
/**
 * Compute the proper rotation that aligns source_direction with target_direction.
 *
 * RTM is written in column-major order, matching the point multiplication used by
 * CorrectPoints() and CorrectPoints_cor(). The input vectors are never modified.
 * Returns 0 on success and -1 for null, non-finite, or zero-length directions.
 */
int ComputeVectorAlignmentRotation(float *RTM,
                                   const float *source_direction,
                                   const float *target_direction);
int GetRTMatrix_cor(float *RTM, float *v0, float *v1);
int CorrectPoints_cor(float *fPoints,int pointNum,float *gndPos);
int GetGndPos(float *pos, float *fPoints,int pointNum);
#endif  // LIO_LIVOX_POINTS_CORRECT_HPP
