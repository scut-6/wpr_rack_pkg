#pragma once
#include "RackDetect.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

typedef struct stTrackTest
{
    float x_offset[5];
    float y_offset[5];
    int result[5];
}stTrackTest;

using namespace std;

class CBoxTrack
{
public:
    CBoxTrack();
    ~CBoxTrack();
    void InitShape();
    void LoadTemp(const char* filename);
    void InitTrackStep();
    void InitICPStep();
    bool SingleTest(stTrackTest* inTest, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);
    void GetCurCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);
    void CalBoxPosition();
    float GetMaxHeight();
    float GetMinHeight();
    stRackTemp* pRackTemp;
    stTrackTest* pICPStep;
    stICPResult stepResult;
    stICPResult curResult;
    stICPResult boxPosition;
};