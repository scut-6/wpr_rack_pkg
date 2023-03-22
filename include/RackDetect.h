#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#define RACK_TEMP_X 200
#define RACK_TEMP_Y 200
#define RACK_TEMP_Z 200

#define PLACE_MIDDLE    1
#define PLACE_COMPACT   2

typedef struct stRackTemp
{
    unsigned char data[RACK_TEMP_X][RACK_TEMP_Y][RACK_TEMP_Z];
    int h_partition_y[4];
    int v_partition_z[4];
    int d_partition_x;
    int b_back;
}stRackTemp;

typedef struct stScalePoint
{
    unsigned char data[40][40][40];
}stScalePoint;

typedef struct stICPTest
{
    float theta[3];
    float x_offset[5];
    float y_offset[5];
    int result[3][5];
}stICPTest;

typedef struct stICPResult
{
    float angle;
    float x_offset;
    float y_offset;
    int result;
}stICPResult;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

typedef struct stOperateHeight
{
    float fMaxZ;
    float fMinZ;
    float fHeight;
}stOperateHeight;

using namespace std;

class CRackDetect
{
public:
    CRackDetect();
    ~CRackDetect();
    void InitShape();
    virtual void GenTemp(){};
    int CalDist(int x1 ,int y1, int z1, int x2, int y2, int z2);
    void ScalePoint(int x, int y, int z);
    void SaveTemp(const char* filename);
    void LoadTemp(const char* filename);
    void FirstTest(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);
    void InitFirstTest();
    void InitICPStep();
    void InitScalePoint();
    bool SingleTest(stICPTest* inTest, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);
    void UpdateResult();
    void CloudToTemp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);
    void ScaleTemp();
    void ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);
    void GetCurCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);
    void CalRackPosition();
    void GetObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSrcCloud);
    void MovePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSrcCloud, float inX, float inY, float inAngle, bool bForward);
    float GetRackHeight();
    float GetRackLeft();
    float GetRackRight();
    float GetRackDepth();
    bool InSideRange(float inY, float inDist);
    bool inObjRange(float inX, float inY, float inZ, float inDist);
    bool inPlaceRange(float inX, float inY, float inZ, float inDist);
    bool bRestart;
    int nPlaceMode;
    float fGrabHeightMax;
    float fGrabHeightMin;
    float fCulling_v;
    float fCulling_h;
    float fPlaceDistSide;
    float fPlaceDistObj;
    float fGrabAbovePlane;
    float fRackHeight;
    float fRackLeft;
    float fRackRight;
    float fRackDepth;
    stScalePoint* pScalePoint;
    stRackTemp* pRackTemp;
    stICPResult* pArFirstTest;
    stICPTest* pICPStep;
    stICPResult stepResult;
    stICPResult curResult;
    stICPResult rackPosition;
    vector<stBoxMarker> arObject;
    vector<pcl::PointXYZRGB> arPlacePos;
};