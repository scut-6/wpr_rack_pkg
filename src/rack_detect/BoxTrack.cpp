#include "BoxTrack.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr move_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZRGB>);     //用于迭代计算的点云

CBoxTrack::CBoxTrack()
{
    memset(&boxPosition,0,sizeof(boxPosition));
    pRackTemp = new stRackTemp;
    InitShape();
    pICPStep = new stTrackTest;
    InitICPStep();
}
    
CBoxTrack::~CBoxTrack()
{
    delete pRackTemp;
    delete pICPStep;
}

void CBoxTrack::InitShape()
{
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                pRackTemp->data[x][y][z] = 0;
            }
    memset(pRackTemp,0,sizeof(stRackTemp));
}

void CBoxTrack::LoadTemp(const char* filename)
{
    int fd = open(filename, O_RDONLY);

    if(fd == -1) 
    {
        printf("error is %s\n", strerror(errno));
        return;
    }

    read(fd, pRackTemp, sizeof(stRackTemp));
    close(fd);
}

void CBoxTrack::InitICPStep()
{
    pICPStep->x_offset[0] = 0;
    pICPStep->y_offset[0] = 0;
    pICPStep->x_offset[1] = -1;
    pICPStep->y_offset[1] = 0;
    pICPStep->x_offset[2] = 1;
    pICPStep->y_offset[2] = 0;
    pICPStep->x_offset[3] = 0;
    pICPStep->y_offset[3] = -1;
    pICPStep->x_offset[4] = 0;
    pICPStep->y_offset[4] = 1;
}

bool CBoxTrack::SingleTest(stTrackTest* inTest, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud)
{
    bool res = false;
    // 计算当前盒子位置，用这个位置去移动点云
    CalBoxPosition();
    // 将上一帧的盒子移到中心
    Eigen::Affine3f transform_offset = Eigen::Affine3f::Identity();
    float fx = -boxPosition.x_offset;
    float fy = -boxPosition.y_offset;
    transform_offset.translation() << fx, fy, 0.0;
    pcl::transformPointCloud (*inCloud, *cloud_mid, transform_offset);

    //遍历叠加偏移量，求匹配值
    for(int oi=0;oi<5;oi++)
    {
        //printf("SingleTest ai= %d  angle = %.2f ; io= %d (%.2f,%.2f)\n",ai,inTest->theta[ai],oi,inTest->x_offset[oi],inTest->y_offset[oi]);
        inTest->result[oi] = 0;
        // 1、先按照偏移量将点云拉到原点
        int x_offset = inTest->x_offset[oi];
        int y_offset = inTest->y_offset[oi];
        transform_offset = Eigen::Affine3f::Identity();
        fx = (float)x_offset/100;
        fy = (float)y_offset/100;
        transform_offset.translation() << fx, fy, 0.0;
        pcl::transformPointCloud (*cloud_mid, *move_cloud, transform_offset);

        int nResult = 0;
        int nNumPoints = move_cloud->points.size();
        int tx,ty,tz;
        for(int i=0;i<nNumPoints;i++)
        {
            tx = move_cloud->points[i].x*100 + 100; //模板的中心位置在（100,100）
            ty = move_cloud->points[i].y*100 + 100; //模板的中心位置在（100,100）
            tz = move_cloud->points[i].z*100;
            if(tx >= 0 && tx < RACK_TEMP_X && ty >= 0 && ty < RACK_TEMP_Y && tz >= 0 && tz < RACK_TEMP_Z)
                nResult += pRackTemp->data[tx][ty][tz];
        }
        //printf("[%d] ( %d,%d ) = ( %d )\n", oi,x_offset,y_offset,nResult);
        inTest->result[oi] = nResult;
    }
    
    //找出最佳匹配偏移量，并放置到stepResult。给res赋值
    stepResult.angle = 0;
    stepResult.x_offset = inTest->x_offset[0];
    stepResult.y_offset = inTest->y_offset[0];
    stepResult.result = inTest->result[0];
    for(int oi=0;oi<5;oi++)
    {
        if(inTest->result[oi] > stepResult.result)
        {
            stepResult.angle = 0;
            stepResult.x_offset = inTest->x_offset[oi];
            stepResult.y_offset = inTest->y_offset[oi];
            stepResult.result = inTest->result[oi];
        }
    }
    
    //printf("stepResult = ( %.2f , %.2f )\n",(float)stepResult.x_offset/100,(float)stepResult.y_offset/100);
    //printf("stepResult angle= %.2f ( %.2f , %.2f ) res= %d\n",stepResult.angle,stepResult.x_offset,stepResult.y_offset,stepResult.result);
    if(stepResult.angle == 0 && stepResult.x_offset == 0 && stepResult.y_offset == 0)
        res = true;

    curResult.angle -= stepResult.angle;
    curResult.x_offset -= stepResult.x_offset;
    curResult.y_offset -= stepResult.y_offset;
    CalBoxPosition();

    return res;
}

void CBoxTrack::GetCurCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud)
{
    outCloud->header   = cur_cloud->header;
    outCloud->width    = cur_cloud->width;
    outCloud->height   = cur_cloud->height;
    outCloud->is_dense = cur_cloud->is_dense;
    outCloud->sensor_orientation_ = cur_cloud->sensor_orientation_;
    outCloud->sensor_origin_ = cur_cloud->sensor_origin_;
    outCloud->points.resize (cur_cloud->points.size ());
    memcpy (&outCloud->points[0], &cur_cloud->points[0], cur_cloud->points.size () * sizeof (pcl::PointXYZRGB));
}

void CBoxTrack::CalBoxPosition()
{
    boxPosition.x_offset = curResult.x_offset/100;
    boxPosition.y_offset = curResult.y_offset/100;
    boxPosition.angle = curResult.angle;
    boxPosition.result = curResult.result;
}

float CBoxTrack::GetMaxHeight()
{
    int height = pRackTemp->v_partition_z[0];
    for(int i=1;i<4;i++)
    {
        if(pRackTemp->v_partition_z[i] > height)
        {
            height = pRackTemp->v_partition_z[i];
        }
    }
    float res = (float)height/100;
    return res;
}

float CBoxTrack::GetMinHeight()
{
    int height = pRackTemp->v_partition_z[0];
    for(int i=1;i<4;i++)
    {
        if(pRackTemp->v_partition_z[i] < height)
        {
            height = pRackTemp->v_partition_z[i];
        }
    }
    float res = (float)height/100;
    return res;
}