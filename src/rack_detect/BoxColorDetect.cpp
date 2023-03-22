#include "BoxColorDetect.h"
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

static float box_min_width = 0.06;	//宽度大于这个才是盒子
static float box_max_width = 0.50;	//宽度小于这个才是盒子
//static float box_min_length = 0.20;	//长度度大于这个才是盒子
static float box_min_length = 0.03;	//被上方挡板遮住一部分，没那么长了
static float box_max_height = 0.3;	//高度高于这个肯定不是盒子

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clr_0(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clr_1(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clr_2(new pcl::PointCloud<pcl::PointXYZRGB>);

CBoxColorDetect::CBoxColorDetect()
{
    // 绿色料盒阈值
    thHSV[0].iLowH = 90;
    thHSV[0].iHighH = 180;
    thHSV[0].iLowS = 40; 
    thHSV[0].iHighS = 255;
    thHSV[0].iLowV = 30;
    thHSV[0].iHighV = 255;

    // 黄色料盒阈值
    thHSV[1].iLowH = 0;
    thHSV[1].iHighH = 80;
    thHSV[1].iLowS = 90; 
    thHSV[1].iHighS = 255;
    thHSV[1].iLowV = 150;
    thHSV[1].iHighV = 255;

    // 放置区域颜色阈值
    thHSV[2].iLowH = 90;
    thHSV[2].iHighH = 180;
    thHSV[2].iLowS = 40; 
    thHSV[2].iHighS = 255;
    thHSV[2].iLowV = 30;
    thHSV[2].iHighV = 255;

    fCenterX = 1.0;
    fCenterY = 0;
    fCenterZ = 0.7;
}
    
CBoxColorDetect::~CBoxColorDetect()
{
   
}

void CBoxColorDetect::Init()
{
   
}

void CBoxColorDetect::ColorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSrcCloud)
{
    int nNumCloud = inSrcCloud->points.size();
    //ROS_WARN("Thera are %d points in cloud_in",nNumCloud);
    int minH,minS,minV,maxH,maxS,maxV;
    minH=minS=minV=maxH=maxS=maxV = 100;
    int PntH,PntS,PntV;
    /************************************************/
    // 检测颜色 0
    cloud_clr_0->header   = inSrcCloud->header;
    cloud_clr_0->sensor_orientation_ = inSrcCloud->sensor_orientation_;
    cloud_clr_0->sensor_origin_ = inSrcCloud->sensor_origin_;
    cloud_clr_0->points.clear();
    for(int i=0;i<nNumCloud;i++)
    {
        RGB2HSI(inSrcCloud->points[i].r,inSrcCloud->points[i].g,inSrcCloud->points[i].b,&PntH, &PntS, &PntV);
        if(
            PntH >= thHSV[0].iLowH && PntH <= thHSV[0].iHighH &&
            PntS >= thHSV[0].iLowS && PntS <= thHSV[0].iHighS &&
            PntV >= thHSV[0].iLowV && PntV <= thHSV[0].iHighV 
        )
        {
            cloud_clr_0->points.push_back(inSrcCloud->points[i]);
        }
        if(PntH < minH) minH=PntH; if(PntH > maxH) maxH=PntH;
        if(PntS < minS) minS=PntS; if(PntS > maxS) maxS=PntS;
        if(PntV < minV) minV=PntV; if(PntV > maxV) maxV=PntV;
    }
    cloud_clr_0->width = cloud_clr_0->points.size();
    cloud_clr_0->height = 1;
    cloud_clr_0->is_dense = true;
    //ROS_INFO(" H ( %d , %d ) S ( %d , %d ) V ( %d , %d )",maxH,minH,maxS,minS,maxV,minV);
    int nNumCloudClr_0 = cloud_clr_0->points.size();
    ROS_WARN("[ColorFilter 0] %d  in / %d  Out",nNumCloud,nNumCloudClr_0);
    //pcl::copyPointCloud(*cloud_clr_0 , *inSrcCloud);
	// 提取联通域做盒子
	GetBox(0, cloud_clr_0);
    /************************************************/
    // 检测颜色 1
    cloud_clr_1->header   = inSrcCloud->header;
    cloud_clr_1->sensor_orientation_ = inSrcCloud->sensor_orientation_;
    cloud_clr_1->sensor_origin_ = inSrcCloud->sensor_origin_;
    cloud_clr_1->points.clear();
    for(int i=0;i<nNumCloud;i++)
    {
        RGB2HSI(inSrcCloud->points[i].r,inSrcCloud->points[i].g,inSrcCloud->points[i].b,&PntH, &PntS, &PntV);
        if(
            PntH >= thHSV[1].iLowH && PntH <= thHSV[1].iHighH &&
            PntS >= thHSV[1].iLowS && PntS <= thHSV[1].iHighS &&
            PntV >= thHSV[1].iLowV && PntV <= thHSV[1].iHighV 
        )
        {
            cloud_clr_1->points.push_back(inSrcCloud->points[i]);
        }
    }
    cloud_clr_1->width = cloud_clr_1->points.size();
    cloud_clr_1->height = 1;
    cloud_clr_1->is_dense = true;
    int nNumCloudClr_1 = cloud_clr_1->points.size();
    ROS_WARN("[ColorFilter 1] %d  in / %d  Out",nNumCloud,nNumCloudClr_1);
    //pcl::copyPointCloud(*cloud_clr_1 , *inSrcCloud);
    //合并两个盒子的点云，发送给模板匹配节点进行模板匹配
    *cloud_clr_0 = *cloud_clr_0 +*cloud_clr_1;
    pcl::copyPointCloud(*cloud_clr_0 , *inSrcCloud);
	// 提取联通域做盒子
	GetBox(1, cloud_clr_1);
     /************************************************/
    // 检测颜色 2
    cloud_clr_2->header   = inSrcCloud->header;
    cloud_clr_2->sensor_orientation_ = inSrcCloud->sensor_orientation_;
    cloud_clr_2->sensor_origin_ = inSrcCloud->sensor_origin_;
    cloud_clr_2->points.clear();
    for(int i=0;i<nNumCloud;i++)
    {
        RGB2HSI(inSrcCloud->points[i].r,inSrcCloud->points[i].g,inSrcCloud->points[i].b,&PntH, &PntS, &PntV);
        if(
            PntH >= thHSV[2].iLowH && PntH <= thHSV[2].iHighH &&
            PntS >= thHSV[2].iLowS && PntS <= thHSV[2].iHighS &&
            PntV >= thHSV[2].iLowV && PntV <= thHSV[2].iHighV 
        )
        {
            cloud_clr_2->points.push_back(inSrcCloud->points[i]);
        }
        if(PntH < minH) minH=PntH; if(PntH > maxH) maxH=PntH;
        if(PntS < minS) minS=PntS; if(PntS > maxS) maxS=PntS;
        if(PntV < minV) minV=PntV; if(PntV > maxV) maxV=PntV;
    }
    cloud_clr_2->width = cloud_clr_2->points.size();
    cloud_clr_2->height = 1;
    cloud_clr_2->is_dense = true;
    //ROS_INFO(" H ( %d , %d ) S ( %d , %d ) V ( %d , %d )",maxH,minH,maxS,minS,maxV,minV);
    int nNumCloudClr_2 = cloud_clr_2->points.size();
    ROS_WARN("[ColorFilter 2] %d  in / %d  Out",nNumCloud,nNumCloudClr_0);
    //pcl::copyPointCloud(*cloud_clr_0 , *inSrcCloud);
	// 提取联通域做盒子
	GetBox(2, cloud_clr_2);
}

bool CBoxColorDetect::SingleFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSrcCloud, int inBoxIndex, bool bGetBox)
{
    bool res = false;
    int nNumCloud = inSrcCloud->points.size();
    //ROS_WARN("Thera are %d points in cloud_in",nNumCloud);
    int minH,minS,minV,maxH,maxS,maxV;
    minH=minS=minV=maxH=maxS=maxV = 100;
    int PntH,PntS,PntV;
    if(inBoxIndex == 0)
    {
        // 检测颜色 0
        cloud_clr_0->header   = inSrcCloud->header;
        cloud_clr_0->sensor_orientation_ = inSrcCloud->sensor_orientation_;
        cloud_clr_0->sensor_origin_ = inSrcCloud->sensor_origin_;
        cloud_clr_0->points.clear();
        for(int i=0;i<nNumCloud;i++)
        {
            RGB2HSI(inSrcCloud->points[i].r,inSrcCloud->points[i].g,inSrcCloud->points[i].b,&PntH, &PntS, &PntV);
            if(
                PntH >= thHSV[0].iLowH && PntH <= thHSV[0].iHighH &&
                PntS >= thHSV[0].iLowS && PntS <= thHSV[0].iHighS &&
                PntV >= thHSV[0].iLowV && PntV <= thHSV[0].iHighV 
            )
            {
                cloud_clr_0->points.push_back(inSrcCloud->points[i]);
            }
            if(PntH < minH) minH=PntH; if(PntH > maxH) maxH=PntH;
            if(PntS < minS) minS=PntS; if(PntS > maxS) maxS=PntS;
            if(PntV < minV) minV=PntV; if(PntV > maxV) maxV=PntV;
        }
        cloud_clr_0->width = cloud_clr_0->points.size();
        cloud_clr_0->height = 1;
        cloud_clr_0->is_dense = true;
        //ROS_INFO(" H ( %d , %d ) S ( %d , %d ) V ( %d , %d )",maxH,minH,maxS,minS,maxV,minV);
        int nNumCloudClr_0 = cloud_clr_0->points.size();
        //ROS_WARN("[SingleFilter 0] %d  in / %d  Out",nNumCloud,nNumCloudClr_0);
        if(nNumCloudClr_0 > 0)
        {
            //提出来盒子的点云，发送给模板匹配节点进行模板匹配
            pcl::copyPointCloud(*cloud_clr_0 , *inSrcCloud);
            if(bGetBox == true)
            {
                // 提取联通域做盒子
                GetBox(0, cloud_clr_0);
                if(arBox[0].size() > 0)
                    res = true;
            }
        }
    }
    
    if(inBoxIndex == 1)
    {
        // 检测颜色 1
        cloud_clr_1->header   = inSrcCloud->header;
        cloud_clr_1->sensor_orientation_ = inSrcCloud->sensor_orientation_;
        cloud_clr_1->sensor_origin_ = inSrcCloud->sensor_origin_;
        cloud_clr_1->points.clear();
        for(int i=0;i<nNumCloud;i++)
        {
            RGB2HSI(inSrcCloud->points[i].r,inSrcCloud->points[i].g,inSrcCloud->points[i].b,&PntH, &PntS, &PntV);
            if(
                PntH >= thHSV[1].iLowH && PntH <= thHSV[1].iHighH &&
                PntS >= thHSV[1].iLowS && PntS <= thHSV[1].iHighS &&
                PntV >= thHSV[1].iLowV && PntV <= thHSV[1].iHighV 
            )
            {
                cloud_clr_1->points.push_back(inSrcCloud->points[i]);
            }
        }
        cloud_clr_1->width = cloud_clr_1->points.size();
        cloud_clr_1->height = 1;
        cloud_clr_1->is_dense = true;
        int nNumCloudClr_1 = cloud_clr_1->points.size();
        //ROS_WARN("[SingleFilter 1] %d  in / %d  Out",nNumCloud,nNumCloudClr_1);
        if(nNumCloudClr_1 > 0)
        {
            //提出来盒子的点云，发送给模板匹配节点进行模板匹配
            pcl::copyPointCloud(*cloud_clr_1 , *inSrcCloud);
            if(bGetBox == true)
            {
                // 提取联通域做盒子
                GetBox(1, cloud_clr_1);
                if(arBox[1].size() > 0)
                    res = true;
            }
        }
    }
    if(inBoxIndex == 2)
    {
        // 检测颜色 2
        cloud_clr_2->header   = inSrcCloud->header;
        cloud_clr_2->sensor_orientation_ = inSrcCloud->sensor_orientation_;
        cloud_clr_2->sensor_origin_ = inSrcCloud->sensor_origin_;
        cloud_clr_2->points.clear();
        for(int i=0;i<nNumCloud;i++)
        {
            RGB2HSI(inSrcCloud->points[i].r,inSrcCloud->points[i].g,inSrcCloud->points[i].b,&PntH, &PntS, &PntV);
            if(
                PntH >= thHSV[2].iLowH && PntH <= thHSV[2].iHighH &&
                PntS >= thHSV[2].iLowS && PntS <= thHSV[2].iHighS &&
                PntV >= thHSV[2].iLowV && PntV <= thHSV[2].iHighV 
            )
            {
                cloud_clr_2->points.push_back(inSrcCloud->points[i]);
            }
        }
        cloud_clr_2->width = cloud_clr_2->points.size();
        cloud_clr_2->height = 1;
        cloud_clr_2->is_dense = true;
        int nNumCloudClr_2 = cloud_clr_2->points.size();
        ROS_WARN("[SingleFilter-2] %d  in / %d  Out",nNumCloud,nNumCloudClr_2);
        if(nNumCloudClr_2 > 0)
        {
            //提出来盒子的点云，发送给模板匹配节点进行模板匹配
            pcl::copyPointCloud(*cloud_clr_2 , *inSrcCloud);
            if(bGetBox == true)
            {
                // 提取联通域做盒子
                GetBox(2, cloud_clr_2);
                if(arBox[2].size() > 0)
                    res = true;
            }
        }
    }
    return res;
}

void CBoxColorDetect::GetBox(int inIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inFiltedCloud)
{
    if(inFiltedCloud->points.size() == 0)
        return;
	// 对点云1进行K临域检测 提取点云簇
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(inFiltedCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01); // 单位：米
    ec.setMinClusterSize(30);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (inFiltedCloud);
    ec.extract (cluster_indices);
    
	arBox[inIndex].clear();
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        stBoxMarker obj_marker;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (inFiltedCloud->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        bool bFirstPoint = true;
        int cluster_size = cloud_cluster->points.size();
        for (int i=0;i<cluster_size;i++)
        {
            if(bFirstPoint == true)
            {
                bFirstPoint = false;
                obj_marker.xMax = obj_marker.xMin = cloud_cluster->points[i].x;
                obj_marker.yMax = obj_marker.yMin = cloud_cluster->points[i].y;
                obj_marker.zMax = obj_marker.zMin = cloud_cluster->points[i].z;
            }
            if(cloud_cluster->points[i].x > obj_marker.xMax) obj_marker.xMax = cloud_cluster->points[i].x;
            if(cloud_cluster->points[i].x < obj_marker.xMin) obj_marker.xMin = cloud_cluster->points[i].x;
            if(cloud_cluster->points[i].y > obj_marker.yMax) obj_marker.yMax = cloud_cluster->points[i].y;
            if(cloud_cluster->points[i].y < obj_marker.yMin) obj_marker.yMin = cloud_cluster->points[i].y;
            if(cloud_cluster->points[i].z > obj_marker.zMax) obj_marker.zMax = cloud_cluster->points[i].z;
            if(cloud_cluster->points[i].z < obj_marker.zMin) obj_marker.zMin = cloud_cluster->points[i].z;
        }
        //ROS_INFO("obj[%d] x(%.2f , %.2f) y(%.2f , %.2f) z(%.2f , %.2f)",j,obj_marker.xMax,obj_marker.xMin,obj_marker.yMax,obj_marker.yMin,obj_marker.zMax,obj_marker.zMin);
        if(
            fabs(obj_marker.yMax-obj_marker.yMin) > box_min_width &&  
            fabs(obj_marker.yMax-obj_marker.yMin) < box_max_width && 
            fabs(obj_marker.xMax-obj_marker.xMin) > box_min_length && 
            fabs(obj_marker.zMax-obj_marker.zMin) < box_max_height
            )
        {
			//ROS_WARN("obj[%d] x(%.2f , %.2f) y(%.2f , %.2f) z(%.2f , %.2f)",j,obj_marker.xMax,obj_marker.xMin,obj_marker.yMax,obj_marker.yMin,obj_marker.zMax,obj_marker.zMin);
            arBox[inIndex].push_back(obj_marker);
            j ++;
        }
    }
    int nNumObj = arBox[inIndex].size();
    ROS_INFO("arBox[%d] Num= %d",inIndex,nNumObj);
	if(nNumObj > 0)
	{
		//找出中心离观测点最近的作为盒子
		int nIndexClosest = 0;
		float minDist = CalDist( (arBox[inIndex][0].xMax+arBox[inIndex][0].xMin)/2 ,  (arBox[inIndex][0].yMax+arBox[inIndex][0].yMin)/2 , (arBox[inIndex][0].zMax+arBox[inIndex][0].zMin)/2 , fCenterX, fCenterY, fCenterZ);
		for(int i=0;i<nNumObj;i++)
		{
			float tmpDist = CalDist( (arBox[inIndex][i].xMax+arBox[inIndex][i].xMin)/2 ,  (arBox[inIndex][i].yMax+arBox[inIndex][i].yMin)/2 , (arBox[inIndex][i].zMax+arBox[inIndex][i].zMin)/2 , fCenterX, fCenterY, fCenterZ);
			if(tmpDist < minDist)
			{
				minDist = tmpDist;
				nIndexClosest = i;
			}
            //if(nNumObj > 1)
            //ROS_WARN("arBox[%d][%d] (%.2f , %.2f , %.2f) w=%.2f h=%.2f",inIndex,i,(arBox[inIndex][i].xMax+arBox[inIndex][i].xMin)/2 ,  (arBox[inIndex][i].yMax+arBox[inIndex][i].yMin)/2 , (arBox[inIndex][i].zMax+arBox[inIndex][i].zMin)/2,(arBox[inIndex][i].yMax-arBox[inIndex][i].yMin),(arBox[inIndex][i].xMax-arBox[inIndex][i].xMin));
		}
		nIndexBox[inIndex] = nIndexClosest;
		float cx = (arBox[inIndex][nIndexClosest].xMax + arBox[inIndex][nIndexClosest].xMin)/2;
		float cy = (arBox[inIndex][nIndexClosest].yMax + arBox[inIndex][nIndexClosest].yMin)/2;
		float cz = (arBox[inIndex][nIndexClosest].zMax + arBox[inIndex][nIndexClosest].zMin)/2;
		//ROS_WARN("BoxClosest_%d (%.2f , %.2f , %.2f)",inIndex,cx,cy,cz);
	}
}

int _Max(int x, int y, int z)
{
	//取x,y,z中的最大值
	if(x>=y && x>=z)
		return x;
	if(y>=x && y>=z)
		return y;
	if(z>=y && z>=x)
		return z;
	return z;
}

int _Min(int x, int y, int z)
{
	//取x,y,z中的最小值
	if(x<=y && x<=z)
		return x;
	if(y<=x && y<=z)
		return y;
	if(z<=y && z<=x)
		return z;
	return z;
}

void CBoxColorDetect::RGB2HSI(int inR,int inG,int inB,int* outH, int* outS, int* outI)
{
    unsigned char R;
	unsigned char G;
	unsigned char B;
	int H;
	int S;
	int I;
	int max;
	int min;

    B = inB;
	G = inG;
	R = inR;

	max = _Max(R,G,B);
	min = _Min(R,G,B);

	//计算H
	if(max!=min)
	{
		if(max==R)
		{
			H=60*(G-B)/(max-min);
			if(G<B && ((max-min)+360)!= 0)
				H=60*(G-B)/(max-min)+360;
            else
                H=0;
            
		}
		if(max==G)
			H=60*(B-R)/(max-min)+120;
		if(max==B)
			H=60*(R-G)/(max-min)+240;
	}
	else
	H=0;

	//计算S
	S = max-min;

	//计算I
	I = max;

	//将结果赋值到HSI矩阵里
	*outH = H;
	*outS = S;
	*outI = I;
}

float CBoxColorDetect::CalDist(float x1 ,float y1, float z1, float x2, float y2, float z2)
{
    float dist = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);
    dist = sqrt(dist);
    return dist;
}
