#include "RackDetect.h"
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
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr rot_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZRGB>);     //用于迭代计算的点云
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_place(new pcl::PointCloud<pcl::PointXYZRGB>);

CRackDetect::CRackDetect()
{
    pScalePoint = new stScalePoint;
    for(int x=0;x<40;x++)
        for(int y=0;y<40;y++)
            for(int z=0;z<40;z++)
            {
                pScalePoint->data[x][y][z] = 0;
            }
    memset(&rackPosition,0,sizeof(rackPosition));
    pRackTemp = new stRackTemp;
    InitShape();
    pArFirstTest = new stICPResult[361];
    InitFirstTest();
    pICPStep = new stICPTest;
    InitICPStep();
    bRestart = true;
    nPlaceMode = PLACE_COMPACT;
    fCulling_h = 0.08;
    fPlaceDistSide = 0.3;
    fPlaceDistObj = 0.2;
    fGrabAbovePlane = 0.08;
    fGrabHeightMax = 1.2;
    fGrabHeightMin = 0.6;
}
    
CRackDetect::~CRackDetect()
{
    delete pScalePoint;
    delete pRackTemp;
    delete pICPStep;
    delete []pArFirstTest;
}

void CRackDetect::InitScalePoint()
{
    for(int x=0;x<40;x++)
        for(int y=0;y<40;y++)
            for(int z=0;z<40;z++)
            {
                int dist = CalDist(x, y, z, 20, 20, 20);
                int value = 255 - dist*15;
                    if(value < 0) value = 0;
                pScalePoint->data[x][y][z] = value;
            }
}

void CRackDetect::InitShape()
{
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                pRackTemp->data[x][y][z] = 0;
            }
    memset(pRackTemp,0,sizeof(stRackTemp));
}

int CRackDetect::CalDist(int x1 ,int y1, int z1, int x2, int y2, int z2)
{
    int dist = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);
    dist = sqrt(dist);
    return dist;
}

void CRackDetect::ScalePoint(int x, int y, int z)
{
    for(int range = 0 ; range <20 ; range ++)
    {
        for(int tx= x-range ;tx<x+range;tx++)
            for(int ty= y-range ;ty<y+range;ty++)
                for(int tz= z-range ;tz<z+range;tz++)
                {
                    if( (tx>=0 && tx < RACK_TEMP_X) && (ty>=0 && ty < RACK_TEMP_Y) && (tz>=0 && tz < RACK_TEMP_Z))
                    {
                        int ix = tx - x + 20;
                        int iy = ty - y + 20;
                        int iz = tz - z + 20;
                        int value =  pScalePoint->data[ix][iy][iz];
                        if(pRackTemp->data[tx][ty][tz] < value)
                        {
                            pRackTemp->data[tx][ty][tz] = value;
                        }
                    }
                }
    }
}


void CRackDetect::SaveTemp(const char* filename)
{
    printf("SaveTemp %s \n",filename);
    // int fd = open(filename, O_CREAT | O_RDWR);
    int fd = open(filename, O_CREAT | O_RDWR , 0777);
    if(fd == -1) 
    {
        printf("error is %s\n", strerror(errno));
        return;
    }
    write(fd, pRackTemp, sizeof(stRackTemp));
    close(fd);
}

void CRackDetect::LoadTemp(const char* filename)
{
    int fd = open(filename, O_RDONLY);

    if(fd == -1) 
    {
        printf("error is %s\n", strerror(errno));
        return;
    }

    read(fd, pRackTemp, sizeof(stRackTemp));
    close(fd);

    fRackHeight = GetRackHeight();
    fRackLeft = GetRackLeft();
    fRackRight = GetRackRight();
    fRackDepth = GetRackDepth();
}

void CRackDetect::InitFirstTest()
{ 
    for(int yi=0;yi<19;yi++)
        for(int xi=0;xi<19;xi++)
        {
            pArFirstTest[yi*19 + xi].angle = 0;
            pArFirstTest[yi*19 + xi].x_offset = -45 + xi*5; //为Mani缩减了搜索范围
            pArFirstTest[yi*19 + xi].y_offset = -45 + yi*5; //正常应该为-90 + yi*10
            //printf("[%d][%d] = ( %d , %d )\n", xi,yi,pArFirstTest[i].x_offset[xi][yi],pArFirstTest[i].y_offset[xi][yi]);
        }
}

void CRackDetect::FirstTest(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud)
{
    //std::cout << "[FirstTest]There are " << inCloud->points.size () << " points in rankTemp" << std::endl;
    // 第一帧，把所有位置都检测一下
    for(int i=0;i<361;i++)
    {
        pArFirstTest[i].result = 0;
        int nNumPoints = inCloud->points.size();
        int tx,ty,tz;
        for(int j=0;j<nNumPoints;j++)
        {
            tx = inCloud->points[j].x*100 - pArFirstTest[i].x_offset;
            ty = inCloud->points[j].y*100 - pArFirstTest[i].y_offset + 100; //模板的中心在(1米，1米)处
            tz = inCloud->points[j].z*100;
            if(tx >= 0 && tx < RACK_TEMP_X && ty >= 0 && ty < RACK_TEMP_Y && tz >= 0 && tz < RACK_TEMP_Z)
                pArFirstTest[i].result += pRackTemp->data[tx][ty][tz];
        }
    }
    // 从结果中挑选匹配值最大的
    curResult.angle = pArFirstTest[0].angle;
    curResult.x_offset = pArFirstTest[0].x_offset;
    curResult.y_offset = pArFirstTest[0].y_offset;
    curResult.result = pArFirstTest[0].result;
    for(int i=0;i<361;i++)
    { 
        if(pArFirstTest[i].result > curResult.result)
        {
            curResult.angle = pArFirstTest[i].angle;
            curResult.x_offset = pArFirstTest[i].x_offset;
            curResult.y_offset = pArFirstTest[i].y_offset;
            curResult.result = pArFirstTest[i].result;
        }
        //ROS_INFO("[%d] ( %.2f,%.2f ) - %.0f = ( %d )",i,  pArFirstTest[i].x_offset, pArFirstTest[i].y_offset, pArFirstTest[i].angle, pArFirstTest[i].result);
    }
    ROS_INFO("[FirstTest] 初始位置 ( %.2f,%.2f ) - %.0f = ( %d )", curResult.x_offset,curResult.y_offset,curResult.angle,curResult.result);
}

void CRackDetect::InitICPStep()
{
    pICPStep->theta[0] = -1*M_PI / 180;
    pICPStep->theta[1] = 0;
    pICPStep->theta[2] = M_PI / 180;
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

bool CRackDetect::SingleTest(stICPTest* inTest, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud)
{
    bool res = false;
    //遍历叠加偏移量，求匹配值
    for(int ai=0;ai<3;ai++)
        for(int oi=0;oi<5;oi++)
        {
            //printf("SingleTest ai= %d  angle = %.2f ; io= %d (%.2f,%.2f)\n",ai,inTest->theta[ai],oi,inTest->x_offset[oi],inTest->y_offset[oi]);
            inTest->result[ai][oi] = 0;
            // 1、先按照偏移量将点云拉到原点
            int x_offset = inTest->x_offset[oi];
            int y_offset = inTest->y_offset[oi];
            Eigen::Affine3f transform_offset = Eigen::Affine3f::Identity();
            float fx = (float)x_offset/100 - 1.0;
            float fy = (float)y_offset/100;
            transform_offset.translation() << fx, fy, 0.0;
            pcl::transformPointCloud (*inCloud, *move_cloud, transform_offset);

            // 2、旋转进来的点云
            Eigen::Affine3f transform_rot = Eigen::Affine3f::Identity();
            transform_rot.rotate (Eigen::AngleAxisf (inTest->theta[ai], Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud (*move_cloud, *rot_cloud, transform_rot);
            
            int nResult = 0;
            int nNumPoints = rot_cloud->points.size();
            int tx,ty,tz;
            for(int i=0;i<nNumPoints;i++)
            {
                tx = rot_cloud->points[i].x*100 + 100; //模板的中心位置在（100,100）
                ty = rot_cloud->points[i].y*100 + 100; //模板的中心位置在（100,100）
                tz = rot_cloud->points[i].z*100;
                if(tx >= 0 && tx < RACK_TEMP_X && ty >= 0 && ty < RACK_TEMP_Y && tz >= 0 && tz < RACK_TEMP_Z)
                    nResult += pRackTemp->data[tx][ty][tz];
            }
            //printf("[%d][%d] ( %d,%d ) = ( %d )\n", xi,yi,x_offset,y_offset,nResult);
            inTest->result[ai][oi] = nResult;
        }
    
    //找出最佳匹配偏移量，并放置到stepResult。给res赋值
    stepResult.angle = inTest->theta[0];
    stepResult.x_offset = inTest->x_offset[0];
    stepResult.y_offset = inTest->y_offset[0];
    stepResult.result = inTest->result[0][0];
    for(int ai=0;ai<3;ai++)
        for(int oi=0;oi<5;oi++)
        {
            if(inTest->result[ai][oi] > stepResult.result)
            {
                stepResult.angle = inTest->theta[ai];
                stepResult.x_offset = inTest->x_offset[oi];
                stepResult.y_offset = inTest->y_offset[oi];
                stepResult.result = inTest->result[ai][oi];
            }
        }
    
    //ROS_INFO("stepResult angle= %.2f ( %.2f , %.2f ) res= %d",stepResult.angle,stepResult.x_offset,stepResult.y_offset,stepResult.result);
    if(stepResult.angle == 0 && stepResult.x_offset == 0 && stepResult.y_offset == 0)
        res = true;

    return res;
}

void CRackDetect::UpdateResult()
{
    // 将stepResult偏移量叠加到 cur_cloud 和 curResult
    curResult.angle -= stepResult.angle;
    curResult.x_offset -= stepResult.x_offset;
    curResult.y_offset -= stepResult.y_offset;
    //printf("[curResult] angle= %.2f offset( %.2f , %.2f )\n",curResult.angle,curResult.x_offset,curResult.y_offset);

    // 1、先按照偏移量将点云拉到原点
    int x_offset = stepResult.x_offset;
    int y_offset = stepResult.y_offset;
    Eigen::Affine3f transform_offset = Eigen::Affine3f::Identity();
    float fx = (float)x_offset/100 - 1.0;
    float fy = (float)y_offset/100;
    transform_offset.translation() << fx, fy, 0.0;
    pcl::transformPointCloud (*cur_cloud, *move_cloud, transform_offset);
    
    // 2、旋转点云
    Eigen::Affine3f transform_rot = Eigen::Affine3f::Identity();
    transform_rot.rotate (Eigen::AngleAxisf (stepResult.angle, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*move_cloud, *rot_cloud, transform_rot);

    // 3、再将点云放回1米处
    transform_offset = Eigen::Affine3f::Identity();
    transform_offset.translation() << 1.0, 0.0, 0.0;
    pcl::transformPointCloud (*rot_cloud, *cur_cloud, transform_offset);
}

void CRackDetect::ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud)
{
    cur_cloud->header   = inCloud->header;
    cur_cloud->width    = inCloud->width;
    cur_cloud->height   = inCloud->height;
    cur_cloud->is_dense = inCloud->is_dense;
    cur_cloud->sensor_orientation_ = inCloud->sensor_orientation_;
    cur_cloud->sensor_origin_ = inCloud->sensor_origin_;
    cur_cloud->points.resize (inCloud->points.size ());
    memcpy (&cur_cloud->points[0], &inCloud->points[0], inCloud->points.size () * sizeof (pcl::PointXYZRGB));
    
    if(bRestart == true)
    {
        FirstTest(cur_cloud);
        // 按照初始匹配拉过来
        int x_offset = -curResult.x_offset;
        int y_offset = -curResult.y_offset;
        Eigen::Affine3f transform_offset = Eigen::Affine3f::Identity();
        float fx = (float)x_offset/100;
        float fy = (float)y_offset/100;
        transform_offset.translation() << fx, fy, 0.0;
        pcl::transformPointCloud (*cur_cloud, *cur_cloud, transform_offset);
        bRestart = false;
    }

    //printf("bGot = false\n");
    bool bGot = false;
    while(bGot == false)
    {
        bGot = SingleTest(pICPStep, cur_cloud);
        UpdateResult();
    }
    //printf("done\n");
}

void CRackDetect::GetCurCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud)
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

void CRackDetect::CalRackPosition()
{
    rackPosition.x_offset = curResult.x_offset/100 + 1.0;
    rackPosition.y_offset = curResult.y_offset/100;
    rackPosition.angle = curResult.angle;
    rackPosition.result = curResult.result;
}

void CRackDetect::MovePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSrcCloud, float inX, float inY, float inAngle, bool bForward)
{
    if(bForward == true)
    {
        // 先平移点云
        Eigen::Affine3f transform_move = Eigen::Affine3f::Identity();
        transform_move.translation() << inX, inY, 0.0;
        pcl::transformPointCloud (*inSrcCloud, *move_cloud, transform_move);
        
        // 后旋转点云
        Eigen::Affine3f transform_rot = Eigen::Affine3f::Identity();
        transform_rot.rotate (Eigen::AngleAxisf (inAngle, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*move_cloud, *inSrcCloud, transform_rot);
    }
    else
    {
        // 先旋转点云
        Eigen::Affine3f transform_rot = Eigen::Affine3f::Identity();;
        transform_rot.rotate (Eigen::AngleAxisf (inAngle, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*inSrcCloud, *rot_cloud, transform_rot);
        // 后平移点云
        Eigen::Affine3f transform_move = Eigen::Affine3f::Identity();
        transform_move.translation() << inX, inY, 0.0;
        pcl::transformPointCloud (*rot_cloud, *inSrcCloud, transform_move);
        
    }
}

float CRackDetect::GetRackHeight()
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

float CRackDetect::GetRackLeft()
{
    int left = pRackTemp->h_partition_y[0];
    for(int i=1;i<4;i++)
    {
        if(pRackTemp->h_partition_y[i] > left)
        {
            left = pRackTemp->h_partition_y[i];
        }
    }
    float res = (float)left/100 - 1.0;
    return res;
}

float CRackDetect::GetRackRight()
{
    int right = pRackTemp->h_partition_y[0];
    for(int i=1;i<4;i++)
    {
        if(pRackTemp->h_partition_y[i] < right)
        {
            right = pRackTemp->h_partition_y[i];
        }
    }
    float res = (float)right/100 - 1.0;
    return res;
}

float CRackDetect::GetRackDepth()
{
    int depth = pRackTemp->d_partition_x;
    float res = (float)depth/100 ;
    return res;
}

bool CRackDetect::InSideRange(float inY, float inDist)
{
    bool res = false;
    for(int i=0;i<4;i++)
    {
        float side_y = (float)(pRackTemp->h_partition_y[i])/100 - 1.0;
        if(abs(inY - side_y) < (inDist - 0.01))
        {
            res = true;
            break;
        }
    }
    return res;
}

bool CRackDetect::inObjRange(float inX, float inY, float inZ, float inDist)
{
    bool res = false;
    int nNumObj = arObject.size();
    for(int i=0;i<nNumObj;i++)
    {
        if(abs(inZ - arObject[i].zMin) < 0.1)
        {
            float obj_x = (arObject[i].xMin + arObject[i].xMax)/2;
            float obj_y = (arObject[i].yMin + arObject[i].yMax)/2;
            float dist = sqrt((obj_x-inX) * (obj_x-inX) + (obj_y-inY)*(obj_y-inY));
            if(dist < (inDist - 0.01))
            {
                res = true;
                break;
            }
        }
    }
    return res;
}

bool CRackDetect::inPlaceRange(float inX, float inY, float inZ, float inDist)
{
    bool res = false;
    int nNumPlace = arPlacePos.size();
    for(int i=0;i<nNumPlace;i++)
    {
        if(abs(inZ - arPlacePos[i].z) < 0.1)
        {
            float dist = sqrt((arPlacePos[i].x-inX) * (arPlacePos[i].x-inX) + (arPlacePos[i].y-inY)*(arPlacePos[i].y-inY));
            if(dist < (inDist - 0.01))
            {
                res = true;
                break;
            }
        }
    }
    return res;
}

void CRackDetect::GetObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSrcCloud)
{
    // 把点云拉到原点
    MovePC(inSrcCloud,-rackPosition.x_offset,-rackPosition.y_offset,-rackPosition.angle,true);

    // 剔除四周
    pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象
    pass.setInputCloud(inSrcCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(0.0, GetRackHeight());
    pass.filter (*inSrcCloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(-GetRackDepth() - 0.05,  -GetRackDepth() + 0.4);
    pass.filter (*inSrcCloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits(GetRackRight() + 0.05 , GetRackLeft() - 0.05);
    pass.filter (*inSrcCloud);

    // 剔除横隔板
    pass.setNegative(true);
    for(int i=0;i<4;i++)
    {
        float lvHeight = (float)(pRackTemp->v_partition_z[i])/100;
        pass.setFilterFieldName ("z");
        pass.setFilterLimits(lvHeight-fCulling_v-0.05, lvHeight+fCulling_v);
        pass.filter (*inSrcCloud);
    }

    // 剔除竖隔板
    for(int i=1;i<4;i++)
    {
        float h_y = (float)(pRackTemp->h_partition_y[i] - 100)/100;
        pass.setFilterFieldName ("y");
        pass.setFilterLimits(h_y-fCulling_h, h_y+fCulling_h);
        pass.filter (*inSrcCloud);
    }

    // 统计物品
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(inSrcCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01); // 1cm
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (inSrcCloud);
    ec.extract (cluster_indices);

    arObject.clear();
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        bool bFirstPoint = true;
        stBoxMarker obj_marker;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (inSrcCloud->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        MovePC(cloud_cluster,rackPosition.x_offset,rackPosition.y_offset,rackPosition.angle,false);
        
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
        float obj_width = abs(obj_marker.yMax-obj_marker.yMin);
        if(obj_marker.zMin > fGrabHeightMin && obj_marker.zMin < fGrabHeightMax && obj_width > 0.02 && obj_width < 0.1)
        {
            // ROS_WARN("obj[%d] x(%.2f , %.2f) y(%.2f , %.2f) z(%.2f , %.2f)",j,obj_marker.xMax,obj_marker.xMin,obj_marker.yMax,obj_marker.yMin,obj_marker.zMax,obj_marker.zMin);
            arObject.push_back(obj_marker);
            j ++;
        }
        ROS_WARN("[CRackDetect::GetObjects]obj[%d] x(%.2f , %.2f) y(%.2f , %.2f) z(%.2f , %.2f)",j,obj_marker.xMax,obj_marker.xMin,obj_marker.yMax,obj_marker.yMin,obj_marker.zMax,obj_marker.zMin);
    }
    int nNumObj = arObject.size();
    //ROS_INFO("nNumObj = %d",nNumObj);

    // 统计空位(逐层扫描)
    cloud_place->points.clear();
    cloud_place->width = cloud_place->points.size();
    cloud_place->height = 1;
    cloud_place->is_dense = false;
    pcl::PointXYZRGB pnt;
    // [1]放在中间
    if(nPlaceMode == PLACE_MIDDLE)
    {
        for(int zi=0;zi<4;zi++)
        {
            float lvHeight =  (float)pRackTemp->v_partition_z[zi]/100;
            if(lvHeight < fGrabHeightMin-fGrabAbovePlane || lvHeight > fGrabHeightMax-fGrabAbovePlane)
                continue;   //超出抓取高度范围的隔层就不考虑了
        
            for(int yi=0;yi<3;yi++)
            {
                float place_x = -fRackDepth/2 + 0.05;
                //找相邻的立柱并求其中间位置
                if(pRackTemp->h_partition_y[yi] == pRackTemp->h_partition_y[yi+1])
                    continue;
                float place_y = (float)(pRackTemp->h_partition_y[yi] + pRackTemp->h_partition_y[yi+1])/(2*100) - 1.0;
                //ROS_INFO("place test (%.2f , %.2f , %.2f)",place_x,place_y,lvHeight);
                while(InSideRange(place_y, fPlaceDistSide) == false )
                {
                    pnt.x = place_x;
                    pnt.y = place_y;
                    pnt.z = lvHeight+fCulling_v;
                    cloud_place->points.push_back(pnt);
                    place_y += fPlaceDistObj;
                }
            }
        }
    }
    // [2]紧凑放置
    if(nPlaceMode == PLACE_COMPACT)
    {
        for(int zi=0;zi<4;zi++)
        {
            float lvHeight =  (float)pRackTemp->v_partition_z[zi]/100;
            if(lvHeight < fGrabHeightMin-fGrabAbovePlane || lvHeight > fGrabHeightMax-fGrabAbovePlane)
            {
                continue;   //超出抓取高度范围的隔层就不考虑了
            }
        
            for(int yi=0;yi<4;yi++)
            {
                float place_x = -fRackDepth/2 + 0.05; 
                float ty = (float)pRackTemp->h_partition_y[yi]/100 - 1.0;
                float place_y = ty + fPlaceDistSide;
                // 避开侧板和中间隔板
                while(InSideRange(place_y, fPlaceDistSide) == false && place_y < fRackLeft)
                {
                    pnt.x = place_x;
                    pnt.y = place_y;
                    pnt.z = lvHeight+fCulling_v;
                    cloud_place->points.push_back(pnt);
                    place_y += fPlaceDistObj;
                }
            }
        }
    }
    //将放置点移动到货架位置
    MovePC(cloud_place,rackPosition.x_offset,rackPosition.y_offset,rackPosition.angle,false);
    arPlacePos.clear();
    int nNumPlace = cloud_place->points.size();
    for(int i=0;i<nNumPlace;i++)
    {
        if(
            inObjRange(cloud_place->points[i].x,cloud_place->points[i].y,cloud_place->points[i].z,fPlaceDistObj) == false &&
            inPlaceRange(cloud_place->points[i].x,cloud_place->points[i].y,cloud_place->points[i].z,fPlaceDistObj) == false
            )
            arPlacePos.push_back(cloud_place->points[i]);
    }
}

void CRackDetect::CloudToTemp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud)
{
    // 模板生成
    int cloud_size = inCloud->points.size();
    for (int i=0;i<cloud_size;i++)
    {
        int index_x = inCloud->points[i].x * 100;
        int index_y = (inCloud->points[i].y + 1.0) * 100;
        int index_z = inCloud->points[i].z * 100;
        if(index_x >= 0 && index_x < RACK_TEMP_X && index_y >= 0 && index_y < RACK_TEMP_Y && index_z >= 0 && index_z < RACK_TEMP_Z)
        {
            pRackTemp->data[index_x][index_y][index_z] = 255;
        }
    }

    // 统计尺寸
    float front = inCloud->points[0].x;
    float back = inCloud->points[0].x;
    float left = inCloud->points[0].y;
    float right = inCloud->points[0].y;
    float top = inCloud->points[0].z;
    float bottom = inCloud->points[0].z;
    for (int i=0;i<cloud_size;i++)
    {
        if(inCloud->points[i].x < front) front = inCloud->points[i].x;
        if(inCloud->points[i].x > back) back = inCloud->points[i].x;
        if(inCloud->points[i].y < right) right = inCloud->points[i].y;
        if(inCloud->points[i].y > left) left = inCloud->points[i].y;
        if(inCloud->points[i].z < bottom) bottom = inCloud->points[i].z;
        if(inCloud->points[i].z > top) top = inCloud->points[i].z;
    }
    pRackTemp->d_partition_x = ((back - front)/2) * 100;
    pRackTemp->h_partition_y[0] = pRackTemp->h_partition_y[1] = right*100 + 100;
    pRackTemp->h_partition_y[2] = pRackTemp->h_partition_y[3] = left*100 + 100;

    pRackTemp->v_partition_z[0] = pRackTemp->v_partition_z[1] = bottom*100;
    pRackTemp->v_partition_z[2] = pRackTemp->v_partition_z[3] = top*100;
}

void CRackDetect::ScaleTemp()
{
    //统计一下一共有多少个点
    int mNumOfPoints = 0;
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                if(pRackTemp->data[x][y][z] == 255)
                    mNumOfPoints ++;
            }
    int nPointIndex = 0;
    // 初始化单个渐变模板
    InitScalePoint();
    // 对所有点距离渐变
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                if(pRackTemp->data[x][y][z] == 255)
                {
                    ScalePoint(x,y,z);
                    nPointIndex ++;
                    float perc = (float)nPointIndex*100/mNumOfPoints;
                    ROS_INFO("%.0f%% -- %d / %d -- (%d, %d, %d)",perc,nPointIndex,mNumOfPoints,x,y,z);
                }
            }
}