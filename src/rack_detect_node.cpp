#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream> //标准输入/输出
#include <pcl/io/pcd_io.h> //pcd文件输入/输出
#include <pcl/point_types.h> //各种点类型
#include <pcl/registration/icp.h> //ICP(iterative closest point)配准
#include <wpr_rack_pkg/ObjectBox.h>
#include <geometry_msgs/PoseArray.h>
#include "StorageRack.h"
#include "MobileRack.h"
#include "PalletRack.h"
#include "BoxObject.h"

using namespace std;

static std::string pc_topic;
static ros::Publisher obj_pc_pub;
static ros::Publisher rack_marker_pub;
static ros::Publisher obj_marker_pub;
static ros::Publisher place_marker_pub;
static ros::Publisher rack_pos_pub;
static ros::Publisher obj_pos_pub;
static ros::Publisher place_pos_pub;
static tf::TransformListener *tf_listener; 
static tf::TransformBroadcaster *rack_pos_tf;
void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();
void SortObjects();
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_rack;
static visualization_msgs::Marker line_grab;
static visualization_msgs::MarkerArray arrow_place;
static visualization_msgs::Marker text_marker;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static ros::Publisher segmented_objects;
static ros::Publisher rack_temp;

static CRackDetect* pRackDetect;
static CStorageRack storageRack;
static CMobileRack mobileRack;
static CPalletRack palletRack;
static CBoxObject boxObject;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_storage(new pcl::PointCloud<pcl::PointXYZRGB>);    //格子货架模板
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_pallet(new pcl::PointCloud<pcl::PointXYZRGB>);    //仓储货架模板
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_mobile(new pcl::PointCloud<pcl::PointXYZRGB>);    //移动货架模板
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_box(new pcl::PointCloud<pcl::PointXYZRGB>);    //料盒模板
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);     //输入点云
static bool bPointCloudUpdate = false;
static bool bStopCloud = true;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZRGB>);     //用于迭代计算的点云
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr move_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr rot_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

#define MODE_RACK_DETECT    1
#define MODE_OBJ_DETECT     2
static int nMode = MODE_RACK_DETECT;
static bool bGot = false;

#define STORAGE_RACK    1
#define PALLET_RACK     2
#define MOBILE_RACK     3
#define BOX_OBJECT      4
static int nRackType = STORAGE_RACK;

static float filter_x_min = 0;
static float filter_x_max = 2.0;
static float filter_y_min = -0.5; //为Mani缩减了搜索范围
static float filter_y_max = 0.5; //为Mani缩减了搜索范围
static float filter_z_min = 0;
static float filter_z_max = 1.0;
static float sample_dist = 0.05f;

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //ROS_INFO("ProcCloudCB");
    if(bStopCloud == true || bPointCloudUpdate == true)
        return;
    //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
    if(res == false)
    {
        return;
    }
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    ROS_WARN("[ProcCloudCB]原云的点数 = %d",(int)(cloud_src.size())); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
    cloud_source_ptr = cloud_src.makeShared(); 

    if(nMode == MODE_RACK_DETECT)
    {
        pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象
        pass.setInputCloud (cloud_source_ptr);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (filter_z_min, filter_z_max);
        pass.filter (*cloud_source_ptr);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (filter_x_min, filter_x_max);
        pass.filter (*cloud_source_ptr);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (filter_y_min, filter_y_max);
        pass.filter (*cloud_source_ptr);

        int nNumCloudSrc = cloud_source_ptr->points.size();
        ROS_WARN("滤波之后点数=  %d",nNumCloudSrc);

        ////////////////////////////////////
        //  kinetic / Ubuntu 16.04
        // Uniform sampling object.
        // pcl::UniformSampling<pcl::PointXYZRGB> filter;
        // filter.setInputCloud(cloud_source_ptr);
        // filter.setRadiusSearch(sample_dist);
        // We need an additional object to store the indices of surviving points.
        // pcl::PointCloud<int> keypointIndices;
        // filter.compute(keypointIndices);
        // pcl::copyPointCloud(*cloud_source_ptr, keypointIndices.points, *cloud_in);
        //////////////////////////////////////////////
        //melodic / Ubuntu 18.04
        pcl::UniformSampling<pcl::PointXYZRGB> filter;
        filter.setInputCloud(cloud_source_ptr);
        filter.setRadiusSearch(sample_dist);
        filter.filter (*cloud_in);
        //////////////////////////////////////////////
    }

    if(nMode == MODE_OBJ_DETECT)
    {
        pcl::copyPointCloud(*cloud_source_ptr, *cloud_in);
    }
    
    bPointCloudUpdate = true;
    ROS_WARN("bPointCloudUpdate = true 新的点云数据");
}

void CommandCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("detect storage rack");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[rack_detect_node] detect storage rack - 开始检测storage货架");
        nMode = MODE_RACK_DETECT;
        nRackType = STORAGE_RACK;
        pRackDetect = &storageRack;
        filter_x_min = 0;
        filter_x_max = 2.0;
        filter_y_min = -1.5;
        filter_y_max = 1.5;
        filter_z_min = 0;
        filter_z_max = 2.0;
        sample_dist = 0.05f;
        bGot = false;
        bStopCloud = false;
    }

    nFindIndex = msg->data.find("detect pallet rack");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[rack_detect_node] detect pallet rack - 开始检测pallet货架");
        nMode = MODE_RACK_DETECT;
        nRackType = PALLET_RACK;
        pRackDetect = &palletRack;
        filter_x_min = 0;
        filter_x_max = 2.0;
        filter_y_min = -1.5;
        filter_y_max = 1.5;
        filter_z_min = 0;
        filter_z_max = 1.0;
        sample_dist = 0.03f;
        bGot = false;
        bPointCloudUpdate = false;
        bStopCloud = false;
    }

    nFindIndex = msg->data.find("detect mobile rack");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[rack_detect_node] detect mobile rack - 开始检测mobile货架");
        nMode = MODE_RACK_DETECT;
        nRackType = MOBILE_RACK;
        pRackDetect = &mobileRack;
        filter_x_min = 0;
        filter_x_max = 2.0;
        filter_y_min = -1.5;
        filter_y_max = 1.5;
        filter_z_min = 0.7;
        filter_z_max = 2.0;
        sample_dist = 0.03f;
        bGot = false;
        bPointCloudUpdate = false;
        bStopCloud = false;
    }

    // nFindIndex = msg->data.find("detect box");
    // if( nFindIndex >= 0 )
    // {
    //     ROS_WARN("[rack_detect_node] detect box");
    //     nMode = MODE_RACK_DETECT;
    //     nRackType = BOX_OBJECT;
    //     pRackDetect = &boxObject;
    //     filter_x_min = 0;
    //     filter_x_max = 1.5;
    //     filter_y_min = -0.5;
    //     filter_y_max = 0.5;
    //     filter_z_min = 0.73;
    //     filter_z_max = 1.2;
    //     sample_dist = 0.005f;
    //     bGot = false;
    //     bStopCloud = false;
    // }

    nFindIndex = msg->data.find("detect object");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[rack_detect_node] detect object - 开始检测货架上的物品");
        nMode = MODE_OBJ_DETECT;
        bPointCloudUpdate = false;
        bStopCloud = false;
    }
}

void RackTF()
{
    static tf::Transform transform_rack;
    transform_rack.setOrigin( tf::Vector3(pRackDetect->rackPosition.x_offset, pRackDetect->rackPosition.y_offset, 0) );
    tf::Quaternion quat;
    // 目标姿态,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
    quat.setRPY(0.0, 0.0, pRackDetect->rackPosition.angle);
    // 将欧拉角旋转量转换成四元数表达
    transform_rack.setRotation(quat);
    rack_pos_tf->sendTransform(tf::StampedTransform(transform_rack, ros::Time::now(), "base_footprint", "rack_pos"));
}

void DrawRackPos(stRackTemp* inRack, stICPResult* inPos)
{
    line_rack.header.frame_id = "rack_pos";
    line_rack.ns = "line_rack";
    line_rack.action = visualization_msgs::Marker::ADD;
    line_rack.id = 1;
    line_rack.type = visualization_msgs::Marker::LINE_LIST;
    line_rack.scale.x = 0.005;
    line_rack.color.r = 1.0;
    line_rack.color.g = 1.0;
    line_rack.color.b = 0;
    line_rack.color.a = 1.0;

    // 找左右边界
    int nLeft = inRack->h_partition_y[0];
    for(int i=1;i<4;i++)
    {
        if(inRack->h_partition_y[i] > nLeft)
        {
            nLeft = inRack->h_partition_y[i];
        }
    }
    int nRight = inRack->h_partition_y[0];
    for(int i=1;i<4;i++)
    {
        if(inRack->h_partition_y[i] < nRight)
        {
            nRight = inRack->h_partition_y[i];
        }
    }
    float left = (float) nLeft / 100 - 1.0;
    float right = (float) nRight / 100 - 1.0;
    // 找前后边界
    float depth = (float)inRack->d_partition_x / 100;
    // 先画层
    for(int i=0;i<4;i++)
    {
        float lvHeight = (float)inRack->v_partition_z[i] / 100;
        geometry_msgs::Point p;
        p.z = lvHeight;
        p.x = -depth/2; p.y = left + 0.3; line_rack.points.push_back(p);
        p.x = -depth/2; p.y = right - 0.3; line_rack.points.push_back(p);
        p.x = -depth/2; p.y = right - 0.3; line_rack.points.push_back(p);
        p.x = depth/2; p.y = right - 0.3; line_rack.points.push_back(p);
        p.x = depth/2; p.y = right - 0.3; line_rack.points.push_back(p);
        p.x = depth/2; p.y = left + 0.3; line_rack.points.push_back(p);
        p.x = depth/2; p.y = left + 0.3; line_rack.points.push_back(p);
        p.x = -depth/2; p.y = left + 0.3; line_rack.points.push_back(p);
    }
    // 绘制侧边栏
    for(int i=0;i<4;i++)
    {
        float y = (float)inRack->h_partition_y[i] / 100 - 1.0;
        float height = pRackDetect->GetRackHeight();
        geometry_msgs::Point p;
        p.y = y;
        p.z = 0; p.x = depth/2; line_rack.points.push_back(p);
        p.z = 0; p.x = -(depth/2); line_rack.points.push_back(p);
        p.z = 0; p.x = -(depth/2); line_rack.points.push_back(p);
        p.z = height+0.3; p.x = -(depth/2); line_rack.points.push_back(p);
        p.z = height+0.3; p.x = -(depth/2); line_rack.points.push_back(p);
        p.z = height+0.3; p.x = depth/2; line_rack.points.push_back(p);
        p.z = height+0.3; p.x = depth/2; line_rack.points.push_back(p);
        p.z = 0; p.x = depth/2; line_rack.points.push_back(p);
    }
    rack_marker_pub.publish(line_rack);
}

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 3;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = inMinZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.z = inMaxZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);
    obj_marker_pub.publish(line_box);
}

void DrawGrabHeight()
{
    // 找左右边界
    float left = pRackDetect->GetRackLeft();
    float right = pRackDetect->GetRackRight();
    // 找前后边界
    float depth = pRackDetect->GetRackDepth();
    // 绘制抓取范围
    line_grab.header.frame_id = "rack_pos";
    line_grab.ns = "line_grab";
    line_grab.action = visualization_msgs::Marker::ADD;
    line_grab.id = 2;
    line_grab.type = visualization_msgs::Marker::LINE_LIST;
    line_grab.scale.x = 0.005;
    line_grab.color.r = 0.0;
    line_grab.color.g = 0.0;
    line_grab.color.b = 1.0;
    line_grab.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = pRackDetect->fGrabHeightMax;
    p.x = -depth/2; p.y = left + 0.2; line_grab.points.push_back(p);
    p.x = -depth/2; p.y = right - 0.2; line_grab.points.push_back(p);
    p.x = -depth/2; p.y = right - 0.2; line_grab.points.push_back(p);
    p.x = depth/2; p.y = right - 0.2; line_grab.points.push_back(p);
    p.x = depth/2; p.y = right - 0.2; line_grab.points.push_back(p);
    p.x = depth/2; p.y = left + 0.2; line_grab.points.push_back(p);
    p.x = depth/2; p.y = left + 0.2; line_grab.points.push_back(p);
    p.x = -depth/2; p.y = left + 0.2; line_grab.points.push_back(p);

    p.z = pRackDetect->fGrabHeightMin;
    p.x = -depth/2; p.y = left + 0.2; line_grab.points.push_back(p);
    p.x = -depth/2; p.y = right - 0.2; line_grab.points.push_back(p);
    p.x = -depth/2; p.y = right - 0.2; line_grab.points.push_back(p);
    p.x = depth/2; p.y = right - 0.2; line_grab.points.push_back(p);
    p.x = depth/2; p.y = right - 0.2; line_grab.points.push_back(p);
    p.x = depth/2; p.y = left + 0.2; line_grab.points.push_back(p);
    p.x = depth/2; p.y = left + 0.2; line_grab.points.push_back(p);
    p.x = -depth/2; p.y = left + 0.2; line_grab.points.push_back(p);
    obj_marker_pub.publish(line_grab);
}

static int nTextNum = 2;
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = nTextNum;
    nTextNum ++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    obj_marker_pub.publish(text_marker);
}

void DrawPlacePos()
{
    arrow_place.markers.clear();
    int nNumPlace = pRackDetect->arPlacePos.size();
    ROS_INFO("可放置位置数量 nNumPlace = %d",nNumPlace);
    visualization_msgs::Marker arrow;
    for(int i=0;i<nNumPlace;i++)
    {
        arrow.header.frame_id = "base_footprint";
        arrow.ns = "arrow_place";
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.id = i+1;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.scale.x = 0.12;
        arrow.scale.y = 0.05;
        arrow.scale.z = 0.05;
        arrow.color.r = 1.0;
        arrow.color.g = 0;
        arrow.color.b = 1.0;
        arrow.color.a = 1.0;
        arrow.pose.position.x = pRackDetect->arPlacePos[i].x;
        arrow.pose.position.y = pRackDetect->arPlacePos[i].y;
        arrow.pose.position.z = pRackDetect->arPlacePos[i].z + 0.12;
        arrow.pose.orientation.x = 0;
        arrow.pose.orientation.y = 0.71;
        arrow.pose.orientation.z = 0;
        arrow.pose.orientation.w = 0.71;
        // printf("arrow (%.2f , %.2f , %.2f , %.2f)\n",arrow.pose.orientation.x,arrow.pose.orientation.y,arrow.pose.orientation.z,arrow.pose.orientation.w);
        arrow_place.markers.push_back(arrow);
        // geometry_msgs::Point p;
        // p.x = pRackDetect->arPlacePos[i].x; p.y = pRackDetect->arPlacePos[i].y; p.z = pRackDetect->arPlacePos[i].z; line_place.points.push_back(p);
        // p.z += 0.2; arrow_place.points.push_back(p);
        printf("放置位置 (%.2f , %.2f , %.2f)\n",arrow.pose.position.x,arrow.pose.position.y,arrow.pose.position.z);
    }
    place_marker_pub.publish(arrow_place);
}

void RemoveBoxes()
{
    line_box.action = 3;
    line_box.points.clear();
    obj_marker_pub.publish(line_box);
    text_marker.action = 3;
    obj_marker_pub.publish(text_marker);
    line_grab.action = 3;
    line_grab.points.clear();
    obj_marker_pub.publish(line_grab);
}
void RemoveRackPos()
{
    line_rack.action = 3;
    line_rack.points.clear();
    rack_marker_pub.publish(line_rack);
}

// 检测传送带料盒
void ColorBoxCB(const wpr_rack_pkg::ObjectBox::ConstPtr &msg)
{

    int nNumObj = msg->name.size();
    ROS_WARN("[Rack ColorBoxCB] nNumObj = %d name = ( %s )",nNumObj,msg->name[nNumObj-1].c_str());
    if(nNumObj > 0)
    {
        float box_x,box_y,box_z;
        //优先找box_0
        if(nNumObj > 1)
        {
            int nBoxIndex = 0;
            int nFindIndex = 0;
            for(int i=0;i<nNumObj;i++)
            {
                nFindIndex = msg->name[i].find("box_0");
                if( nFindIndex >= 0 )
                {
                    nBoxIndex = i;
                    break;
                }
            }
            box_x = (msg->xMin[nBoxIndex] + msg->xMax[nBoxIndex])/2;
            box_y = (msg->yMin[nBoxIndex] + msg->yMax[nBoxIndex])/2;
            box_z = msg->zMin[nBoxIndex];
        }
        else
        {
            box_x = (msg->xMin[0] + msg->xMax[0])/2;
            box_y = (msg->yMin[0] + msg->yMax[0])/2;
            box_z = msg->zMin[0];
        }
        
        pRackDetect->curResult.x_offset = (box_x - 1.0)*100;
        pRackDetect->curResult.y_offset = box_y *100;
        ROS_WARN("[Rack ColorBoxCB] box( %.1f , %.1f ) ",pRackDetect->curResult.x_offset,pRackDetect->curResult.y_offset);
    }
}

// 盒子的点云
void BoxCloudCB(const sensor_msgs::PointCloud2 &input)
{
    ROS_INFO("[rack_detect_node]BoxCloudCB");
    
    //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
    if(res == false)
    {
        return;
    }
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    ROS_INFO("[BoxCloudCB]cloud_src size = %d",(int)(cloud_src.size())); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
    cloud_source_ptr = cloud_src.makeShared(); 

    pcl::copyPointCloud(*cloud_source_ptr, *cloud_in);

    nMode = MODE_RACK_DETECT;
    nRackType = BOX_OBJECT;
    pRackDetect = &boxObject;
    bGot = false;
    bPointCloudUpdate = true;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "rack_detect_node");
    ROS_WARN("rack_detect_node start!");
    tf_listener = new tf::TransformListener();
    rack_pos_tf = new tf::TransformBroadcaster();

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/sd/points");

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 2 , ProcCloudCB);
    ros::Subscriber sub_sr = nh.subscribe("/rack/command", 1, CommandCB);

    rack_temp = nh.advertise<sensor_msgs::PointCloud2> ("/rack/rack_temp",1);
    ros::Publisher rack_cld_in = nh.advertise<sensor_msgs::PointCloud2> ("/rack/cloud_in",1);
    rack_marker_pub = nh.advertise<visualization_msgs::Marker>("/rack/rack_marker", 10);
    obj_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/rack/obj_pc",1);
    obj_marker_pub = nh.advertise<visualization_msgs::Marker>("/rack/obj_marker", 10);
    place_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/rack/place_marker", 10);

    rack_pos_pub = nh.advertise<geometry_msgs::Pose>("/rack/rack_position", 10);
    obj_pos_pub = nh.advertise<wpr_rack_pkg::ObjectBox>("/rack/obj_position", 10);
    place_pos_pub = nh.advertise<geometry_msgs::PoseArray>("/rack/place_positon", 10);
    ros::Subscriber color_box_sub = nh.subscribe("/box/result", 1, ColorBoxCB);
    ros::Subscriber box_cloud_sub = nh.subscribe("/box/cloud_in", 1, BoxCloudCB);
    
    //读取模板
    std::string strHomePath,strLoadFile;
    char const* home = getenv("HOME");
    strHomePath = home;
    strLoadFile = strHomePath + "/storage_rack.tmp";
    storageRack.LoadTemp(strLoadFile.c_str());
    strLoadFile = strHomePath + "/mobile_rack.tmp";
    mobileRack.LoadTemp(strLoadFile.c_str());
    strLoadFile = strHomePath + "/pallet_rack.tmp";
    palletRack.LoadTemp(strLoadFile.c_str());
    strLoadFile = strHomePath + "/box.tmp";
    boxObject.LoadTemp(strLoadFile.c_str());

    // 初始化
    nRackType = PALLET_RACK;
    pRackDetect = &palletRack;
    
    //将模板赋值到点云里显示
    pcl::PointXYZRGB new_point;
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                //if(storageRack.pRackTemp->data[x][y][z] > 0)     //显示所有点（巨卡……）
                if(storageRack.pRackTemp->data[x][y][z] == 255)    //只显示模板
                {
                    new_point.x = (float)x/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.y = (float)y/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.z = (float)z/100;
                    new_point.g = storageRack.pRackTemp->data[x][y][z];
                    cloud_temp_storage->points.push_back(new_point);
                }
                if(mobileRack.pRackTemp->data[x][y][z] == 255)    //只显示模板
                {
                    new_point.x = (float)x/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.y = (float)y/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.z = (float)z/100;
                    new_point.g = mobileRack.pRackTemp->data[x][y][z];
                    cloud_temp_mobile->points.push_back(new_point);
                }
                if(palletRack.pRackTemp->data[x][y][z] == 255)    //只显示模板
                {
                    new_point.x = (float)x/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.y = (float)y/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.z = (float)z/100;
                    new_point.g = palletRack.pRackTemp->data[x][y][z];
                    cloud_temp_pallet->points.push_back(new_point);
                }
                if(boxObject.pRackTemp->data[x][y][z] == 255)    //只显示模板
                {
                    new_point.x = (float)x/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.y = (float)y/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.z = (float)z/100;
                    new_point.g = boxObject.pRackTemp->data[x][y][z];
                    cloud_temp_box->points.push_back(new_point);
                }
            }
    cloud_temp_storage->width = (int) cloud_temp_storage->points.size();
    cloud_temp_storage->height = 1;
    cloud_temp_storage->header.frame_id = "rack_pos";
    std::cout << "storage_rack.tmp 模板文件有 " << cloud_temp_storage->points.size () << " 个点 " << std::endl;
    cloud_temp_mobile->width = (int) cloud_temp_mobile->points.size();
    cloud_temp_mobile->height = 1;
    cloud_temp_mobile->header.frame_id = "rack_pos";
    cloud_temp_pallet->width = (int) cloud_temp_pallet->points.size();
    cloud_temp_pallet->height = 1;
    cloud_temp_pallet->header.frame_id = "rack_pos";
    std::cout << "pallet_rack.tmp 模板文件有 " << cloud_temp_pallet->points.size () << " 个点 " << std::endl;
    cloud_temp_box->width = (int) cloud_temp_box->points.size();
    cloud_temp_box->height = 1;
    cloud_temp_box->header.frame_id = "rack_pos";

    //bStopCloud = false;//直接开始，测试用！！
    bGot = false;
    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
        if(nMode == MODE_RACK_DETECT)
        {
            if(bPointCloudUpdate == true)
            {
                ROS_WARN("nMode == [MODE_RACK_DETECT]");
                int nNumCloudIn = cloud_in->points.size();
                ROS_WARN("cloud_in 点云里有 %d 个点",nNumCloudIn);
                if(nRackType != BOX_OBJECT)
                {
                    //pRackDetect->ICP(cloud_in);
                    pRackDetect->FirstTest(cloud_in);
                    //rack_cld_in.publish(cloud_in);
                }
            
                // 按照初始匹配拉过来
                int x_offset = -pRackDetect->curResult.x_offset;
                int y_offset = -pRackDetect->curResult.y_offset;
                Eigen::Affine3f transform_offset = Eigen::Affine3f::Identity();
                float fx = (float)x_offset/100;
                float fy = (float)y_offset/100;
                transform_offset.translation() << fx, fy, 0.0;
                pcl::transformPointCloud (*cloud_in, *cloud_mid, transform_offset);
                while(ros::ok() && bGot == false)
                {
                    bGot = pRackDetect->SingleTest(pRackDetect->pICPStep, cloud_mid);
                    /////////////////////////////////////////////////
                    // 将stepResult偏移量叠加到 cur_cloud 和 curResult
                    pRackDetect->curResult.angle -= pRackDetect->stepResult.angle;
                    pRackDetect->curResult.x_offset -= pRackDetect->stepResult.x_offset;
                    pRackDetect->curResult.y_offset -= pRackDetect->stepResult.y_offset;
                    //ROS_WARN("[curResult] angle= %.2f offset( %.2f , %.2f )",pRackDetect->curResult.angle,pRackDetect->curResult.x_offset,pRackDetect->curResult.y_offset);

                    // 1、先按照偏移量将点云拉到原点
                    int x_offset = pRackDetect->stepResult.x_offset;
                    int y_offset = pRackDetect->stepResult.y_offset;
                    Eigen::Affine3f transform_offset = Eigen::Affine3f::Identity();
                    float fx = (float)x_offset/100 - 1.0;
                    float fy = (float)y_offset/100;
                    transform_offset.translation() << fx, fy, 0.0;
                    pcl::transformPointCloud (*cloud_mid, *move_cloud, transform_offset);
                    
                    // 2、旋转点云
                    Eigen::Affine3f transform_rot = Eigen::Affine3f::Identity();
                    transform_rot.rotate (Eigen::AngleAxisf (pRackDetect->stepResult.angle, Eigen::Vector3f::UnitZ()));
                    pcl::transformPointCloud (*move_cloud, *rot_cloud, transform_rot);

                    // 3、再将点云放回1米处
                    transform_offset = Eigen::Affine3f::Identity();
                    transform_offset.translation() << 1.0, 0.0, 0.0;
                    pcl::transformPointCloud (*rot_cloud, *cloud_mid, transform_offset);
                    /////////////////////////////////////////////////
                    //ROS_WARN(" publish cloud_temp");
                    switch(nRackType)
                    {
                        case STORAGE_RACK:
                            rack_temp.publish(cloud_temp_storage);
                            break; 
                        case PALLET_RACK:
                            rack_temp.publish(cloud_temp_pallet);
                            break;
                        case MOBILE_RACK:
                            rack_temp.publish(cloud_temp_mobile);
                            break;
                        case BOX_OBJECT:
                            rack_temp.publish(cloud_temp_box);
                            break;
                    }
                    rack_cld_in.publish(cloud_mid);
                    pRackDetect->CalRackPosition();
                    // 显示货架位置
                    RackTF();
                    DrawRackPos(pRackDetect->pRackTemp,&(pRackDetect->rackPosition));
                }
                if(bGot == true)
                {
                    pRackDetect->CalRackPosition();
                    ROS_WARN("[货架检测结果 RackPos] angle= %.2f pos( %.2f , %.2f )",pRackDetect->rackPosition.angle,pRackDetect->rackPosition.x_offset,pRackDetect->rackPosition.y_offset);
                    switch(nRackType)
                    {
                        case STORAGE_RACK:
                            rack_temp.publish(cloud_temp_storage);
                            break; 
                        case PALLET_RACK:
                            rack_temp.publish(cloud_temp_pallet);
                            break;
                        case MOBILE_RACK:
                            rack_temp.publish(cloud_temp_mobile);
                            break;
                        case BOX_OBJECT:
                            rack_temp.publish(cloud_temp_box);
                            break;
                    }
                    rack_cld_in.publish(cloud_mid);
                    // 发布货架结果
                    geometry_msgs::Pose rack_pose_msg;
                    rack_pose_msg.position.x = pRackDetect->rackPosition.x_offset;
                    rack_pose_msg.position.y = pRackDetect->rackPosition.y_offset;
                    rack_pose_msg.position.z= 0;
                    rack_pose_msg.orientation = tf::createQuaternionMsgFromYaw(pRackDetect->rackPosition.angle);
                    rack_pos_pub.publish(rack_pose_msg);
                    //nMode = MODE_OBJ_DETECT;    //马上切换到物品检测，测试用！
                }
                bPointCloudUpdate = false;
                bStopCloud = true;   // 让点云回调函数不接收Kinect新的点云
                ros::spinOnce();
                continue;
            }
        }

        if(nMode == MODE_OBJ_DETECT)
        {
            //ROS_WARN("nMode == [MODE_OBJ_DETECT]");
            if(bPointCloudUpdate == true)
            {
                pRackDetect->GetObjects(cloud_in);
                obj_pc_pub.publish(cloud_in);  // 显示物品点云
                RemoveBoxes();
                int nNumObj = pRackDetect->arObject.size();
                ROS_WARN("检测到 %d 个物品", nNumObj);
                for(int i=0;i<nNumObj;i++)
                {
                    DrawBox(pRackDetect->arObject[i].xMin,pRackDetect->arObject[i].xMax,pRackDetect->arObject[i].yMin,pRackDetect->arObject[i].yMax,pRackDetect->arObject[i].zMin,pRackDetect->arObject[i].zMax,1.0,0,0);
                }
                RackTF();           //发布货架位置TF
                DrawGrabHeight();   //绘制抓取高度范围
                DrawPlacePos();     //绘制可放位置

                // 发布物品地点结果
                wpr_rack_pkg::ObjectBox obj_box_msg;
                for(int i=0;i<nNumObj;i++)
                {
                    std::ostringstream stringStream;
                    stringStream << "obj_" << i;
                    std::string obj_id = stringStream.str();
                    obj_box_msg.name.push_back(obj_id);
                    obj_box_msg.xMax.push_back(pRackDetect->arObject[i].xMax);
                    obj_box_msg.xMin.push_back(pRackDetect->arObject[i].xMin);
                    obj_box_msg.yMax.push_back(pRackDetect->arObject[i].yMax);
                    obj_box_msg.yMin.push_back(pRackDetect->arObject[i].yMin);
                    obj_box_msg.zMax.push_back(pRackDetect->arObject[i].zMax);
                    obj_box_msg.zMin.push_back(pRackDetect->arObject[i].zMin);
                }
                obj_pos_pub.publish(obj_box_msg);
                // 发布放置地点结果
                geometry_msgs::PoseArray place_pose_msg;
                geometry_msgs::Pose pose;
                int nNumPlace = pRackDetect->arPlacePos.size();
                for(int i=0;i<nNumPlace;i++)
                {
                    pose.position.x = pRackDetect->arPlacePos[i].x;
                    pose.position.y = pRackDetect->arPlacePos[i].y;
                    pose.position.z = pRackDetect->arPlacePos[i].z;
                    place_pose_msg.poses.push_back(pose);
                }
                place_pos_pub.publish(place_pose_msg);
                bPointCloudUpdate = false;
                bStopCloud = true;
            }
        }
    }

    delete tf_listener;
    delete rack_pos_tf;

    return 0;

}