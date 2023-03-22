
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/highgui/highgui.hpp> 
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <pcl/io/pcd_io.h> //pcd文件输入/输出
#include <pcl/point_types.h> //各种点类型
#include "StorageRack.h"
#include "MobileRack.h"
#include "PalletRack.h"
#include "BoxObject.h"

using namespace std;

static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static tf::TransformListener *tf_listener; 
void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();
void SortObjects();
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_follow;
static visualization_msgs::Marker text_marker;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static ros::Publisher segmented_objects;
static ros::Publisher rack_temp;

static CStorageRack storageRack;
static CMobileRack mobileRack;
static CPalletRack palletRack;
static CBoxObject boxObject;

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 0;
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
    marker_pub.publish(line_box);
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

    marker_pub.publish(text_marker);
}

void RemoveBoxes()
{
    line_box.action = 3;
    line_box.points.clear();
    marker_pub.publish(line_box);
    line_follow.action = 3;
    line_follow.points.clear();
    marker_pub.publish(line_follow);
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "template_build_node");
    ROS_WARN("template_build_node start!");
    tf_listener = new tf::TransformListener(); 

    ros::NodeHandle nh;
    rack_temp = nh.advertise<sensor_msgs::PointCloud2> ("/rack/rack_temp",1);
    CRackDetect* pRackDetect = NULL;
 
    std::string strLoadFile;
    char const* home = getenv("HOME");
    strLoadFile = home;

    //////////////////////////////////////////////////////////////////////////
    //[1-购物货架]生成模板
    // storageRack.pRackTemp->d_partition_x = 40;

    // storageRack.pRackTemp->h_partition_y[0] = -50;
    // storageRack.pRackTemp->h_partition_y[1] = -50;
    // storageRack.pRackTemp->h_partition_y[2] = 50;
    // storageRack.pRackTemp->h_partition_y[3] = 50;

    // storageRack.pRackTemp->v_partition_z[0] = 53;
    // storageRack.pRackTemp->v_partition_z[1] = 90;
    // storageRack.pRackTemp->v_partition_z[2] = 124;
    // storageRack.pRackTemp->v_partition_z[3] = 163;

    // storageRack.pRackTemp->b_back = 0;

    // storageRack.GenTemp(true);
    // pRackDetect = &storageRack;
    // strLoadFile += "/storage_rack.tmp";
    // //保存
    // storageRack.SaveTemp(strLoadFile.c_str());
    /////////////////////////////////////////////
    //[1-购物货架]读取
    //strLoadFile += "/storage_rack.tmp";
    //storageRack.LoadTemp(strLoadFile.c_str());
    //pRackDetect = &storageRack;
    ///////////////////////////////////////////////////////////////////////////
    //[2-移动载物架]生成模板
    // mobileRack.pRackTemp->d_partition_x = 45;

    // mobileRack.pRackTemp->h_partition_y[0] = -25;
    // mobileRack.pRackTemp->h_partition_y[1] = -25;
    // mobileRack.pRackTemp->h_partition_y[2] = 25;
    // mobileRack.pRackTemp->h_partition_y[3] = 25;

    // mobileRack.pRackTemp->v_partition_z[0] = 45;
    // mobileRack.pRackTemp->v_partition_z[1] = 85;
    // mobileRack.pRackTemp->v_partition_z[2] = 125;
    // mobileRack.pRackTemp->v_partition_z[3] = 125;

    // mobileRack.pRackTemp->b_back = 1;

    // mobileRack.GenTemp(true);
    // pRackDetect = &mobileRack;
    // strLoadFile += "/mobile_rack.tmp";
    // //保存
    // mobileRack.SaveTemp(strLoadFile.c_str());
    /////////////////////////////////////////////
    //[2-移动载物架]读取
    // strLoadFile += "/mobile_rack.tmp";
    // mobileRack.LoadTemp(strLoadFile.c_str());
    // pRackDetect = &mobileRack;
    ///////////////////////////////////////////////////////////////////////////
    //[3-小型货架]生成模板
    palletRack.pRackTemp->d_partition_x = 34;

    palletRack.pRackTemp->h_partition_y[0] = -27;
    palletRack.pRackTemp->h_partition_y[1] = -27;
    palletRack.pRackTemp->h_partition_y[2] = 27;
    palletRack.pRackTemp->h_partition_y[3] = 27;

    palletRack.pRackTemp->v_partition_z[0] = 8;
    palletRack.pRackTemp->v_partition_z[1] = 23;
    palletRack.pRackTemp->v_partition_z[2] = 39;
    palletRack.pRackTemp->v_partition_z[3] = 60;

    palletRack.GenTemp(true);
    pRackDetect = &palletRack;
    strLoadFile += "/pallet_rack.tmp";
    //保存
    palletRack.SaveTemp(strLoadFile.c_str());
    /////////////////////////////////////////////
    //[3-小型货架]读取
    //strLoadFile += "/pallet_rack.tmp";
    //palletRack.LoadTemp(strLoadFile.c_str());
    //pRackDetect = &palletRack;
    ///////////////////////////////////////////////////////////////////////////
    //[4-方形物料盒子]生成模板
    // boxObject.GenTemp();
    // strLoadFile += "/box_70.tmp";
    // //保存
    // boxObject.SaveTemp(strLoadFile.c_str());
    // pRackDetect = &boxObject;
    /////////////////////////////////////////////
    //[4-方形物料盒子]读取
    // strLoadFile += "/box.tmp";
    // boxObject.LoadTemp(strLoadFile.c_str());
    // pRackDetect = &boxObject;
    ///////////////////////////////////////////////////////////////////////////

    //创建点云指针
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>); //创建输入点云（指针）

    //将模板赋值到点云里显示
    pcl::PointXYZRGB new_point;
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                //if(pRackDetect->pRackTemp->data[x][y][z] > 0)     //显示所有点
                if(pRackDetect->pRackTemp->data[x][y][z] == 255)    //只显示模板
                {
                    new_point.x = (float)x/100;
                    new_point.y = (float)y/100;
                    new_point.z = (float)z/100;
                    new_point.g = pRackDetect->pRackTemp->data[x][y][z];
                    cloud_in->points.push_back(new_point);
                }
            }
    cloud_in->width = (int) cloud_in->points.size();
    cloud_in->height = 1;
    cloud_in->header.frame_id = "base_footprint";
    std::cout << "There are " << cloud_in->points.size () << " points in rankTemp" << std::endl;
    //打印点云cloud_in中所有点的坐标信息
    // for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
    //     cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;


    ros::Rate r(10);
    while(ros::ok())
    {
        //ROS_WARN(" publish cloud_in");
        rack_temp.publish(cloud_in);
        ros::spinOnce();
    }

    delete tf_listener; 

    return 0;

}