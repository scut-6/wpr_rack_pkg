#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream> //标准输入/输出
#include "StorageRack.h"
#include "MobileRack.h"
#include "PalletRack.h"
#include "BoxObject.h"

#define STORAGE_RACK    1
#define PALLET_RACK     2
#define MOBILE_RACK     3
#define BOX_OBJECT      4
static int nRackType = MOBILE_RACK;

static std::string pc_topic;
static ros::Publisher marker_pub;
static tf::TransformBroadcaster *rack_pos_tf;

static visualization_msgs::Marker line_rack;
static ros::Publisher rack_temp;

static CStorageRack storageRack;
static CMobileRack mobileRack;
static CPalletRack palletRack;
static CBoxObject boxObject;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_storage(new pcl::PointCloud<pcl::PointXYZRGB>);    //格子货架模板
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_pallet(new pcl::PointCloud<pcl::PointXYZRGB>);    //仓储货架模板
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_mobile(new pcl::PointCloud<pcl::PointXYZRGB>);    //移动货架模板
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_box(new pcl::PointCloud<pcl::PointXYZRGB>);    //料盒模板

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //ROS_INFO("ProcCloudCB");
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
        float height = storageRack.GetRackHeight();
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
    marker_pub.publish(line_rack);
}

void RemoveRackPos()
{
    line_rack.action = 3;
    line_rack.points.clear();
    marker_pub.publish(line_rack);
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "/rack_calibration_node");
    ROS_WARN("rack_calibration_node start!");
    rack_pos_tf = new tf::TransformBroadcaster();

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/sd/points");

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 10 , ProcCloudCB);

    rack_temp = nh.advertise<sensor_msgs::PointCloud2> ("/rack/rack_temp",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/rack/rack_marker", 10);
    
    //读取模板
    storageRack.LoadTemp("/home/master/storage_rack.tmp");
    mobileRack.LoadTemp("/home/master/mobile_rack.tmp");
    palletRack.LoadTemp("/home/master/pallet_rack.tmp");
    boxObject.LoadTemp("/home/master/box_70.tmp");
    
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
    //std::cout << "There are " << cloud_temp_storage->points.size () << " points in rankTemp" << std::endl;
    cloud_temp_mobile->width = (int) cloud_temp_mobile->points.size();
    cloud_temp_mobile->height = 1;
    cloud_temp_mobile->header.frame_id = "rack_pos";
    std::cout << "There are " << cloud_temp_mobile->points.size () << " points in mobileRack" << std::endl;
    cloud_temp_pallet->width = (int) cloud_temp_pallet->points.size();
    cloud_temp_pallet->height = 1;
    cloud_temp_pallet->header.frame_id = "rack_pos";
    //std::cout << "There are " << cloud_temp_pallet->points.size () << " points in palletRack" << std::endl;
    cloud_temp_box->width = (int) cloud_temp_box->points.size();
    cloud_temp_box->height = 1;
    cloud_temp_box->header.frame_id = "rack_pos";
    // pcl::PointXYZRGB new_point;
    // for(int x=0;x<RACK_TEMP_X;x++)
    //     for(int y=0;y<RACK_TEMP_Y;y++)
    //         for(int z=0;z<RACK_TEMP_Z;z++)
    //         {
    //             //if(rackDetect.pRackTemp->data[x][y][z] > 0)     //显示所有点
    //             if(storageRack.pRackTemp->data[x][y][z] == 255)    //只显示模板
    //             {
    //                 new_point.x = (float)x/100 - 1.0; //模板是2米×2米，将其拉到中间
    //                 new_point.y = (float)y/100 - 1.0; //模板是2米×2米，将其拉到中间
    //                 new_point.z = (float)z/100;
    //                 new_point.g = storageRack.pRackTemp->data[x][y][z];
    //                 cloud_temp->points.push_back(new_point);
    //             }
    //         }
    // cloud_temp->width = (int) cloud_temp->points.size();
    // cloud_temp->height = 1;
    // cloud_temp->header.frame_id = "rack_pos";
    
    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
        // 显示货架位置
        static tf::Transform transform_rack;
        transform_rack.setOrigin( tf::Vector3(1.0, 0, 0) );
        tf::Quaternion quat;
        // 目标姿态,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
        quat.setRPY(0.0, 0.0, 0);
        // 将欧拉角旋转量转换成四元数表达
        transform_rack.setRotation(quat);
        rack_pos_tf->sendTransform(tf::StampedTransform(transform_rack, ros::Time::now(), "base_footprint", "rack_pos"));
        switch(nRackType)
        {
            case STORAGE_RACK:
                DrawRackPos(storageRack.pRackTemp,&(storageRack.rackPosition));
                rack_temp.publish(cloud_temp_storage);
                break; 
            case PALLET_RACK:
                DrawRackPos(palletRack.pRackTemp,&(palletRack.rackPosition));
                rack_temp.publish(cloud_temp_pallet);
                break;
            case MOBILE_RACK:
                DrawRackPos(mobileRack.pRackTemp,&(mobileRack.rackPosition));
                rack_temp.publish(cloud_temp_mobile);
                break;
            case BOX_OBJECT:
                DrawRackPos(boxObject.pRackTemp,&(boxObject.rackPosition));
                rack_temp.publish(cloud_temp_box);
                break;
        }
    }

    delete rack_pos_tf;

    return 0;
}