#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream> //标准输入/输出
#include "BoxObject.h"

static float box_bottom = 0.83;//0.81;
static float box_top = box_bottom + 0.06;
static float box_center_x = 1.0;
static float box_length = 0.26;
static float box_width = 0.16;

static std::string pc_topic;
static ros::Publisher marker_pub;
static tf::TransformBroadcaster *rack_pos_tf;
static tf::TransformListener *tf_listener; 
static ros::Publisher rack_cld_in;
static bool bSaveTemp = false;
static int nCaptureCount = 0;

static visualization_msgs::Marker line_box;
static ros::Publisher rack_temp;

static CBoxObject boxObject;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_box(new pcl::PointCloud<pcl::PointXYZRGB>);    //料盒模板

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //ROS_INFO("ProcCloudCB");
    if(bSaveTemp == true)
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
    //ROS_INFO("[ProcCloudCB]cloud_src size = %d",(int)(cloud_src.size())); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
    cloud_source_ptr = cloud_src.makeShared(); 

    // 截取盒子范围的点云
    pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象
    pass.setInputCloud (cloud_source_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (box_bottom, box_top);
    pass.filter (*cloud_source_ptr);
    float front = box_center_x - box_length/2;
    float back = box_center_x + box_length/2;
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (front, back);
    pass.filter (*cloud_source_ptr);
    float left = box_width/2;
    float right = -box_width/2;
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (right, left);
    pass.filter (*cloud_source_ptr);

    // 将盒子点云发布出去显示
    rack_cld_in.publish(*cloud_source_ptr);

    boxObject.CloudToTemp(cloud_source_ptr);
}


void CaptrueCtrlCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("save");
    if( nFindIndex >= 0 )
    {
        nCaptureCount = 0;
        bSaveTemp = true;
        ROS_WARN("[box_capture_node] Save Temp !!");
    }

}

void DrawBoxRect()
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 1;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = 1.0;
    line_box.color.g = 1.0;
    line_box.color.b = 0;
    line_box.color.a = 1.0;

    // 找左右边界
    float left = box_width/2;
    float right = -box_width/2;
    // 找前后边界
    float front = box_center_x - box_length/2;
    float back = box_center_x + box_length/2;
    
    // 先画上面一圈
    geometry_msgs::Point p;
    p.z = box_top;
    p.x = front; p.y = left; line_box.points.push_back(p);
    p.x = front; p.y = right; line_box.points.push_back(p);
    p.x = front; p.y = right;  line_box.points.push_back(p);
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.x = front; p.y = left; line_box.points.push_back(p);

    // 画下面一圈
    p.z = box_bottom;
    p.x = front; p.y = left; line_box.points.push_back(p);
    p.x = front; p.y = right; line_box.points.push_back(p);
    p.x = front; p.y = right;  line_box.points.push_back(p);
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.x = front; p.y = left; line_box.points.push_back(p);

    // 画侧面立柱
    p.z = box_bottom;
    p.x = front; p.y = left; line_box.points.push_back(p);
    p.z = box_top;line_box.points.push_back(p);
    p.z = box_bottom;
    p.x = front; p.y = right; line_box.points.push_back(p);
    p.z = box_top;line_box.points.push_back(p);
    p.z = box_bottom;
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.z = box_top;line_box.points.push_back(p);
    p.z = box_bottom;
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.z = box_top;line_box.points.push_back(p);
    
    marker_pub.publish(line_box);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_capture_node");
    ROS_WARN("box_capture_node start!");
    tf_listener = new tf::TransformListener();
    rack_pos_tf = new tf::TransformBroadcaster();

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/qhd/points");

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 10 , ProcCloudCB);

    marker_pub = nh.advertise<visualization_msgs::Marker>("/rack/box_capture", 10);
    rack_cld_in = nh.advertise<sensor_msgs::PointCloud2> ("/rack/cloud_in",1);
    ros::Subscriber ctrl_sub = nh.subscribe("/rack/capture_ctrl", 2, CaptrueCtrlCB);
    
    boxObject.InitShape();
    ros::Rate r(30);
    while(ros::ok())
    {
        DrawBoxRect();
        if(bSaveTemp == true)
        {
            boxObject.ScaleTemp();
            boxObject.SaveTemp("/home/master/box.tmp");
            bSaveTemp = false;
        }
        ros::spinOnce();
        r.sleep();
    }

    delete tf_listener;
    delete rack_pos_tf;

    return 0;
}