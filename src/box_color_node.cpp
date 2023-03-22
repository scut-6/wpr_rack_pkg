#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <wpr_rack_pkg/ObjectBox.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream> //标准输入/输出
#include "BoxColorDetect.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

static float focus_x = 1.0;
static float focus_y = 0.0;
static float focus_z = 0.65;

static float pass_bottom = focus_z ;//- 0.2;
static float pass_top = pass_bottom + 0.4;
static float pass_center_x = focus_x;
static float pass_length = 1.0;
static float pass_width = 1.5;

static std::string pc_topic;
static ros::Publisher box_pub_0;
static ros::Publisher box_pub_1;
static tf::TransformBroadcaster *rack_pos_tf;
static tf::TransformListener *tf_listener; 
static ros::Publisher rack_cld_in;
static bool bDetectBox = false;

static visualization_msgs::Marker line_box;

static CBoxColorDetect boxColorDetect;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_box(new pcl::PointCloud<pcl::PointXYZRGB>);    //料盒模板

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //ROS_INFO("ProcCloudCB");
    if(bDetectBox == false)
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
    pass.setFilterLimits (pass_bottom, pass_top);
    pass.filter (*cloud_source_ptr);
    float front = pass_center_x - pass_length/2;
    float back = pass_center_x + pass_length/2;
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (front, back);
    pass.filter (*cloud_source_ptr);
    float left = pass_width/2;
    float right = -pass_width/2;
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (right, left);
    pass.filter (*cloud_source_ptr);

    boxColorDetect.ColorFilter(cloud_source_ptr);

    // 将盒子点云发布出去显示
    rack_cld_in.publish(*cloud_source_ptr);
    if( boxColorDetect.arBox[0].size()>0 || boxColorDetect.arBox[1].size()>0 )
    {
        bDetectBox = false;
    }
}

void BoxCmdCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("start color");
    if( nFindIndex >= 0 )
    {
        boxColorDetect.arBox[0].clear();
        boxColorDetect.arBox[1].clear();
        bDetectBox = true;
        ROS_WARN("[box_color_node] Start Detect Box !!");
    }

     nFindIndex = msg->data.find("stop color");
    if( nFindIndex >= 0 )
    {
        boxColorDetect.arBox[0].clear();
        boxColorDetect.arBox[1].clear();
        bDetectBox = false;
        ROS_WARN("[box_color_node] Stop ...");
    }
}

void FocusParamsCB(const geometry_msgs::Point32::ConstPtr &msg)
{
    ROS_WARN("[box_color_node] Focus ( %.2f , %.2f , %.2f)",msg->x,msg->y,msg->z);
    pass_bottom = msg->z ;
    pass_top = pass_bottom + 0.4;
    pass_center_x = msg->x;
}

void DrawBoxRect(stBoxMarker* inBox, int inID, ros::Publisher* inPub,float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = inID;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    // 找左右边界
    float left = inBox->yMax;
    float right = inBox->yMin;
    // 找前后边界
    float front = inBox->xMin;
    float back = inBox->xMax;
    // 上下
    float top = inBox->zMax;
    float bottom = inBox->zMin;
    
    line_box.points.clear();
    // 先画上面一圈
    geometry_msgs::Point p;
    p.z = top;
    p.x = front; p.y = left; line_box.points.push_back(p);
    p.x = front; p.y = right; line_box.points.push_back(p);
    p.x = front; p.y = right;  line_box.points.push_back(p);
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.x = front; p.y = left; line_box.points.push_back(p);

    // 画下面一圈
    p.z = bottom;
    p.x = front; p.y = left; line_box.points.push_back(p);
    p.x = front; p.y = right; line_box.points.push_back(p);
    p.x = front; p.y = right;  line_box.points.push_back(p);
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.x = front; p.y = left; line_box.points.push_back(p);

    // 画侧面立柱
    p.z = bottom;
    p.x = front; p.y = left; line_box.points.push_back(p);
    p.z = top;line_box.points.push_back(p);
    p.z = bottom;
    p.x = front; p.y = right; line_box.points.push_back(p);
    p.z = top;line_box.points.push_back(p);
    p.z = bottom;
    p.x = back; p.y = left; line_box.points.push_back(p);
    p.z = top;line_box.points.push_back(p);
    p.z = bottom;
    p.x = back; p.y = right; line_box.points.push_back(p);
    p.z = top;line_box.points.push_back(p);
    
    inPub->publish(line_box);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_color_node");
    ROS_WARN("box_color_node start!");
    tf_listener = new tf::TransformListener();
    rack_pos_tf = new tf::TransformBroadcaster();

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/sd/points");

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 10 , ProcCloudCB);

    box_pub_0 = nh.advertise<visualization_msgs::Marker>("/box/marker_0", 10);
    box_pub_1 = nh.advertise<visualization_msgs::Marker>("/box/marker_1", 10);
    rack_cld_in = nh.advertise<sensor_msgs::PointCloud2> ("/box/cloud_in",1);
    ros::Publisher box_result_pub = nh.advertise<wpr_rack_pkg::ObjectBox> ("/box/result",1);
    ros::Subscriber box_cmd_sub = nh.subscribe("/box/command", 2, BoxCmdCB);
    ros::Subscriber focus_params_sub = nh.subscribe("/box/focus_params", 1, FocusParamsCB);
    
    boxColorDetect.Init();

    ros::Rate r(30);
    while(ros::ok())
    {
        wpr_rack_pkg::ObjectBox box_msg;
        if(boxColorDetect.arBox[0].size()>0)
        {
            int nBoxIndex = boxColorDetect.nIndexBox[0];
            stBoxMarker tmpBox = boxColorDetect.arBox[0][nBoxIndex];
            DrawBoxRect(&(tmpBox),0,&box_pub_0, 1.0,0,1.0 );
            box_msg.name.push_back("box_0");
            box_msg.xMax.push_back(tmpBox.xMax);
            box_msg.xMin.push_back(tmpBox.xMin);
            box_msg.yMax.push_back(tmpBox.yMax);
            box_msg.yMin.push_back(tmpBox.yMin);
            box_msg.zMax.push_back(tmpBox.zMax);
            box_msg.zMin.push_back(tmpBox.zMin);
            box_result_pub.publish(box_msg);
            boxColorDetect.arBox[0].clear();
        }

        if(boxColorDetect.arBox[1].size()>0)
        {
            int nBoxIndex = boxColorDetect.nIndexBox[1];
            stBoxMarker tmpBox = boxColorDetect.arBox[1][nBoxIndex];
            DrawBoxRect(&(tmpBox),1,&box_pub_1, 0.0,0,1.0);
            box_msg.name.push_back("box_1");
            box_msg.xMax.push_back(tmpBox.xMax);
            box_msg.xMin.push_back(tmpBox.xMin);
            box_msg.yMax.push_back(tmpBox.yMax);
            box_msg.yMin.push_back(tmpBox.yMin);
            box_msg.zMax.push_back(tmpBox.zMax);
            box_msg.zMin.push_back(tmpBox.zMin);
            box_result_pub.publish(box_msg);
            boxColorDetect.arBox[1].clear();
        } 
        
        ros::spinOnce();
        r.sleep();
    }

    delete tf_listener;
    delete rack_pos_tf;

    return 0;
}