/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <wpr_rack_pkg/ObjectBox.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream> //标准输入/输出
#include "BoxColorDetect.h"
#include "BoxDim.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <wpr_rack_pkg/BoxParam.h>

static std::string pc_topic;
static ros::Publisher rack_marker_pub;
static ros::Publisher vel_pub;
static ros::Publisher box_pub_0;
static ros::Publisher box_pub_1;
static ros::Publisher box_pose_pub;
static ros::Publisher box_color_pub;
static ros::Publisher place_pub;
static geometry_msgs::Pose place_msg;
static tf::TransformListener *tf_listener; 
static tf::TransformBroadcaster *rack_pos_tf;
static ros::Publisher rack_temp;
static ros::Publisher rack_cld_in;
static bool bBoxDim = false;
static int nBoxColorIndex = 0;
static bool bFirstFrame = true;
static int nFrameCount =0;
static int nCount = 0;
static int nFaceCount = 0;

// 改这里没用，wpr1_agent_node会更新这些参数。改wpr1_agent_node.cpp的box_grab_z
static float focus_x = 1.0;
static float focus_y = 0.0;
static float focus_z = 0.85;
static float range_z = 0.3;

static float pass_top = focus_z + range_z;
static float pass_bottom = focus_z - range_z;
static float pass_center_x = focus_x;
static float pass_length = 1.0;
static float pass_width = 2.0;

static float face_box_x = 0;
static float face_box_y = 0;
static float face_box_th = 0;

static float face_box_dist = 1.0f;       //机器人和盒子的距离（改这个没用，从wpr1_agent传过来）
static float box_grab_z = 0.84;          //对料盒进行抓取的垂直高度（改这个没用，从wpr1_agent传过来）
static float grab_x_offset = 0.02;         //抓取过程中，向前抓取物体的修正值，改这里有用！

static float place_face_dist = 1.1;    //放置的对准距离比抓取的远从(改这个没用，wpr1_agent传过来）
static float place_x_offset = -0.0;    //放置距离的修正量
static float place_color_z = 0.1;       //放置高度的修正值(改这个没用，wpr1_agent传过来)

static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_rack;
static geometry_msgs::Pose box_pose_msg;
static std_msgs::Int64 box_color_msg;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr move_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_box(new pcl::PointCloud<pcl::PointXYZRGB>);    //料盒模板
static CBoxColorDetect boxColorDetect;
static CBoxDim boxDim;

#define BOX_DIM_WAIT 0
#define BOX_DIM_FACE 1
#define BOX_DIM_GRAB 2
#define BOX_DIM_DONE 3
static int nState = BOX_DIM_WAIT;

#define OP_GRAB_BOX         0
#define OP_PLACE_COLOR  1
static int nOperate = OP_GRAB_BOX;

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void RackTF()
{
    static tf::Transform transform_rack;
    transform_rack.setOrigin( tf::Vector3(boxDim.boxPosition.x_offset, boxDim.boxPosition.y_offset, 0) );
    tf::Quaternion quat;
    // 目标姿态,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
    quat.setRPY(0.0, 0.0, -boxDim.boxPosition.angle);
    // 将欧拉角旋转量转换成四元数表达
    transform_rack.setRotation(quat);
    rack_pos_tf->sendTransform(tf::StampedTransform(transform_rack, ros::Time::now(), "base_footprint", "rack_pos"));
}

void DrawRackPos(stRackTemp* inRack)
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

    // 左右边界
    float left = -0.07;
    float right = 0.07;
    // 前后边界
    float depth = 0.2;
    // 先画层
    for(int i=0;i<2;i++)
    {
        float lvHeight = 0;
        if(i == 0)
        {
            lvHeight = box_grab_z+0.05;//-0.04;
        }
        else
        {
            lvHeight = box_grab_z+0.08;//+0.04;
        }
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
    for(int i=0;i<2;i++)
    {
        float y = 0;
        if(i == 0)
        {
            y = left;
        }
        else
        {
            y = right;
        }
        
        float height = 0.9;//boxDim.GetMaxHeight();
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

void RemoveRackPos()
{
    line_rack.action = 3;
    line_rack.points.clear();
    rack_marker_pub.publish(line_rack);
}


void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    nFrameCount ++;
    //ROS_INFO("box_dim_node ProcCloudCB ...");
    if(bBoxDim == false)
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

    if(cloud_source_ptr->points.size() == 0)
        return;
    
    // 如果是第一帧，先判断一下是指定颜色的盒子是否在视野里
    // if(bFirstFrame == true)
    // {
    //     bool bFoundBox = false;
    //     ROS_WARN("bef ColorFilter");
    //     boxColorDetect.ColorFilter(cloud_source_ptr);
    //     ROS_WARN("aft ColorFilter");
    //     if(nBoxColorIndex == 0 && boxColorDetect.arBox[0].size()>0)
    //     {
    //         bFoundBox = true;
    //     }
    //     if(nBoxColorIndex == 1 && boxColorDetect.arBox[1].size()>0)
    //     {
    //         bFoundBox = true;
    //     }
    //     boxColorDetect.arBox[0].clear();
    //     boxColorDetect.arBox[1].clear();
        
    //     if(bFoundBox == true)
    //     {
    //         bFirstFrame = false;
    //         // 将盒子颜色检测结果发送给wpr1_agent，目的是返送给wpr_warehousing_monitor
    //         box_color_msg.data = nBoxColorIndex;
    //         box_color_pub.publish(box_color_msg);
    //         nFaceCount = 0;
    //         nState = BOX_DIM_FACE; //开始对准
    //         ROS_WARN("nState -> BOX_DIM_FACE nBoxColorIndex= %d",nBoxColorIndex);
    //     }
    //     else
    //     {
    //         ROS_WARN("[box_dim_node ProcCloudCB] No box found! Check the box thredhold in file (BoxColorDetetct.cpp)");
    //     }
    //     return;
    // }

    // 使用nBoxColorIndex盒子的颜色阈值进行二值化处理
    bool bFoundBox = boxColorDetect.SingleFilter(cloud_source_ptr, nBoxColorIndex , true);
    if(bFoundBox == true)
    {
        if(nState == BOX_DIM_WAIT)
        {
            // 将盒子颜色检测结果发送给wpr1_agent，目的是返送给wpr_warehousing_monitor
            box_color_msg.data = nBoxColorIndex;
            box_color_pub.publish(box_color_msg);
            nFaceCount = 0;
            nState = BOX_DIM_FACE; //开始对准
            ROS_WARN("nState -> BOX_DIM_FACE nBoxColorIndex= %d",nBoxColorIndex);
        }
    }
    else
    {
        ROS_WARN("[box_dim_node ProcCloudCB] No box found! Check the box thredhold in file (BoxColorDetetct.cpp)");
    }

    // 显示二值化后的点云
    //rack_cld_in.publish(*cloud_source_ptr);
    //ros::spinOnce();
    
    // 计算盒子位置
    boxDim.CalBoxPosition(cloud_source_ptr);

    // printf("[box_dim_node] Final boxPosition= ( %.2f , %.2f , %.2f)\n",box_pose_msg.position.x , box_pose_msg.position.y , box_pose_msg.position.z);
    RackTF();
    DrawRackPos(boxDim.pRackTemp);
    // 注释掉，不显示盒子模板
    //rack_temp.publish(cloud_temp_box);
    ros::spinOnce();

}

void BoxCmdCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    // nFindIndex = msg->data.find("grab color box");
    // if( nFindIndex >= 0 )
    // {
    //     nState = BOX_DIM_WAIT;
    //     nBoxColorIndex = 0;
    //     boxDim.bBoxTracked = false; //开始新的匹配任务
    //     bFirstFrame = true;
    //     bBoxDim = true;
    //     ROS_WARN("[box_dim_node] grab color box !!");
    // }

    nFindIndex = msg->data.find("stop");
    if( nFindIndex >= 0 )
    {
        bBoxDim = false;
        nState = BOX_DIM_WAIT;
        ROS_WARN("[box_dim_node] stop ...");
    }
}

void BoxParamCB(const wpr_rack_pkg::BoxParam::ConstPtr& msg)
{
    nOperate = msg->operate;
    if(nOperate == OP_GRAB_BOX)
    {
        face_box_dist = msg->face_dist;       //机器人和盒子的距离（从wpr1_agent传过来）
        box_grab_z = msg->z;          //对料盒进行抓取的垂直高度（从wpr1_agent传过来）
    }
    if(nOperate == OP_PLACE_COLOR)
    {
        place_face_dist = msg->face_dist;       //机器人和颜色区域的距离（从wpr1_agent传过来）
        place_color_z = msg->z;          //放置动作的垂直高度（从wpr1_agent传过来）
    }
    focus_z = msg->z;
    pass_top = focus_z + range_z;
    pass_bottom = focus_z - range_z;
    nBoxColorIndex = msg->color;     //目标料盒颜色
    nState = BOX_DIM_WAIT;
    boxDim.bBoxTracked = false; //开始新的匹配任务
    bFirstFrame = true;
    bBoxDim = true;
    ROS_WARN("[box_dim_node] grab color %d !!",nBoxColorIndex);
}


void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[GrabResultCB] %s",msg->data.c_str());
    if(nState == BOX_DIM_GRAB)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("box grab done!");
            nCount = 0;
            nState = BOX_DIM_DONE;
        }
    }
}

void PlaceResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[PlaceResultCB] %s",msg->data.c_str());
    if(nState == BOX_DIM_GRAB)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("box grab done!");
            nCount = 0;
            nState = BOX_DIM_DONE;
        }
    }
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    if(nOperate == OP_GRAB_BOX)
    {
        int nFindIndex = msg->data.find("grab stop");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("[grab stop] ");
            nState = BOX_DIM_WAIT;
            VelCmd(0,0,0);
        }
    }
    if(nOperate == OP_PLACE_COLOR)
    {
        int nFindIndex = msg->data.find("place stop");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("[place stop] ");
            nState = BOX_DIM_WAIT;
            VelCmd(0,0,0);
        }
    }
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

void CalFaceBox(float inX, float inY, float inAngle, float inFaceDist)
{
    face_box_x = inX - inFaceDist*cos(inAngle);
    face_box_y = inY - inFaceDist*sin(inAngle);
    face_box_th = inAngle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_dim_node");
    ROS_WARN("box_dim_node start!");
    tf_listener = new tf::TransformListener();
    rack_pos_tf = new tf::TransformBroadcaster();

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/sd/points");

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 2 , ProcCloudCB);

    rack_temp = nh.advertise<sensor_msgs::PointCloud2> ("/rack/rack_temp",1);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    box_pub_0 = nh.advertise<visualization_msgs::Marker>("/box/marker_0", 10);
    box_pub_1 = nh.advertise<visualization_msgs::Marker>("/box/marker_1", 10);
    box_pose_pub = nh.advertise<geometry_msgs::Pose>("/box/pose", 10);
    box_color_pub = nh.advertise<std_msgs::Int64>("/box/color", 10);
    rack_cld_in = nh.advertise<sensor_msgs::PointCloud2> ("/box/track_show",1);
    rack_marker_pub = nh.advertise<visualization_msgs::Marker>("/rack/rack_marker", 10);
    ros::Publisher box_result_pub = nh.advertise<wpr_rack_pkg::ObjectBox> ("/box/result",1);
    ros::Subscriber box_cmd_sub = nh.subscribe("/box/command", 2, BoxCmdCB);
    ros::Subscriber grab_res_sub = nh.subscribe("/wpr1/grab_result", 30, GrabResultCB);
    ros::Subscriber param_res_sub = nh.subscribe("/box/param", 2, BoxParamCB);
    place_pub = nh.advertise<geometry_msgs::Pose>("/wpr1/place_action", 1);
    ros::Subscriber place_res_sub = nh.subscribe("/wpr1/place_result", 30, PlaceResultCB);
    ros::Subscriber sub_beh = nh.subscribe("/wpr1/behaviors", 30, BehaviorCB);
    
    boxColorDetect.Init();

    boxDim.InitCV();
    
    ros::Rate r(30);
    while(ros::ok())
    {
        if(nState == BOX_DIM_FACE)
        {
            if(nOperate == OP_GRAB_BOX)
                CalFaceBox(boxDim.boxPosition.x_offset, boxDim.boxPosition.y_offset, -boxDim.boxPosition.angle, face_box_dist); 
            if(nOperate == OP_PLACE_COLOR)
                CalFaceBox(boxDim.boxPosition.x_offset, boxDim.boxPosition.y_offset, -boxDim.boxPosition.angle, place_face_dist);

            printf("-----------------------------------------\n");
            printf("[box_dim_node-BOX_DIM_FACE] boxPosition= ( %.2f , %.2f ) ang= %.2f\n",boxDim.boxPosition.x_offset , -boxDim.boxPosition.y_offset , boxDim.boxPosition.angle);
            printf("[box_dim_node-BOX_DIM_FACE] facePosition= ( %.2f , %.2f ) ang= %.2f\n",face_box_x,face_box_y,face_box_th);

            float vx,vy;
            vx = face_box_x /2;
            vy = face_box_y /2;
            if(fabs(vx) <= 0.005 && fabs(vy) <= 0.005)
            {
                vx = 0;
                vy = 0;
            }
            float vth = face_box_th * 1;
            if(fabs(vth) < 0.02)
            {
                vth = 0;
            }
            // 速度太快的话需要限速
            if(vx > 0.2) vx=0.2;
            if(vx < -0.2) vx = -0.2;
            if(vy > 0.2) vy=0.2;
            if(vy < -0.2) vy= -0.2;
            if(vth > 1.0) vth=1.0;
            if(vth < -1.0) vx = -1.0;
            VelCmd(vx,vy,vth);
            printf("[box_dim_node-BOX_DIM_FACE] vel= ( %.2f , %.2f , %.2f)\n",vx,vy,vth);
            nFaceCount ++;
            if(vx == 0 && vy ==0 && vth ==0 && nFaceCount > 20)
            {
                ROS_INFO("nState -> BOX_DIM_GRAB");
                nState = BOX_DIM_GRAB;
                if(nOperate == OP_PLACE_COLOR)
                {
                    place_msg.position.x = boxDim.boxPosition.x_offset + place_x_offset;      //放置的前后距离
                    place_msg.position.y = boxDim.boxPosition.y_offset;                   //放置的左右偏移量
                    place_msg.position.z = place_color_z;   //放置的高度
                    place_pub.publish(place_msg);
                    boxDim.bBoxTracked = false;
                    bBoxDim = false; //关闭视觉处理
                    ROS_INFO("[box_dim_node-BOX_DIM_GRAB] place ( %.2f , %.2f , %.2f)",place_msg.position.x,place_msg.position.y,place_msg.position.z);
                }
            }
        }

        if(nState == BOX_DIM_GRAB)
        {
            if(nOperate == OP_GRAB_BOX)
            {
                // 盒子位置发布出去
                box_pose_msg.position.x = boxDim.boxPosition.x_offset + grab_x_offset;
                box_pose_msg.position.y = boxDim.boxPosition.y_offset;
                box_pose_msg.position.z = box_grab_z;
                box_pose_pub.publish(box_pose_msg);  //这个发布会激活wpr1_behavior里的grab_box行为
                //ROS_INFO("[box_dim_node-BOX_DIM_GRAB] grab ( %.2f , %.2f , %.2f)",box_pose_msg.position.x,box_pose_msg.position.y,box_pose_msg.position.z);
                // 在回调函数里监视抓取行为何时完成
            }
        }

        if(nState == BOX_DIM_DONE)
        {
            VelCmd(0,0,0);
            if(nCount > 10)
            {
                nState = BOX_DIM_WAIT;
            }
        }
        nCount ++;
        if(nCount >= 30)
        {
            //ROS_INFO("[box_dim_node] fps= %d", nFrameCount);
            nFrameCount = 0;
            nCount = 0;
        }
        ros::spinOnce();
        r.sleep();
    }

    delete tf_listener;
    delete rack_pos_tf;

    return 0;
}