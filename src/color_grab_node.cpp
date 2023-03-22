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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 预期盒子位置
static float box_pre_x = 1.0;
static float box_pre_y = 0.5;
static float box_pre_z = 0.69;

// 速度比例
static float linear_v_k = 0.01;

// 点云直通滤波区域
static float focus_x = 1.0;
static float focus_y = 0.0;
static float focus_z = 0.85;
static float range_z = 0.3;

static float pass_top = focus_z + range_z;
static float pass_bottom = focus_z - range_z;
static float pass_center_x = focus_x;
static float pass_length = 1.0;
static float pass_width = 2.0;

// 记录平移值，以便回到初始位置
static float y_return = 0;

static std::string pc_topic;
static ros::Publisher rack_marker_pub;
static ros::Publisher vel_pub;
static ros::Publisher box_pub_0;
static ros::Publisher box_pub_1;
static ros::Publisher box_pose_pub;
static ros::Publisher box_color_pub;
static tf::TransformListener *tf_listener; 
static tf::TransformBroadcaster *rack_pos_tf;
static ros::Publisher rack_temp;
static ros::Publisher rack_cld_in;
static bool bPointCloudIn = false;
static int nBoxColorIndex = 0;
static bool bFirstFrame = true;
static int nFrameCount =0;
static int nCount = 0;
static int nFaceCount = 0;

static float face_box_x = 0;
static float face_box_y = 0;
static float face_box_th = 0;

static float face_box_dist = 1.0f;       //机器人和盒子的距离（从wpr1_agent传过来）
static float box_grab_z = 0.84;          //对料盒进行抓取的垂直高度（从wpr1_agent传过来）
static float box_grab_x = 1.0;            // 抓取距离（从点云计算得到）
static float box_grab_y = 0.5;            // 抓取横向偏移量（从点云计算得到）

static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_rack;
static geometry_msgs::Pose box_pose_msg;
static std_msgs::Int64 box_color_msg;
static std_msgs::String grab_res_msg;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr move_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_box(new pcl::PointCloud<pcl::PointXYZRGB>);    //料盒模板
static CBoxColorDetect boxColorDetect;

#define COLOR_GRAB_WAIT         0
#define COLOR_GRAB_FACE         1
#define COLOR_GRAB_GRAB         2
#define COLOR_GRAB_RETURN   3
#define COLOR_GRAB_DONE         4

static int nState = COLOR_GRAB_WAIT;

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void DrawBoxPos(stBoxMarker* inBox)
{
    line_rack.header.frame_id = "base_footprint";
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
    float left = inBox->yMax;
    float right = inBox->yMin;
    // 前后边界
    float depth = (inBox->xMax - inBox->xMin);
    // 高度范围
    float top =  inBox->zMax;
    float bottom =  inBox->zMin;
    for(int i=0;i<2;i++)
    {
        float lvHeight = 0;
        if(i == 0)
        {
            lvHeight = top;
        }
        else
        {
            lvHeight = bottom;
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
        
        geometry_msgs::Point p;
        p.y = y;
        p.z = top; p.x = depth/2; line_rack.points.push_back(p);
        p.z = top; p.x = -(depth/2); line_rack.points.push_back(p);
        p.z = top; p.x = -(depth/2); line_rack.points.push_back(p);
        p.z = bottom; p.x = -(depth/2); line_rack.points.push_back(p);
        p.z = bottom; p.x = -(depth/2); line_rack.points.push_back(p);
        p.z = bottom; p.x = depth/2; line_rack.points.push_back(p);
        p.z = bottom; p.x = depth/2; line_rack.points.push_back(p);
        p.z = top; p.x = depth/2; line_rack.points.push_back(p);
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
    //ROS_INFO("COLOR_GRAB_node ProcCloudCB ...");
    if(bPointCloudIn == false)
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
    
    // 如果是第一帧，先判断一下是什么颜色的盒子
    if(bFirstFrame == true)
    {
        bool bFoundBox = false;
        boxColorDetect.ColorFilter(cloud_source_ptr);
        if(boxColorDetect.arBox[0].size()>0)
        {
            nBoxColorIndex = 0;
            bFoundBox = true;
        }
        else if(boxColorDetect.arBox[1].size()>0)
        {
            nBoxColorIndex = 1;
            bFoundBox = true;
        }
        boxColorDetect.arBox[0].clear();
        boxColorDetect.arBox[1].clear();

        // 将盒子颜色检测结果发送给wpr1_agent
        box_color_msg.data = nBoxColorIndex;
        box_color_pub.publish(box_color_msg);
        
        if(bFoundBox == true)
        {
            bFirstFrame = false;
            nFaceCount = 0;
            nState = COLOR_GRAB_FACE; //开始对准
            ROS_WARN("nState -> COLOR_GRAB_FACE nBoxColorIndex= %d",nBoxColorIndex);
        }
        else
        {
            ROS_WARN("[color_grab_node] No box found! Check the box thredhold in file (BoxColorDetetct.cpp)");
        }
        return;
    }

    // 使用nBoxColorIndex盒子的颜色阈值进行二值化处理
    boxColorDetect.SingleFilter(cloud_source_ptr, nBoxColorIndex , true);   //（最后参数true是提取盒子）

    // 显示二值化后的点云
    //rack_cld_in.publish(*cloud_source_ptr);
    //ros::spinOnce();
    
    // 找出距离预期位置最接近的盒子位置
    int nNumObj = boxColorDetect.arBox[nBoxColorIndex].size();
    //ROS_INFO("arBox[%d] = %d",inIndex,nNumObj);
	if(nNumObj > 0)
	{
		//找出中心离预期点最近的作为盒子
		int nIndexClosest = 0;
		float minDist = boxColorDetect.CalDist( ( boxColorDetect.arBox[nBoxColorIndex][0].xMax+ boxColorDetect.arBox[nBoxColorIndex][0].xMin)/2 ,  ( boxColorDetect.arBox[nBoxColorIndex][0].yMax+ boxColorDetect.arBox[nBoxColorIndex][0].yMin)/2 , ( boxColorDetect.arBox[nBoxColorIndex][0].zMax+ boxColorDetect.arBox[nBoxColorIndex][0].zMin)/2 , box_pre_x, box_pre_y, box_pre_z);
		for(int i=0;i<nNumObj;i++)
		{
			float tmpDist = boxColorDetect.CalDist( ( boxColorDetect.arBox[nBoxColorIndex][i].xMax+ boxColorDetect.arBox[nBoxColorIndex][i].xMin)/2 ,  ( boxColorDetect.arBox[nBoxColorIndex][i].yMax+ boxColorDetect.arBox[nBoxColorIndex][i].yMin)/2 , ( boxColorDetect.arBox[nBoxColorIndex][i].zMax+ boxColorDetect.arBox[nBoxColorIndex][i].zMin)/2 , box_pre_x, box_pre_y, box_pre_z);
			if(tmpDist < minDist)
			{
				minDist = tmpDist;
				nIndexClosest = i;
			}
		}
		float cx = (boxColorDetect.arBox[nBoxColorIndex][nIndexClosest].xMax + boxColorDetect.arBox[nBoxColorIndex][nIndexClosest].xMin)/2;
		float cy = (boxColorDetect.arBox[nBoxColorIndex][nIndexClosest].yMax + boxColorDetect.arBox[nBoxColorIndex][nIndexClosest].yMin)/2;
		float cz = (boxColorDetect.arBox[nBoxColorIndex][nIndexClosest].zMax + boxColorDetect.arBox[nBoxColorIndex][nIndexClosest].zMin)/2;
		//ROS_WARN("BoxClosest_%d (%.2f , %.2f , %.2f)",inIndex,cx,cy,cz);
        box_grab_x = boxColorDetect.arBox[nBoxColorIndex][nIndexClosest].xMin;
        box_grab_y = cy;
        DrawBoxPos(&(boxColorDetect.arBox[nBoxColorIndex][nIndexClosest]));
	}

    // 注释掉，不显示盒子模板
    //rack_temp.publish(cloud_temp_box);
    ros::spinOnce();

}

void ColorCmdCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("grab color blob");
    if( nFindIndex >= 0 )
    {
        nState = COLOR_GRAB_WAIT;
        nBoxColorIndex = 0;
        bFirstFrame = true;
        bPointCloudIn = true;
        ROS_WARN("[color_grab_node] grab color blob !!");
    }

    nFindIndex = msg->data.find("stop");
    if( nFindIndex >= 0 )
    {
        bPointCloudIn = false;
        nState = COLOR_GRAB_WAIT;
        ROS_WARN("[color_grab_node] stop ...");
    }
}

void ColorParamCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() < 2)
        return;
    face_box_dist = msg->data[0];       //机器人和盒子的距离（从wpr1_agent传过来）
    box_grab_z = msg->data[1];          //对料盒进行抓取的垂直高度（从wpr1_agent传过来）
    pass_top = focus_z + range_z;
    pass_bottom = focus_z - range_z;
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[GrabResultCB] %s",msg->data.c_str());
    if(nState == COLOR_GRAB_GRAB)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("grab done. Return!");
            nCount = 0;
            nState = COLOR_GRAB_RETURN;
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
    ros::init(argc, argv, "color_grab_node");
    ROS_WARN("color_grab_node start!");
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
    ros::Publisher color_res_pub = nh.advertise<std_msgs::String>("/color/result", 10);
    ros::Subscriber color_cmd_sub = nh.subscribe("/color/command", 2, ColorCmdCB);
    ros::Subscriber grab_res_sub = nh.subscribe("/wpr1/grab_result", 30, GrabResultCB);
    ros::Subscriber color_param_sub = nh.subscribe("/color/param", 2, ColorParamCB);
    
    boxColorDetect.Init();
    
    ros::Rate r(30);
    while(ros::ok())
    {
        if(nState == COLOR_GRAB_FACE)
        {
            printf("-----------------------------------------\n");
            printf("[color_grab_node-BOX_COLOR_FACE] box_grab_pos= ( %.2f , %.2f %.2f)\n",box_grab_x , box_grab_y , box_grab_z);

            float vx,vy;
            vx = 0;
            vy = box_grab_y * linear_v_k;
            if(fabs(vx) <= 0.005 && fabs(vy) <= 0.005)
            {
                vx = 0;
                vy = 0;
            }
            float vth = 0;
            VelCmd(vx,vy,vth);
            printf("[COLOR_GRAB_node-COLOR_GRAB_FACE] vel= ( %.2f , %.2f , %.2f)\n",vx,vy,vth);
            nFaceCount ++;
            if(vx == 0 && vy ==0 && vth ==0 && nFaceCount > 30)
            {
                ROS_INFO("nState -> COLOR_GRAB_GRAB");
                nCount = 0;
                nState = COLOR_GRAB_GRAB;
            }
        }

        if(nState == COLOR_GRAB_GRAB)
        {
            if(nCount == 0)
            {
                // 盒子位置发布出去
                box_pose_msg.position.x = box_grab_x;
                box_pose_msg.position.y = box_grab_y;
                box_pose_msg.position.z = box_grab_z;
                box_pose_pub.publish(box_pose_msg);  //这个发布会激活wpr1_behavior里的grab_box行为
                //ROS_INFO("[COLOR_GRAB_node-COLOR_GRAB_GRAB] grab ( %.2f , %.2f , %.2f)",box_pose_msg.position.x,box_pose_msg.position.y,box_pose_msg.position.z);            
                y_return = box_grab_y;
                ROS_INFO("y_return = %.2f",y_return);
                nCount ++;
             }
            // 在回调函数里监视抓取行为何时完成
        }

        if(nState == COLOR_GRAB_RETURN)
        {
            int sec_retrun = 4;     //回到初始位置的秒数
            float vy = -y_return/sec_retrun;
            VelCmd(0,vy,0);
            if(nCount > sec_retrun*30)
            {
                nCount = 0;
                nState = COLOR_GRAB_DONE;
            }
        }

        if(nState == COLOR_GRAB_DONE)
        {
            grab_res_msg.data = "done";
            color_res_pub.publish(grab_res_msg);
            VelCmd(0,0,0);
            if(nCount > 10)
            {
                nState = COLOR_GRAB_WAIT;
            }
        }
        nCount ++;
        // if(nCount >= 30)
        // {
        //     ROS_INFO("[COLOR_GRAB_node] fps= %d", nFrameCount);
        //     nFrameCount = 0;
        //     nCount = 0;
        // }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}