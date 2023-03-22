#include <ros/ros.h>
#include <std_msgs/String.h>
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
#include "BoxTrack.h"
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
static ros::Publisher rack_marker_pub;
static ros::Publisher vel_pub;
static ros::Publisher box_pub_0;
static ros::Publisher box_pub_1;
static ros::Publisher box_pose_pub;
static tf::TransformListener *tf_listener; 
static tf::TransformBroadcaster *rack_pos_tf;
static ros::Publisher rack_temp;
static ros::Publisher rack_cld_in;
static bool bTrackBox = false;
static int nBoxColorIndex = 0;
static bool bFirstFrame = true;
static int nFrameCount =0;

static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_rack;
static geometry_msgs::Pose box_pose_msg;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);     //用于迭代计算的点云
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr move_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_box(new pcl::PointCloud<pcl::PointXYZRGB>);    //料盒模板
static CBoxColorDetect boxColorDetect;
static CBoxTrack boxTrack;

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
    transform_rack.setOrigin( tf::Vector3(boxTrack.boxPosition.x_offset, boxTrack.boxPosition.y_offset, 0) );
    tf::Quaternion quat;
    // 目标姿态,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
    quat.setRPY(0.0, 0.0, boxTrack.boxPosition.angle);
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
        float height = boxTrack.GetMaxHeight();
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
    //ROS_INFO("box_track_node ProcCloudCB ...");
    if(bTrackBox == false)
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
    
    // 使用颜色阈值进行二值化处理
    if(bFirstFrame == true)
    {
        // 如果是第一帧，那么需要提取一下联通域，中心坐标作为初始跟踪坐标
        bool res = boxColorDetect.SingleFilter(cloud_source_ptr, nBoxColorIndex , true);
        if(res == false)
        {
            ROS_WARN("SingleFilter = false");
            return;
        }
        bFirstFrame = false;
        int nBoxIndex = boxColorDetect.nIndexBox[nBoxColorIndex];
        stBoxMarker tmpBox = boxColorDetect.arBox[0][nBoxIndex];
        boxTrack.curResult.x_offset = (tmpBox.xMax+tmpBox.xMin)*100/2;
        boxTrack.curResult.y_offset = (tmpBox.yMax+tmpBox.yMin)*100/2;
        boxTrack.curResult.angle = 0;
        ROS_WARN("bFirstFrame [curResult] = ( %.2f , %.2f )  %.2f",boxTrack.curResult.x_offset*0.01 , boxTrack.curResult.y_offset*0.01 , boxTrack.curResult.angle);
    }
    else
    {
        boxColorDetect.SingleFilter(cloud_source_ptr, nBoxColorIndex , false);
    }

    // 显示二值化后的点云
    //rack_cld_in.publish(*cloud_source_ptr);
    //ros::spinOnce();

    pcl::copyPointCloud(*cloud_source_ptr, *box_cloud);
    
    // 从上一帧最后的坐标重新开始匹配
    bool bGot = false;
    while(bGot == false)
    {
        bGot = boxTrack.SingleTest(boxTrack.pICPStep, box_cloud);
    }

    // 发布出去
    box_pose_msg.position.x = boxTrack.boxPosition.x_offset;
    box_pose_msg.position.y = boxTrack.boxPosition.y_offset;
    box_pose_msg.position.z = boxTrack.GetMinHeight();
    box_pose_pub.publish(box_pose_msg);

    ROS_WARN("[Final boxPosition] = ( %.2f , %.2f , %.2f) ",boxTrack.boxPosition.x_offset , boxTrack.boxPosition.y_offset , box_pose_msg.position.z);
    RackTF();
    DrawRackPos(boxTrack.pRackTemp);
    rack_temp.publish(cloud_temp_box);
    ros::spinOnce();

    //按照匹配的结果计算速度并执行（待实现）

}

void BoxCtrlCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("track box 0");
    if( nFindIndex >= 0 )
    {
        nBoxColorIndex = 0;
        bFirstFrame = true;
        bTrackBox = true;
        ROS_WARN("[box_track_node] Track Box - 0 !!");
    }

    nFindIndex = msg->data.find("track box 1");
    if( nFindIndex >= 0 )
    {
        nBoxColorIndex = 1;
        bFirstFrame = true;
        bTrackBox = true;
        ROS_WARN("[box_track_node] Track Box - 1 !!");
    }

    nFindIndex = msg->data.find("stop track");
    if( nFindIndex >= 0 )
    {
        bFirstFrame = true;
        bTrackBox = false;
        ROS_WARN("[box_track_node] Stop !!");
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_track_node");
    ROS_WARN("box_track_node start!");
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
    rack_cld_in = nh.advertise<sensor_msgs::PointCloud2> ("/box/track_show",1);
    rack_marker_pub = nh.advertise<visualization_msgs::Marker>("/rack/rack_marker", 10);
    ros::Publisher box_result_pub = nh.advertise<wpr_rack_pkg::ObjectBox> ("/box/result",1);
    ros::Subscriber ctrl_sub = nh.subscribe("/box/command", 2, BoxCtrlCB);
    
    boxColorDetect.Init();

    boxTrack.LoadTemp("/home/master/box.tmp");
    //将模板赋值到点云里显示
    pcl::PointXYZRGB new_point;
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                if(boxTrack.pRackTemp->data[x][y][z] == 255)    //只显示模板
                {
                    new_point.x = (float)x/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.y = (float)y/100 - 1.0; //模板是2米×2米，将其拉到中间
                    new_point.z = (float)z/100;
                    new_point.g = boxTrack.pRackTemp->data[x][y][z];
                    cloud_temp_box->points.push_back(new_point);
                }
            }
    cloud_temp_box->width = (int) cloud_temp_box->points.size();
    cloud_temp_box->height = 1;
    cloud_temp_box->header.frame_id = "rack_pos";

    ros::Rate r(30);
    int nCount = 0;
    while(ros::ok())
    {
        nCount ++;
        if(nCount >= 30)
        {
            ROS_INFO("[box_track_node] fps= %d", nFrameCount);
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