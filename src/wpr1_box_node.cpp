
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <wpr_rack_pkg/ObjectBox.h>
#include <geometry_msgs/PoseArray.h>

//面对货架的距离，以及左右偏移的范围
static float fFaceDist = 1.2;
static float fFaceMinY = -0.1;
static float fFaceMaxY = 0.1;

#define STEP_IDEL           0
#define STEP_RACK_DETECT    1
#define STEP_FACE_MOVE      2
#define STEP_FACE_ROT       3
#define STEP_OBJ_DETECT     4
#define STEP_OBJ_GRAB       5
#define STEP_OBJ_PLACE      6
#define STEP_NAP            7
static int nStep = STEP_IDEL;
static int nNapCount = 0;

static ros::Publisher rack_pub;
static std_msgs::String rack_msg;
static ros::Publisher grab_pub;
static geometry_msgs::Pose grab_msg;
static ros::Publisher place_pub;
static geometry_msgs::Pose place_msg;
static ros::Publisher odom_ctrl_pub;
static std_msgs::String odom_ctrl_msg;
static geometry_msgs::Pose2D pose_diff;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static bool bGrabbing = false;

std::vector<geometry_msgs::Pose> arPlacePose;

static float face_rack_x = 0;
static float face_rack_y = 0;
static float face_rack_th = 0;

void CalFaceRack(float inX, float inY, float inAngle, float inFaceDist)
{
    face_rack_x = inX - inFaceDist*cos(inAngle);
    face_rack_y = inY - inFaceDist*sin(inAngle);
    face_rack_th = inAngle;
}

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void RackPositionCB(const geometry_msgs::Pose::ConstPtr &msg)
{
    if(nStep == STEP_RACK_DETECT)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        ROS_WARN("[RackPositionCB] rack pos (%.2f , %.2f): %.2f",msg->position.x,msg->position.y,yaw);
        return;

        //位置比较正的时候启动抓取，否则继续调整
        float dist_diff = fabs(msg->position.x-fFaceDist);
        if(dist_diff <0.1 && msg->position.y>fFaceMinY && msg->position.y<fFaceMaxY && fabs(yaw) < 0.08)
        {
            rack_msg.data = "detect object";
            rack_pub.publish(rack_msg);
            nStep = STEP_OBJ_DETECT;
            ROS_WARN("nStep = STEP_OBJ_DETECT ");
        }
        else
        {
            CalFaceRack(msg->position.x,msg->position.y,yaw, fFaceDist );
            ROS_WARN("[RackPositionCB] face pos (%.2f , %.2f): %.2f",face_rack_x,face_rack_y,face_rack_th);
            odom_ctrl_msg.data = "pose_diff reset";
            odom_ctrl_pub.publish(odom_ctrl_msg);
            nStep = STEP_FACE_MOVE;
        }
    }
}

void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    pose_diff.x = msg->x;
    pose_diff.y = msg->y;
    pose_diff.theta = msg->theta;
}


void ObjectsBoxCB(const wpr_rack_pkg::ObjectBox::ConstPtr &msg)
{
    if(nStep == STEP_OBJ_DETECT)
    {
        int nNumObj = msg->name.size();
        //ROS_WARN("[ObjectsBoxCB] obj = %d",nNumObj);
        if(nNumObj > 0)
        {
            int nGrabIndex = 0;
            float fMinY = 100;
            for(int i=0;i<nNumObj;i++)
            {
                float obj_x = msg->xMin[i];
                float obj_y = (msg->yMin[i] + msg->yMax[i])/2;
                float obj_z = msg->zMin[i];
                ROS_INFO("[ObjBoxCB] Obj_%d (%.2f , %.2f , %.2f)",i,obj_x,obj_y,obj_z);
                if(fabs(obj_y) < fMinY)
                {
                    fMinY = fabs(obj_y);
                    nGrabIndex = i;
                }
            }
            float obj_x = msg->xMin[nGrabIndex];
            float obj_y = (msg->yMin[nGrabIndex] + msg->yMax[nGrabIndex])/2;
            float obj_z = msg->zMin[nGrabIndex];
            ROS_WARN("[Grab] Obj_%d (%.2f , %.2f , %.2f)",nGrabIndex,obj_x,obj_y,obj_z);
            grab_msg.position.x = obj_x;
            grab_msg.position.y = obj_y;
            grab_msg.position.z = obj_z;
            grab_pub.publish(grab_msg);
            nStep = STEP_OBJ_GRAB;
        }
    }
}

void PlacePositionCB(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    int nNumPlace = msg->poses.size();
    //ROS_WARN("[PlacePositionCB] obj = %d",nNumPlace);
    arPlacePose.clear();
    geometry_msgs::Pose pose;
    for(int i=0;i<nNumPlace;i++)
    {
        pose.position = msg->poses[i].position;
        arPlacePose.push_back(pose);
    }
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[GrabResultCB] %s",msg->data.c_str());
    if(nStep == STEP_OBJ_GRAB)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("grab done!");
            sleep(1);
            // 将物品放置到货架上
            if(arPlacePose.size() > 0)
            {
                int nPlaceIndex = 0;
                place_msg.position.x = arPlacePose[nPlaceIndex].position.x;
                place_msg.position.y = arPlacePose[nPlaceIndex].position.y;
                place_msg.position.z = arPlacePose[nPlaceIndex].position.z;
                place_pub.publish(place_msg);
                ROS_WARN("[Place] Pos_%d (%.2f , %.2f , %.2f)",nPlaceIndex,place_msg.position.x,place_msg.position.y,place_msg.position.z);
                nStep = STEP_OBJ_PLACE;
                ROS_WARN("nStep = STEP_OBJ_PLACE");
            }
            else
            {
                VelCmd(0,0,0);
                nNapCount = 0;
                nStep = STEP_NAP;
                ROS_WARN("nStep = STEP_NAP");
            }
        }
    }
}

void PlaceResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[PlaceResultCB] %s",msg->data.c_str());
    if(nStep == STEP_OBJ_PLACE)
    {
        int nFindIndex = 0;
        nFindIndex = msg->data.find("done");
        if( nFindIndex >= 0 )
        {
            ROS_WARN("place done!");
            nNapCount = 0;
            nStep = STEP_NAP;
            ROS_WARN("nStep = STEP_NAP");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpr1_box_node");
    ROS_INFO("wpr1_box_node start!");

    ros::NodeHandle n;
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    rack_pub = n.advertise<std_msgs::String>("/rack/command", 10);
    grab_pub = n.advertise<geometry_msgs::Pose>("/wpr1/grab_action", 1);
    ros::Subscriber grab_res_sub = n.subscribe("/wpr1/grab_result", 30, GrabResultCB);
    place_pub = n.advertise<geometry_msgs::Pose>("/wpr1/place_action", 1);
    ros::Subscriber place_res_sub = n.subscribe("/wpr1/place_result", 30, PlaceResultCB);
    ros::Subscriber rack_sub = n.subscribe("/rack/rack_position", 10, RackPositionCB);
    ros::Subscriber obj_sub = n.subscribe("/rack/obj_position", 10, ObjectsBoxCB);
    ros::Subscriber place_sub = n.subscribe("/rack/place_positon", 10, PlacePositionCB);
    odom_ctrl_pub = n.advertise<std_msgs::String>("/wpr1/ctrl", 10);
    ros::Subscriber pose_diff_sub = n.subscribe("/wpr1/pose_diff", 1, PoseDiffCallback);
    mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("/wpr1/mani_ctrl", 30);

    mani_ctrl_msg.name.resize(5);
    mani_ctrl_msg.position.resize(5);
    mani_ctrl_msg.velocity.resize(5);
    mani_ctrl_msg.name[0] = "base_to_torso";
    mani_ctrl_msg.name[1] = "torso_to_upperarm";
    mani_ctrl_msg.name[2] = "upperarm_to_forearm";
    mani_ctrl_msg.name[3] = "forearm_to_palm";
    mani_ctrl_msg.name[4] = "gripper";
    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.position[1] = -1.57;
    mani_ctrl_msg.position[2] = -0.7;
    mani_ctrl_msg.position[3] = 0;
    mani_ctrl_msg.position[4] = 25000;
    mani_ctrl_msg.velocity[0] = 1500;
    mani_ctrl_msg.velocity[1] = 1500;
    mani_ctrl_msg.velocity[2] = 1500;
    mani_ctrl_msg.velocity[3] = 1500;
    mani_ctrl_msg.velocity[4] = 1500;

    sleep(3);
    //rack_msg.data = "detect mobile rack";

    // place_msg.position.x = 0.9;
    // place_msg.position.y = 0.3;
    // place_msg.position.z = 1.0;
    // place_pub.publish(place_msg);

    rack_msg.data = "detect box";
    rack_pub.publish(rack_msg);
    nStep = STEP_RACK_DETECT;

    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
        if(nStep == STEP_RACK_DETECT || nStep == STEP_OBJ_DETECT)
        {
            VelCmd(0,0,0);
        }
        if(nStep == STEP_FACE_MOVE)
        {
            float vx,vy;
            vx = (face_rack_x - pose_diff.x)/2;
            vy = (face_rack_y - pose_diff.y)/2;
            VelCmd(vx,vy,0);
            //ROS_INFO("[STEP_FACE_MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,face_rack_x, face_rack_y, pose_diff.x ,pose_diff.y,vx,vy);
            if(fabs(vx) <= 0.01 && fabs(vy) <= 0.01)
            {
                VelCmd(0,0,0);
                nStep = STEP_FACE_ROT;
                ROS_INFO("nStep = STEP_FACE_ROT");
            }
        }
        if(nStep == STEP_FACE_ROT)
        {
            float vth = (face_rack_th - pose_diff.theta);
            VelCmd(0,0,vth);
            if(fabs(vth) < 0.01)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                // 让rack_detect_node再次启动货架检测
                rack_msg.data = "detect box";
                rack_pub.publish(rack_msg);
                nStep = STEP_RACK_DETECT;
                ROS_INFO("nStep = STEP_RACK_DETECT");
                continue;
            }
        }
        if(nStep == STEP_NAP)
        {
            mani_ctrl_msg.position[0] = 0;
            mani_ctrl_msg.position[1] = -1.57;
            mani_ctrl_msg.position[2] = -0.7;
            mani_ctrl_msg.position[3] = 0;
            mani_ctrl_msg.position[4] = 25000;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            VelCmd(0,0,0);
            nNapCount ++;
            if(nNapCount > 33*3)
            {
                nNapCount = 0;
                rack_msg.data = "detect box";
                rack_pub.publish(rack_msg);
                nStep = STEP_RACK_DETECT;
                ROS_INFO("nStep = STEP_RACK_DETECT");
            }
        }
    }

    return 0;
}
