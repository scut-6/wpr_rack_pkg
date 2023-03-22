#include "BoxObject.h"

CBoxObject::CBoxObject()
{
    fCulling_v = 0.08;
    fCulling_h = 0.08;
    fPlaceDistSide = 0.12;
    fPlaceDistObj = 0.10;
    fGrabAbovePlane = 0.08;
    fGrabHeightMax = 1.2;
    fGrabHeightMin = 0.6;
}
    
CBoxObject::~CBoxObject()
{
    
}

void CBoxObject::GenTemp(bool inScale)
{
     ROS_INFO("CBoxObject::GenTemp");
    
    // 几个尺寸变量
    int box_width = 12;     //盒体的宽度
    int box_length = 22;    //盒体的长度
    int box_height = 6;     //盒体的高度
    int position_height = 72;   //盒体放置的高度

    pRackTemp->h_partition_y[0] = box_width/2;
    pRackTemp->h_partition_y[1] = pRackTemp->h_partition_y[0];
    pRackTemp->h_partition_y[2] = -box_width/2;
    pRackTemp->h_partition_y[3] = pRackTemp->h_partition_y[2];
    pRackTemp->v_partition_z[0] = position_height;
    pRackTemp->v_partition_z[1] = pRackTemp->v_partition_z[0];
    pRackTemp->v_partition_z[2] = position_height + box_height;
    pRackTemp->v_partition_z[3] = pRackTemp->v_partition_z[2];
    pRackTemp->d_partition_x = box_length/2;

    for(int i=0;i<4;i++)
    {
        pRackTemp->h_partition_y[i] += RACK_TEMP_Y/2;
    }

    int y_left = RACK_TEMP_Y/2+box_width/2;
    int y_right = RACK_TEMP_Y/2-box_width/2;
    int x_front = RACK_TEMP_X/2 - box_length/2;
    int x_back = RACK_TEMP_X/2 + box_length/2;
    // 盒体正面和背面
    for(int y=y_right;y<y_left;y++)
    {
        for(int z=position_height;z<(position_height+box_height);z++)
        {
            pRackTemp->data[x_front][y][z] = 255;
            pRackTemp->data[x_back][y][z] = 255;
        }
    }

    // 盒体的左右侧面
    // for(int x=x_front;x<x_back;x++)
    // {
    //     for(int z=position_height;z<(position_height+box_height);z++)
    //     {
    //         pRackTemp->data[x][y_left][z] = 255;
    //         pRackTemp->data[x][y_right][z] = 255;
    //     }
    // }

    // 盒体的底面
    int x_mid = (x_front+x_back)/2;  //底面只保留后半段，因为前半段被挡住了
    for(int x=x_mid;x<x_back;x++)
    {
        for(int y=y_right;y<y_left;y++)
        {
            pRackTemp->data[x][y][position_height] = 255;
        }
    }
    
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
 
    InitScalePoint();
    // 距离渐变
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                if(pRackTemp->data[x][y][z] == 255)
                {
                    if(inScale == true)
                        ScalePoint(x,y,z);
                    nPointIndex ++;
                    float perc = (float)nPointIndex*100/mNumOfPoints;
                    ROS_INFO("%.0f%% -- %d / %d -- (%d, %d, %d)",perc,nPointIndex,mNumOfPoints,x,y,z);
                }
            }
}