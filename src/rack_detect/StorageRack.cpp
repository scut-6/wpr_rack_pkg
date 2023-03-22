#include "StorageRack.h"

CStorageRack::CStorageRack()
{
    fCulling_v = 0.05;
    fCulling_h = 0.08;
    fPlaceDistSide = 0.2;
    fPlaceDistObj = 0.15;
    fGrabAbovePlane = 0.05;
    fGrabHeightMax = 1.5;
    fGrabHeightMin = 0.7;
}
    
CStorageRack::~CStorageRack()
{
    
}

void CStorageRack::GenTemp(bool inScale)
{
    int edgefold_v = 4;   // 竖直隔断的翻边宽度
    int edgefold_h = 4;   // 水平台面的翻边宽度
    for(int i=0;i<4;i++)
    {
        pRackTemp->h_partition_y[i] += RACK_TEMP_Y/2;
    }
    // 模板形状
    int left = pRackTemp->h_partition_y[0];
    for(int i=1;i<4;i++)
    {
        if(pRackTemp->h_partition_y[i] > left)
        {
            left = pRackTemp->h_partition_y[i];
        }
    }
    int right = pRackTemp->h_partition_y[0];
    for(int i=1;i<4;i++)
    {
        if(pRackTemp->h_partition_y[i] < right)
        {
            right = pRackTemp->h_partition_y[i];
        }
    }
    int height = pRackTemp->v_partition_z[0];
    for(int i=1;i<4;i++)
    {
        if(pRackTemp->v_partition_z[i] > height)
        {
            height = pRackTemp->v_partition_z[i];
        }
    }

    //生成水平托板正面翻边
    int front_x = RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
    for(int y=(right+edgefold_v);y<(left-edgefold_v);y++)
    {
        for(int i=0;i<4;i++)
        {
            int z_max = pRackTemp->v_partition_z[i];
            int z_min = z_max - edgefold_h;
            for(int z=z_min;z<z_max;z++)
            {
                pRackTemp->data[front_x][y][z] = 255;
            }
        }
    }

    //生成横向隔断
    for(int x=RACK_TEMP_X/2 - pRackTemp->d_partition_x/2; x<RACK_TEMP_X/2 + pRackTemp->d_partition_x/2; x ++)
    {
        for(int z=0;z<height;z++)
        {
            for(int i=0;i<4;i++)
            {
                pRackTemp->data[x][pRackTemp->h_partition_y[i]][z] = 255;
            }
        }
    }

    //生成纵向隔断
    for(int x=RACK_TEMP_X/2 - pRackTemp->d_partition_x/2; x<RACK_TEMP_X/2 + pRackTemp->d_partition_x/2; x ++)
    {
        for(int y=right;y<left;y++)
        {
            for(int i=0;i<4;i++)
            {
                pRackTemp->data[x][y][pRackTemp->v_partition_z[i]] = 255;
            }
        }
    }
    for(int z=0;z<height;z++)
    {
        //生成左前竖条面板
        for(int y=left;y<left+edgefold_v;y++)
        {
            int x =  RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
            pRackTemp->data[x][y][z] = 255;
        }
         //生成右前竖条面板
        for(int y=right-edgefold_v;y<right;y++)
        {
            int x =  RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
            pRackTemp->data[x][y][z] = 255;
        }
        //生成中间竖条面板
        // int mid_y = (left+right)/2;
        // for(int y=mid_y-edgefold_v/2;y<mid_y+edgefold_v/2;y++)
        // {
        //     int x =  RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
        //     pRackTemp->data[x][y][z] = 255;
        // }
    }

    //生成背板
    if(pRackTemp->b_back > 0)
    {
        int x = RACK_TEMP_X/2 + pRackTemp->d_partition_x/2;
        for(int y=right;y<left;y++)
        {
            for(int z=pRackTemp->v_partition_z[1];z<height;z++)
            {
               pRackTemp->data[x][y][z] = 255; 
            }
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