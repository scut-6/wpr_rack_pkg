#include "MobileRack.h"

CMobileRack::CMobileRack()
{
    fCulling_v = 0.03;
    fCulling_h = 0.08;
    fPlaceDistSide = 0.2;
    fPlaceDistObj = 0.20;
    fGrabAbovePlane = 0.08;
    fGrabHeightMax = 1.0;
    fGrabHeightMin = 0.7;
}
    
CMobileRack::~CMobileRack()
{
    
}

void CMobileRack::GenTemp(bool inScale)
{
     ROS_INFO("GenTemp");
    // 几个尺寸变量
    int beam_width = 3; // 立柱型材的宽度
    int edgefold = 3;   // 载物托盘的翻边宽度

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
    
    //生成载物托板
    for(int x=(RACK_TEMP_X/2 - pRackTemp->d_partition_x/2); x<(RACK_TEMP_X/2 + pRackTemp->d_partition_x/2); x ++)
    {
        for(int y=(right);y<(left);y++)
        {
            for(int i=0;i<4;i++)
            {
                pRackTemp->data[x][y][pRackTemp->v_partition_z[i]] = 255;
            }
        }
    }
    //生成载物托板正面翻边
    int front_x = RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
    for(int y=(right+beam_width);y<(left-beam_width);y++)
    {
        for(int i=0;i<4;i++)
        {
            int z_max = pRackTemp->v_partition_z[i];
            int z_min = z_max - edgefold;
            for(int z=z_min;z<z_max;z++)
            {
                pRackTemp->data[front_x][y][z] = 255;
            }
        }
    }
    //生成侧面翻边
    for(int x=(RACK_TEMP_X/2 - pRackTemp->d_partition_x/2); x<(RACK_TEMP_X/2 + pRackTemp->d_partition_x/2); x ++)
    {
        for(int i=0;i<4;i++)
        {
            int z_max = pRackTemp->v_partition_z[i];
            int z_min = z_max - edgefold;
            for(int z=z_min;z<z_max;z++)
            {
                pRackTemp->data[x][right][z] = 255;
                pRackTemp->data[x][left][z] = 255;
            }
        }
    }

    //生成左前立柱
    for(int z=pRackTemp->v_partition_z[0]-edgefold;z<height;z++)
    {
        for(int y=left-beam_width;y<left;y++)
        {
            int x =  RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
            pRackTemp->data[x][y][z] = 255;
            //后侧面
            // x += beam_width;
            // pRackTemp->data[x][y][z] = 255;
        }
        int x_min =  RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
        for(int x = x_min;x<x_min+beam_width;x++)
        {
            int y =  left;
            pRackTemp->data[x][y][z] = 255;
            // 内侧面
            // y -= beam_width;
            // pRackTemp->data[x][y][z] = 255;
        }
    }

    //生成右前立柱
    for(int z=pRackTemp->v_partition_z[0]-edgefold;z<height;z++)
    {
        for(int y=right;y<right+beam_width;y++)
        {
            int x =  RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
            pRackTemp->data[x][y][z] = 255;
            // x += beam_width;
            // pRackTemp->data[x][y][z] = 255;
        }
        int x_min =  RACK_TEMP_X/2 - pRackTemp->d_partition_x/2;
        for(int x = x_min;x<x_min+beam_width;x++)
        {
            int y =  right;
            pRackTemp->data[x][y][z] = 255;
            // y += beam_width;
            // pRackTemp->data[x][y][z] = 255;
        }
    }

    //生成左后立柱
    for(int z=pRackTemp->v_partition_z[0]-edgefold;z<height;z++)
    {
        for(int y=left-beam_width;y<left;y++)
        {
            int x =  RACK_TEMP_X/2 + pRackTemp->d_partition_x/2 - beam_width;
            pRackTemp->data[x][y][z] = 255;
            // x += beam_width;
            // pRackTemp->data[x][y][z] = 255;
        }
        int x_min =  RACK_TEMP_X/2 + pRackTemp->d_partition_x/2 - beam_width;
        for(int x = x_min;x<x_min+beam_width;x++)
        {
            int y =  left;
            pRackTemp->data[x][y][z] = 255;
            // y -= beam_width;
            // pRackTemp->data[x][y][z] = 255;
        }
    }

    //生成右后立柱
    for(int z=pRackTemp->v_partition_z[0]-edgefold;z<height;z++)
    {
        for(int y=right;y<right+beam_width;y++)
        {
            int x =  RACK_TEMP_X/2 + pRackTemp->d_partition_x/2 - beam_width;
            pRackTemp->data[x][y][z] = 255;
            // x += beam_width;
            // pRackTemp->data[x][y][z] = 255;
        }
        int x_min =  RACK_TEMP_X/2 + pRackTemp->d_partition_x/2 - beam_width;
        for(int x = x_min;x<x_min+beam_width;x++)
        {
            int y =  right;
            pRackTemp->data[x][y][z] = 255;
            // y += beam_width;
            // pRackTemp->data[x][y][z] = 255;
        }
    }

    //生成背板
    int z_min = pRackTemp->v_partition_z[0];
    for(int i=1;i<4;i++)
    {
        if(pRackTemp->v_partition_z[i]<z_min && pRackTemp->v_partition_z[i] > 0)
        {
            z_min = pRackTemp->v_partition_z[i];
        }
    }
    if(pRackTemp->b_back > 0)
    {
        int x = RACK_TEMP_X/2 + pRackTemp->d_partition_x/2;
        for(int y=(right+beam_width);y<(left-beam_width);y++)
        {
            for(int z=z_min;z<height;z++)
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