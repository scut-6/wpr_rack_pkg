#include "BoxDim.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <math.h>

#define CV_WIN

//static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZRGB>);     //用于迭代计算的点云
static float fBoxK = 2.0;
// 真实值
cv::Mat matOverBlack( BOX_DIM_HEIGHT,BOX_DIM_WIDTH, CV_8UC1, Scalar(0,255,0) );
cv::Mat matMid( BOX_DIM_HEIGHT,BOX_DIM_WIDTH, CV_8UC1, Scalar(0,255,0) );

// 盒子投影尺寸
static int nBoxWidht = 14*fBoxK;    // 盒子的窄边
static int nBoxLenght = 22*fBoxK;   // 盒子的长边
static cv::Mat matBox( BOX_TEMP_SIDE,BOX_TEMP_SIDE, CV_8UC1, Scalar(0,0,0) );
static cv::Mat matCurBox( BOX_TEMP_SIDE,BOX_TEMP_SIDE, CV_8UC1, Scalar(0,0,0) );
static cv::Mat matTmpBox( BOX_TEMP_SIDE,BOX_TEMP_SIDE, CV_8UC1, Scalar(0,0,0) );
cv::Mat matScale( OVER_SCALE_SIDE,OVER_SCALE_SIDE, CV_8UC1, Scalar(0,0,0) );

// 显示
cv::Mat matOver( BOX_DIM_HEIGHT,BOX_DIM_WIDTH, CV_8UC3, Scalar(0,255,0) );
cv::Mat matOverfliped( BOX_DIM_HEIGHT,BOX_DIM_WIDTH, CV_8UC3, Scalar(0,255,0) );

// 初始位置遍历
stICPResult testPosition[20][30];
// 单步匹配的遍历
stICPResult stepPostion[22];

CBoxDim::CBoxDim()
{
    memset(&boxPosition,0,sizeof(boxPosition));
    pRackTemp = new stRackTemp;
    //InitShape();
    InitTestPos();
    boxPosition.x_offset = 1.0;
    boxPosition.y_offset = 0.0;
    boxPosition.angle = 0.0;
    bBoxTracked = false;
    #ifdef CV_WIN  
    cv::namedWindow("matOver",WINDOW_AUTOSIZE);
	cv::resizeWindow("matOver",1200, 800);
    #endif 
}
    
CBoxDim::~CBoxDim()
{
    delete pRackTemp;
}

void CBoxDim::InitShape()
{
    // 三维模型的初始化
    for(int x=0;x<RACK_TEMP_X;x++)
        for(int y=0;y<RACK_TEMP_Y;y++)
            for(int z=0;z<RACK_TEMP_Z;z++)
            {
                pRackTemp->data[x][y][z] = 0;
            }
    memset(pRackTemp,0,sizeof(stRackTemp));   
}

void CBoxDim::InitCV()
{
    // 盒子二维模板初始化
    memset(((uchar*)matBox.data),0,BOX_TEMP_SIDE*BOX_TEMP_SIDE);
    for(int x=-nBoxWidht/2;x<nBoxWidht/2;x++)
        for(int y=-nBoxLenght/2;y<nBoxLenght/2;y++)
        {
            int tx = x+BOX_TEMP_SIDE/2;
            int ty = y+BOX_TEMP_SIDE/2;
            matBox.data[ty*BOX_TEMP_SIDE+tx] = 255;
        }

    // 渐变点初始化
    memset(matScale.data,0,OVER_SCALE_SIDE*OVER_SCALE_SIDE);
    for(int x=0;x<OVER_SCALE_SIDE;x++)
        for(int y=0;y<OVER_SCALE_SIDE;y++)
        {
            int dist = sqrt((x-OVER_SCALE_SIDE/2)*(x-OVER_SCALE_SIDE/2) + (y-OVER_SCALE_SIDE/2)*(y-OVER_SCALE_SIDE/2));
            if(dist < OVER_SCALE_SIDE/2)
                matScale.data[y*OVER_SCALE_SIDE+x] = 200 - (dist*2*200)/OVER_SCALE_SIDE ;
            else
                matScale.data[y*OVER_SCALE_SIDE+x] = 0;
        }
    matScale.data[-OVER_SCALE_SIDE*OVER_SCALE_SIDE/2+OVER_SCALE_SIDE/2] = 255;
}

void CBoxDim::InitTestPos()
{
    for(int x=0;x<20;x++)
        for(int y=0;y<30;y++)
        {
            testPosition[x][y].x_offset = 0.1*(float)x;
            testPosition[x][y].y_offset = 0.1*(float)y - 1.5;
            testPosition[x][y].angle = 0;
        }
    float angle_step = 0.02;
    stepPostion[0].x_offset=0; stepPostion[0].y_offset=0; stepPostion[0].angle=-angle_step;
    stepPostion[1].x_offset=0; stepPostion[1].y_offset=0; stepPostion[1].angle=angle_step;

    stepPostion[2].x_offset=0; stepPostion[2].y_offset=0; stepPostion[2].angle=-angle_step*6;
    stepPostion[3].x_offset=0.01; stepPostion[3].y_offset=0; stepPostion[3].angle=0;
    stepPostion[4].x_offset=0; stepPostion[4].y_offset=0; stepPostion[4].angle=angle_step*6;

    stepPostion[5].x_offset=-0; stepPostion[5].y_offset=0; stepPostion[5].angle=-angle_step*7;
    stepPostion[6].x_offset=-0.01; stepPostion[6].y_offset=0; stepPostion[6].angle=0;
    stepPostion[7].x_offset=-0; stepPostion[7].y_offset=0; stepPostion[7].angle=angle_step*7;

    stepPostion[8].x_offset=0; stepPostion[8].y_offset=0; stepPostion[8].angle=-angle_step*8;
    stepPostion[9].x_offset=0; stepPostion[9].y_offset=0.01; stepPostion[9].angle=0;
    stepPostion[10].x_offset=0; stepPostion[10].y_offset=0; stepPostion[10].angle=angle_step*8;

    stepPostion[11].x_offset=0; stepPostion[11].y_offset=-0; stepPostion[11].angle=-angle_step*9;
    stepPostion[12].x_offset=0; stepPostion[12].y_offset=-0.01; stepPostion[12].angle=0;
    stepPostion[13].x_offset=0; stepPostion[13].y_offset=-0; stepPostion[13].angle=angle_step*9;

    stepPostion[14].x_offset=0; stepPostion[14].y_offset=-0; stepPostion[14].angle=-angle_step*2;
    stepPostion[15].x_offset=0; stepPostion[15].y_offset=-0; stepPostion[15].angle=angle_step*2;
    stepPostion[16].x_offset=0; stepPostion[16].y_offset=-0; stepPostion[16].angle=-angle_step*3;
    stepPostion[17].x_offset=0; stepPostion[17].y_offset=-0; stepPostion[17].angle=angle_step*3;
    stepPostion[18].x_offset=0; stepPostion[18].y_offset=-0; stepPostion[18].angle=-angle_step*4;
    stepPostion[19].x_offset=0; stepPostion[19].y_offset=-0; stepPostion[19].angle=angle_step*4;
    stepPostion[20].x_offset=0; stepPostion[20].y_offset=-0; stepPostion[20].angle=-angle_step*5;
    stepPostion[21].x_offset=0; stepPostion[21].y_offset=-0; stepPostion[21].angle=angle_step*5;
}

void CBoxDim::LoadTemp(const char* filename)
{
    int fd = open(filename, O_RDONLY);

    if(fd == -1) 
    {
        printf("error is %s\n", strerror(errno));
        return;
    }

    read(fd, pRackTemp, sizeof(stRackTemp));
    close(fd);    
}

void CBoxDim::ScalePoint(int inX, int inY)
{
    int Range =OVER_SCALE_SIDE/2;
    int nScaleIndex = 0;
    for(int tx=inX-Range;tx<inX+Range;tx++)
        for(int ty=inY-Range;ty<inY+Range;ty++)
        {
            if(tx>=0&&tx<BOX_DIM_WIDTH&&ty>=0&&ty<BOX_DIM_HEIGHT)
            {
                if(matOverBlack.data[ty*BOX_DIM_WIDTH+tx] < matScale.data[nScaleIndex])
                    matOverBlack.data[ty*BOX_DIM_WIDTH+tx] = matScale.data[nScaleIndex];
            }
            nScaleIndex ++;
        }
}

void CBoxDim::RotBox(float inRotAngle , cv::Mat& inMat)
{
    float cos_a = cos(inRotAngle);
    float sin_a = sin(inRotAngle);

    memset(inMat.data,0,BOX_TEMP_SIDE*BOX_TEMP_SIDE);
    // 从matBox旋转到inMat
    for(int x=0;x<BOX_TEMP_SIDE;x++)
    {
        for(int y=0;y<BOX_TEMP_SIDE;y++)
        {
            if(matBox.data[y*BOX_TEMP_SIDE+x] == 255)
            {
                int r= BOX_TEMP_SIDE/2;
                int new_x = r + cos_a*(x-r)-sin_a*(y-r); //x1=cos(angle)*x-sin(angle)*y 
                int new_y = r + cos_a*(y-r)+sin_a*(x-r); //cos(angle)*y+sin(angle)*x

                if(new_x>=0 && new_x<BOX_TEMP_SIDE && new_y>=0 && new_y <BOX_TEMP_SIDE)
                    inMat.data[new_y*BOX_TEMP_SIDE+new_x] = 255;
            }
        }
    }
    // 填充inMat里的空白点
    for(int y=0;y<BOX_TEMP_SIDE;y++)
    {
        for(int x=1;x<BOX_TEMP_SIDE-1;x++)
        {
            if(inMat.data[y*BOX_TEMP_SIDE+x-1] > 0 && inMat.data[y*BOX_TEMP_SIDE+x+1] > 0)
                inMat.data[y*BOX_TEMP_SIDE+x] = 255;
        }
    }
}

float CBoxDim::GetMaxHeight()
{
    // int height = pRackTemp->v_partition_z[0];
    // for(int i=1;i<4;i++)
    // {
    //     if(pRackTemp->v_partition_z[i] > height)
    //     {
    //         height = pRackTemp->v_partition_z[i];
    //     }
    // }
    float res = 0;//(float)height/100;
    return res;
}

float CBoxDim::GetMinHeight()
{
    // int height = pRackTemp->v_partition_z[0];
    // for(int i=1;i<4;i++)
    // {
    //     if(pRackTemp->v_partition_z[i] < height)
    //     {
    //         height = pRackTemp->v_partition_z[i];
    //     }
    // }
    float res = 0;//(float)height/100;
    return res;
}

void CBoxDim::ShowOver()
{
#ifdef CV_WIN
	uchar* pDimOver = matOverBlack.data;

    // 降维的点云
	uchar *uc_pixel = matOver.data;
	int height = matOver.rows;
    int width = matOver.cols;
    for(int row=0; row < height; row++)
	{
        for(int col=0; col < width; col++)
		{
            uc_pixel[0] = (*pDimOver);
            uc_pixel[1] = (*pDimOver);
            uc_pixel[2] = (*pDimOver);

			pDimOver++;
			uc_pixel += 3;
        }
    }

    // 叠加匹配后的盒子位置
    RotBox(boxPosition.angle , matCurBox);
    for(int x=0;x<BOX_TEMP_SIDE;x++)
    {
        for(int y=0;y<BOX_TEMP_SIDE;y++)
        {
            if(matCurBox.data[y*BOX_TEMP_SIDE+x] == 255)
            {
                int nMapK = BOX_DIM_WIDTH/3;
                int nOverX = boxPosition.y_offset*nMapK + BOX_DIM_WIDTH/2 - (BOX_TEMP_SIDE/2) + x;
                int nOverY = boxPosition.x_offset*nMapK - (BOX_TEMP_SIDE/2) + y;
                if(nOverX>=0 && nOverX<BOX_DIM_WIDTH && nOverY>=0 && nOverY <BOX_DIM_HEIGHT)
                {
                    int nTmpIndex = nOverY*BOX_DIM_WIDTH+nOverX;
                    matOver.data[nTmpIndex*3 + 1] = 0;
                }
            }
        }
    }
		
	cv::flip(matOver, matOverfliped, -1);
	cv::imshow("matOver", matOverfliped);

	//cv::imshow("matBox", matBox);
    //cv::imshow("matCurBox", matCurBox);
	//cv::imshow("matScale", matScale);

    cv::waitKey(1);
#endif
}

int CBoxDim::CalPositionResult(stICPResult& inResult)
{
    int res = 0;
    RotBox(inResult.angle , matTmpBox);
    for(int x=0;x<BOX_TEMP_SIDE;x++)
    {
        for(int y=0;y<BOX_TEMP_SIDE;y++)
        {
            if(matTmpBox.data[y*BOX_TEMP_SIDE+x] == 255)
            {
                int nMapK = BOX_DIM_WIDTH/3;
                int nOverX = inResult.y_offset*nMapK + BOX_DIM_WIDTH/2 - (BOX_TEMP_SIDE/2) + x;
                int nOverY = inResult.x_offset*nMapK - (BOX_TEMP_SIDE/2) + y;
                if(nOverX>=0 && nOverX<BOX_DIM_WIDTH && nOverY>=0 && nOverY <BOX_DIM_HEIGHT)
                {
                    int nTmpIndex = nOverY*BOX_DIM_WIDTH+nOverX;
                    res += matOverBlack.data[nTmpIndex];
                }
            }
        }
    }
    return res;
}

// 第一帧全图搜索
void CBoxDim::FindBoxPosition()
{
    int nStep = 1;
    for(int x=0;x<20;x+=nStep)
        for(int y=0;y<30;y+=nStep)
        {
            int tmpResult = CalPositionResult(testPosition[x][y]);
            testPosition[x][y].result = tmpResult;
            //ROS_WARN("(%.2f , %.2f) res= %d",testPosition[x][y].x_offset ,testPosition[x][y].y_offset ,tmpResult );
        }
    stICPResult bestResult = testPosition[0][0];
    bestResult.result = -1;
    for(int x=0;x<20;x++)
        for(int y=0;y<30;y++)
        {
            if(testPosition[x][y].result > bestResult.result)
                bestResult = testPosition[x][y];
        }
    boxPosition = bestResult;
}

// 单步匹配
bool CBoxDim::OneMoreStep()
{
    bool bNewStep = false;

    stICPResult bestResult = boxPosition;
    bestResult.result = CalPositionResult(bestResult);
    stICPResult tmpResult;
    for(int i=0;i<22;i++)
    {
        tmpResult.x_offset = boxPosition.x_offset + stepPostion[i].x_offset;
        tmpResult.y_offset = boxPosition.y_offset + stepPostion[i].y_offset;
        tmpResult.angle = boxPosition.angle + stepPostion[i].angle;
        tmpResult.result = CalPositionResult(tmpResult);
        if(tmpResult.result > bestResult.result)
        {
            // 找到一个匹配度更高的位置，更新bestResult
            bestResult = tmpResult;
            bNewStep = true;
        }
    }

    if(bNewStep == true)
    {
        // 更新盒子的最终位置
        boxPosition = bestResult;
    }

    return bNewStep;
}

bool CBoxDim::CalBoxPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud)
{
    bool res = false;
    memset(matOverBlack.data, 0, BOX_DIM_WIDTH*BOX_DIM_HEIGHT);

    // 将三维点云降维成俯视平面图
    int nMidX = BOX_DIM_WIDTH/2;
    int nNumPoints = inCloud->points.size();
    for(int i=0;i<nNumPoints;i++)
    {
        int nMapK = BOX_DIM_WIDTH/3;
        int x = inCloud->points[i].y * nMapK + nMidX;
        int y = inCloud->points[i].x * nMapK;
        if(x >=0 && x<BOX_DIM_WIDTH && y>=0 && y<BOX_DIM_HEIGHT)
        {
            matOverBlack.data[y*BOX_DIM_WIDTH+x] = 0xff;
        }
    }

    // 渐变处理
    for(int x=0;x<BOX_DIM_WIDTH;x++)
    {
        for(int y=0;y<BOX_DIM_HEIGHT;y++)
        {
            if(matOverBlack.data[y*BOX_DIM_WIDTH+x] == 255)
            {
                ScalePoint(x,y);
            }
        }
    }

    // 旋转盒子俯视图模板，与渐变图进行匹配
    if(bBoxTracked == false)
    {
        FindBoxPosition();
        if(boxPosition.result > 0)
            bBoxTracked = true;
    }
    else
    {
        // 新的一帧，重新计算当前位置的匹配值
        boxPosition.result = CalPositionResult(boxPosition);
    }

    bool bNewStep = true;
    while(bNewStep == true)
        bNewStep = OneMoreStep();
    
    /////////////////////////////////////////
    printf("模板旋转超过45度！转回0度！\n");
    if(fabs(boxPosition.angle) > 3.14/2)
    {
        boxPosition.angle = 0;
    }
    /////////////////////////////////////////

    //int result = CalPositionResult(boxPosition);
    //ROS_WARN("box (%.2f , %.2f) r= %d",boxPosition.x_offset ,boxPosition.y_offset ,boxPosition.result );
    
    ShowOver();
    res = true;

    return res;
}