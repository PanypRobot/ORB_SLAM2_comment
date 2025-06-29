/**
 * @file MapDrawer.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 绘制地图点
 * @version 0.1
 * @date 2019-02-19
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

//关于gl相关的函数, 可直接加上msdn关键词google搜索

namespace ORB_SLAM2
{

//构造函数
MapDrawer::MapDrawer(   Map* pMap,                                  //地图
                        const string &strSettingPath)
                    :mpMap(pMap)   //配置文件路径
{
    //从配置文件中读取设置的
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    // 绘制这些部件的参数, 从yaml配置文件的Viewer中读得
    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    //取出所有的地图点
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    //取出mvpReferenceMapPoints，也即局部地图d点
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    //将vpRefMPs从vector容器类型转化为set容器类型，便于使用set::count快速统计 - 我觉得称之为"重新构造"可能更加合适一些
    //补充, set::count用于返回集合中为某个值的元素的个数
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())   //没有地图点
        return;

    // 对于所有地图点

    // Step 1 绘制所有的地图点(不包括局部地图点, 大小为2个像素, 黑色)
    
    glPointSize(mPointSize); // mPointSize从yaml中读取
    // glBegin和glEnd之间可以调用一些gl相关函数
    // GL_POINTS: 把每一个顶点作为一个点进行处理，顶点n即定义了点n，共绘制N个点
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);         //黑色

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)  //遍历地图点vpMPs[i], i为索引
    {
        // 不包括坏点, 和ReferenceMapPoints（局部地图点）
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i])) // 是坏点, 或存在于局部地图点的set中, 就continue掉
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        // glVertex3f为OpenGL的函数, 创建一个顶点, 三个坐标值分别对应pos的三个坐标值
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    // Step 2 绘制所有的局部地图点(大小为2个像素, 红色)
    glPointSize(mPointSize);
    // glBegin和glEnd之间调用一些gl函数
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);     //红色

    // 遍历存储局部地图点的set
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad()) //坏点跳过
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        //创建一个顶点, 三个坐标值分别对应pos的三个坐标值
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    // 历史关键帧图标
    // mKeyFrameSize表示宽度占总宽度比例(读取自yaml, EuRoC中为0.05)
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    // step 1: 取出地图中所有的关键帧
    // 赋值副本给vpKFs, 为const类型
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    // step 2: 显示所有关键帧图标

    //通过显示界面选择是否显示历史关键帧图标
    if(bDrawKF) // 来自viewer的menu, menuShowKeyFrames
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            // 返回逆矩阵 并做转置, OpenGL中的矩阵为列优先存储
            cv::Mat Twc = pKF->GetPoseInverse().t();

            // ???对于glPushMatrix()和glMultMatrixf()理解不清晰???
            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));

            // 设置绘制图形时的线宽
            glLineWidth(mKeyFrameLineWidth);

            // 设置蓝色(关键帧图标)
            glColor3f(0.0f,0.0f,1.0f);
            // 用线将下面的顶点两两相连

            // 绘制相机坐标系的原点和四个方向上的点,表示相机的位置和朝向
            // 通过绘制相机视锥体的六个面, 将相机视野的范围可视化出来            
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);

            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);

            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);

            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    // step 3: 显示所有关键帧的Essential Graph (本征图)

    // 共视图中存储了所有关键帧的共视关系
    // 本征图中对边进行了优化,保存了所有节点,只存储了具有较多共视点的边,用于进行优化
    // 生成树则进一步进行了优化,保存了所有节点,但是值保存具有最多共视地图点的关键帧的边
    
    //通过显示界面选择是否显示关键帧连接关系
    if(bDrawGraph)  // 来自viewer的menu, menuShowGraph
    {
        //设置绘制图形时的线宽
        glLineWidth(mGraphLineWidth);

        glColor4f(0.0f,1.0f,0.0f,0.6f); // 设置共视图连接线为绿色，透明度为0.6f
        glBegin(GL_LINES);  // 绘制线条时,默认是按照添加顺序,每两个点之间绘制一条直线


        // 共视图 生成树 闭环 的绘制都处于同一个大的关键帧遍历下
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph (共视图)
            // step 3.1 共视程度比较高的共视关键帧用线连接
            // 遍历每一个关键帧, 得到与该关键帧连接权重大于100的关键帧(共视程度比较高)
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            // 遍历每一个关键帧, 得到它在世界坐标系下的相机坐标
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    //单向绘制
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree (生成树)
            // step 3.2 连接最小生成树 (PS: 我觉得这里并不是权值最小,而是其中的边对于其他的图来讲是最少的)
            //TODO 这个部分的理论知识还不是很了解
            KeyFrame* pParent = vpKFs[i]->GetParent(); //每一个关键帧的父关键帧
            if(pParent)
            // ?? 不明白这里用if去判断一个类是什么意思??
            // 关键帧在创建的时候, mpParent应该是NULL, if(NULL)应该是false
            {
                cv::Mat Owp = pParent->GetCameraCenter();   //相机中心
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            // step 3.3 连接闭环时形成的连接关系
            // ????
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges(); //获取和当前关键帧形成闭环关系的关键帧
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++) //遍历闭环
            {
                if((*sit)->mnId<vpKFs[i]->mnId) //闭环帧id 小于 当前帧id, 就跳过
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();    //相机中心
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}


void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    // 相机模型大小: 宽度占总宽度比例为0.08(读取自yaml, EuRoC中为0.05)
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    //百度搜索：glPushMatrix 百度百科
    glPushMatrix();

    //将4*4的矩阵Twc.m右乘一个当前矩阵
    //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
    //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
    //一个是整型,一个是浮点数类型
#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    //设置绘制图形时的线宽
    glLineWidth(mCameraLineWidth);
    //设置当前颜色为绿色(相机图标显示为绿色)
    glColor3f(0.0f,1.0f,0.0f);
    //用线将下面的顶点两两相连
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);

    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);

    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);

    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

//设置当前帧相机的位姿, 设置这个函数是因为要处理多线程的操作
void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone(); // 将参数传递给当前Drawer的成员变量mCameraPose
}

// 将相机位姿mCameraPose由Mat类型转化为OpenGlMatrix类型
// 在Viewer中用到了
void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) 
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            // 从mCameraPose中提取出R, t矩阵
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        // 给传入的引用M赋值, 其类型为pangolin::OpenGlMatrix
        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
