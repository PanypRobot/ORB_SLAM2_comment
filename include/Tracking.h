/**
 * @file Tracking.h
 * @author guoqing (1337841346@qq.com)
 * @brief 追踪线程
 * @version 0.1
 * @date 2019-02-21
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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

/**
 * @brief  追踪当前帧功能
 * 
 */
class Tracking
{  

public:
    /**
     * @brief 构造函数
     * 
     * @param[in] pSys              系统实例 
     * @param[in] pVoc              字典指针
     * @param[in] pFrameDrawer      帧绘制器
     * @param[in] pMapDrawer        地图绘制器
     * @param[in] pMap              地图句柄
     * @param[in] pKFDB             关键帧数据库句柄
     * @param[in] strSettingPath    配置文件路径
     * @param[in] sensor            传感器类型
     */
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // 下面的函数都是对不同的传感器输入的图像进行处理(转换成为灰度图像),并且调用Tracking线程
    /**
     * @brief 处理双目输入
     * 
     * @param[in] imRectLeft    左目图像
     * @param[in] imRectRight   右目图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);

    /**
     * @brief 处理RGBD输入的图像
     * 
     * @param[in] imRGB         彩色图像
     * @param[in] imD           深度图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);

    /**
     * @brief 处理单目输入图像
     * 
     * @param[in] im            图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    /**
     * @brief 设置局部地图句柄
     * 
     * @param[in] pLocalMapper 局部建图器
     */
    void SetLocalMapper(LocalMapping* pLocalMapper);

    /**
     * @brief 设置回环检测器句柄
     * 
     * @param[in] pLoopClosing 回环检测器
     */
    void SetLoopClosing(LoopClosing* pLoopClosing);

    /**
     * @brief 设置可视化查看器句柄
     * 
     * @param[in] pViewer 可视化查看器
     */
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal length should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    /**
     * @brief 
     * 
     * @param[in] strSettingPath 配置文件路径
     */
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    /**
     * @brief 设置进入仅定位模式
     * 
     * @param[in] flag 设置仅仅进行跟踪的标志位
     */
    void InformOnlyTracking(const bool &flag);


public:

    // 跟踪状态类型, 枚举类型
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,        //系统没有准备好的状态,一般就是在启动后加载配置文件和词典文件时候的状态
        NO_IMAGES_YET=0,            //当前无图像
        NOT_INITIALIZED=1,          //有图像但是没有完成初始化
        OK=2,                       //正常时候的工作状态
        LOST=3                      //系统已经跟丢了的状态
    };

    //跟踪状态
    eTrackingState mState;
    //上一帧的跟踪状态. 用于FrameDrawer中的绘制
    //在Track进程中每次处理一张图后, 会把当前追踪状态赋给上一帧状态, 即mLastProcessedState=mState
    eTrackingState mLastProcessedState;

    //传感器类型, MONOCULAR, STEREO, RGBD
    //在system.h中定义, 应该是eSensor枚举类型, 这里简单实用int
    int mSensor;

    //追踪线程中的当前帧
    Frame mCurrentFrame;

    //当前帧的灰度图像
    //在双目输入时, mImGray为左侧图像的灰度图, imGrayRight为右侧图像的灰度图
    //在RGBD输入时, mImGray为灰度图像, imDepth为深度图像
    cv::Mat mImGray;


    // Initialization Variables (Monocular)
    // 初始化时前两帧相关变量
    ///之前的匹配
    std::vector<int> mvIniLastMatches;
    ///初始化阶段中,当前帧中的特征点和参考帧中的特征点的匹配关系
    std::vector<int> mvIniMatches;// 跟踪初始化时前两帧之间的匹配
    ///在初始化的过程中,保存参考帧中的特征点
    std::vector<cv::Point2f> mvbPrevMatched;
    ///初始化过程中匹配后进行三角化得到的空间点
    std::vector<cv::Point3f> mvIniP3D;
    ///初始化过程中的参考帧
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    ///所有的参考关键帧的位姿;看上面注释的意思,这里存储的也是相对位姿
    list<cv::Mat> mlRelativeFramePoses;
    ///参考关键帧
    list<KeyFrame*> mlpReferences;
    ///所有帧的时间戳  //? 还是关键帧的时间戳?
    list<double> mlFrameTimes;
    ///是否跟丢的标志
    list<bool> mlbLost;

    // 标记, 当前系统是处于SLAM状态还是纯定位状态
    // True -> LOCALIZATION
    // False -> SLAM MODE
    // 当Local mapping被关闭时是True
    bool mbOnlyTracking;

    /** 
     * @brief 整个系统进行复位操作
     */
    void Reset();

protected:

    // 相对输入传感器独立
    /** @brief 主追踪进程 */
    void Track();

    /** @brief 在双目输入和RGBD输入时所做的初始化,主要是产生初始地图 */
    void StereoInitialization();

    /** @brief 单目输入的时候所进行的初始化操作 */
    void MonocularInitialization();
   
    /** @brief 单目输入的时候生成初始地图 */
    void CreateInitialMapMonocular();

    /**
     * @brief 检查上一帧中的MapPoints是否被替换
     * 
     * Local Mapping线程可能会将关键帧中某些MapPoints进行替换，由于tracking中需要用到mLastFrame，这里检查并更新上一帧中被替换的MapPoints
     * @see LocalMapping::SearchInNeighbors()
     */

    void CheckReplacedInLastFrame();
    /**
     * @brief 对参考关键帧的MapPoints进行跟踪
     * 
     * 1. 计算当前帧的词包，将当前帧的特征点分到特定层的nodes上
     * 2. 对属于同一node的描述子进行匹配
     * 3. 根据匹配对估计当前帧的姿态
     * 4. 根据姿态剔除误匹配
     * @return 如果匹配数大于10，返回true
     */

    bool TrackReferenceKeyFrame();
    /**
     * @brief 双目或rgbd摄像头根据深度值为上一帧产生新的MapPoints
     *
     * 在双目和rgbd情况下，选取一些深度小一些的点（可靠一些） \n
     * 可以通过深度值产生一些新的MapPoints
     */

    void UpdateLastFrame();
    
    /**
     * @brief 根据匀速度模型对上一帧的MapPoints进行跟踪
     * 
     * 1. 非单目情况，需要对上一帧产生一些新的MapPoints（临时）     
     * 2. 将上一帧的MapPoints投影到当前帧的图像平面上，在投影的位置进行区域匹配
     * 3. 根据匹配对估计当前帧的姿态
     * 4. 根据姿态剔除误匹配
     * @return 如果匹配数大于10，返回true
     * @see V-B Initial Pose Estimation From Previous Frame
     */
    bool TrackWithMotionModel();

    /** @brief 重定位模块 */
    bool Relocalization();

    /**
     * @brief 更新局部地图 LocalMap
     *
     * 局部地图包括：共视关键帧、临近关键帧及其子父关键帧，由这些关键帧观测到的MapPoints
     */
    void  ();
    
    /**
     * @brief 更新局部地图点（来自局部关键帧）
     * 
     */
    void UpdateLocalPoints();

   /**
     * @brief 更新局部关键帧
     * 方法是遍历当前帧的MapPoints，将观测到这些MapPoints的关键帧和相邻的关键帧及其父子关键帧，作为mvpLocalKeyFrames
     * Step 1：遍历当前帧的MapPoints，记录所有能观测到当前帧MapPoints的关键帧 
     * Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有三个策略
     * Step 2.1 策略1：能观测到当前帧MapPoints的关键帧作为局部关键帧 （将邻居拉拢入伙）
     * Step 2.2 策略2：遍历策略1得到的局部关键帧里共视程度很高的关键帧，将他们的家人和邻居作为局部关键帧
     * Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
     */
    void UpdateLocalKeyFrames();

    /**
     * @brief 对Local Map的MapPoints进行跟踪
     * Step 1：更新局部关键帧 mvpLocalKeyFrames 和局部地图点 mvpLocalMapPoints
     * Step 2：在局部地图中查找与当前帧匹配的MapPoints, 其实也就是对局部地图点进行跟踪
     * Step 3：更新局部所有MapPoints后对位姿再次优化
     * Step 4：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
     * Step 5：根据跟踪匹配数目及回环情况决定是否跟踪成功
     * @return true         跟踪成功
     * @return false        跟踪失败
     */
    bool TrackLocalMap();
    /**
     * @brief 对 Local MapPoints 进行跟踪
     * 
     * 在局部地图中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
     */
    void SearchLocalPoints();

    /**
     * @brief 判断当前帧是否为关键帧
     * @return true if needed
     */
    bool NeedNewKeyFrame();
    /**
     * @brief 创建新的关键帧
     *
     * 对于非单目的情况，同时创建新的MapPoints
     */
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    ///当进行纯定位时才会有的一个变量,为false表示该帧匹配了很多的地图点,跟踪是正常的;如果少于10个则为true,表示快要完蛋了
    bool mbVO;

    //其他线程的指针
    ///局部建图器句柄
    LocalMapping* mpLocalMapper;
    ///回环检测器句柄
    LoopClosing* mpLoopClosing;

    // ORB特征提取器
    // mpORBextractorLeft 对于单目, 双目, RGBD都要用到
    // mpORBextractorRight 是双目的另一个提取器
    // 声明ORB特征提取器
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    // 单目初始化的时候使用的特征点提取器, 其提取到的特征点个数是一般提取器两倍
    // 另外单目在初始化的时候使用mpIniORBextractor而不是mpORBextractorLeft，
    // mpIniORBextractor属性中提取的特征点个数是mpORBextractorLeft的两倍
    ORBextractor* mpIniORBextractor;

    //BoW词袋模型相关
    //ORB特征字典
    ORBVocabulary* mpORBVocabulary;
    //当前系统运行的时候,关键帧所产生的数据库
    KeyFrameDatabase* mpKeyFrameDB;

    //Mono 单目初始器
    Initializer* mpInitializer;

    //Local map 局部地图相关
    //参考关键帧
    KeyFrame* mpReferenceKF;// 当前关键帧就是参考帧
    //局部关键帧vector
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    //局部地图点vector
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    //System 系统相关
    //指向系统实例的指针 
    System* mpSystem;
    
    //Drawers 可视化查看器相关
    //查看器对象句柄
    Viewer* mpViewer;
    ///帧绘制器句柄
    FrameDrawer* mpFrameDrawer;
    ///地图绘制器句柄
    MapDrawer* mpMapDrawer;

    //Map
    //(全局)地图句柄
    Map* mpMap;

    //Calibration matrix   相机的参数矩阵相关
    //相机的内参数矩阵
    cv::Mat mK;
    //相机的去畸变矩阵
    cv::Mat mDistCoef;
    //相机的基线长度 * 相机的焦距
    float mbf;

    // New KeyFrame rules (according to fps)
    // 新建关键帧和重定位中用来判断最小最大时间间隔，和帧率有关
    int mMinFrames;
    int mMaxFrames;         

    //用于区分远点/近点的阈值. 
    //近点认为可信度比较高, 远点则要求在两个关键帧中得到匹配
    float mThDepth;

    //深度缩放因子
    //链接深度值和具体深度值的参数.只对RGBD输入有效, 对于某些数据集（例如 TUM），深度图值会进行缩放
    float mDepthMapFactor;

    //Current matches in frame
    ///当前帧中的进行匹配的内点,将会被不同的函数反复使用
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    // 上一关键帧
    KeyFrame* mpLastKeyFrame;
    // 上一帧
    Frame mLastFrame;
    // 上一个关键帧的ID
    unsigned int mnLastKeyFrameId;
    // 上一次重定位的那一帧的ID
    unsigned int mnLastRelocFrameId;

    //运动模式
    //用于恒速跟踪中
    cv::Mat mVelocity;

    //RGB图像的颜色通道顺序
    //true->RGB, false->BGR, 当为灰度时忽略
    //由配置文件中的nRGB得到
    bool mbRGB;

    //临时的地图点,用于提高双目和RGBD摄像头的帧间效果
    //用完之后就失效
    list<MapPoint*> mlpTemporalPoints;
};  //class Tracking

} //namespace ORB_SLAM

#endif // TRACKING_H
