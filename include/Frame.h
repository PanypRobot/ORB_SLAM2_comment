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

/**
 * @file Frame.h
 * @author guoqing (1337841346@qq.com)
 * @brief ORB-SLAM2 中，帧的实现
 * @version 0.1
 * @date 2019-01-03 
 */

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
	
/**
 * @name 定义一帧中有多少个图像网格
 * @{
 */

/**
 * @brief 网格的行数
 * 
 */
#define FRAME_GRID_ROWS 48
/**
 * @brief 网格的列数
 * 
 */
#define FRAME_GRID_COLS 64

/** @} */

class MapPoint;
class KeyFrame;

/**
 * @brief 帧
 */
class Frame
{
public:
	
    /**
     * @brief Construct a new Frame object without parameter. 
     * 
     */
    Frame();

    // Copy constructor. 拷贝构造函数
    /**
     * @brief 拷贝构造函数 
     * @details 复制构造函数, mLastFrame = Frame(mCurrentFrame) \n
     * 如果不是自定以拷贝函数的话，系统自动生成的拷贝函数对于所有涉及分配内存的操作都将是浅拷贝 \n
     * @param[in] frame 引用
     * @note 另外注意，调用这个函数的时候，这个函数中隐藏的this指针其实是指向目标帧的
     */
    Frame(const Frame &frame);

    

    /**
     * @brief 双目 帧构造函数
     * 
     * @param[in] imLeft            左目图像
     * @param[in] imRight           右目图像
     * @param[in] timeStamp         时间戳
     * @param[in] extractorLeft     左目图像特征点提取器句柄
     * @param[in] extractorRight    右目图像特征点提取器句柄
     * @param[in] voc               ORB字典句柄
     * @param[in] K                 相机内参矩阵
     * @param[in] distCoef          相机去畸变参数
     * @param[in] bf                相机基线长度和焦距的乘积
     * @param[in] thDepth           远点和近点的深度区分阈值
     *  
     */
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);


    /**
     * @brief RGBD 帧构造函数
     * 
     * @param[in] imGray        对RGB图像灰度化之后得到的灰度图像
     * @param[in] imDepth       深度图像
     * @param[in] timeStamp     时间戳
     * @param[in] extractor     特征点提取器句柄
     * @param[in] voc           ORB特征点词典的句柄
     * @param[in] K             相机的内参数矩阵
     * @param[in] distCoef      相机的去畸变参数
     * @param[in] bf            baseline*bf
     * @param[in] thDepth       远点和近点的深度区分阈值
     */
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    /**
     * @brief 单目 帧构造函数
     * 
     * @param[in] imGray                            //灰度图
     * @param[in] timeStamp                         //时间戳
     * @param[in & out] extractor                   //ORB特征点提取器的句柄
     * @param[in] voc                               //ORB字典的句柄
     * @param[in] K                                 //相机的内参数矩阵
     * @param[in] distCoef                          //相机的去畸变参数
     * @param[in] bf                                //baseline*f
     * @param[in] thDepth                           //区分远近点的深度阈值
     */
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);


    /**
     * @brief 提取图像的ORB特征, 提取的关键点存放在mvKeys, 描述子存放在mDescriptors. 对于flag, 0是左图, 1是右图
     * 
     * @param[in] flag          标记是左图还是右图。0：左图  1：右图
     * @param[in] im            等待提取特征点的图像
     */
    void ExtractORB(int flag, const cv::Mat &im);


    /**
     * @brief 计算词袋模型, 存放于mBowVec中
     * @details 计算词包 mBowVec 和 mFeatVec ，其中 mFeatVec 记录了属于第i个node（在第4层）的ni个描述子
     * @see CreateInitialMapMonocular() TrackReferenceKeyFrame() Relocalization()
     */
    void ComputeBoW();


    // 设置相机位姿
    // 用Tcw更新mTcw
    /**
     * @brief 用 Tcw 更新 mTcw 以及类中存储的一系列位姿
     * 
     * @param[in] Tcw 从世界坐标系到当前帧相机位姿的变换矩阵
     */
    void SetPose(cv::Mat Tcw);


    // 根据相机位姿计算 rotation, translation and camera center matrices
    /**
     * @brief 根据相机位姿,计算相机的旋转,平移和相机中心等矩阵.
     * @details 其实就是根据Tcw计算mRcw、mtcw和mRwc、mOw.
     */
    void UpdatePoseMatrices();


    // 返回当前相机中心
    /**
     * @brief 返回位于当前帧位姿时,相机的中心
     * 
     * @return cv::Mat 相机中心在世界坐标系下的3D点坐标
     */
    inline cv::Mat GetCameraCenter()
	{
        return mOw.clone();
    }

    // 返回 从世界到相机旋转 所相反的旋转
    /**
     * @brief Get the Rotation Inverse object
     * mRwc存储的是从当前相机坐标系到世界坐标系所进行的旋转，而我们一般用的旋转则说的是从世界坐标系到当前相机坐标系的旋转
     * @return 返回从当前帧坐标系到世界坐标系的旋转
     */
    inline cv::Mat GetRotationInverse()
	{
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum(截椎体) of the camera
    // and fill variables of the MapPoint to be used by the tracking
    /**
     * @brief 判断路标点是否在视野中
     * 步骤
     * Step 1 获得这个地图点的世界坐标
     * Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，返回false
     * Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
     * Step 4 关卡三：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
     * Step 5 关卡四：计算当前视角和“法线”夹角的余弦值, 若小于设定阈值，返回false
     * Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
     * Step 7 记录计算得到的一些参数
     * @param[in] pMP                       当前地图点
     * @param[in] viewingCosLimit           夹角余弦，用于限制地图点和光心连线和法线的夹角
     * @return true                         地图点合格，且在视野内
     * @return false                        地图点不合格，抛弃
     */
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    /**
     * @brief 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
     * 
     * @param[in] kp                    给定的特征点
     * @param[in & out] posX            特征点所在网格坐标的横坐标
     * @param[in & out] posY            特征点所在网格坐标的纵坐标
     * @return true                     如果找到特征点所在的网格坐标，返回true
     * @return false                    没找到返回false
     */
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    /**
     * @brief 找到在 以x,y为中心,半径为r的圆形内且金字塔层级在[minLevel, maxLevel]的特征点
     * 
     * @param[in] x                     特征点坐标x
     * @param[in] y                     特征点坐标y
     * @param[in] r                     搜索半径 
     * @param[in] minLevel              最小金字塔层级
     * @param[in] maxLevel              最大金字塔层级
     * @return vector<size_t>           返回搜索到的候选匹配点id
     */
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    /**
     * @brief 计算双目图像之间的匹配关系 \n
     * @details 如果对于一对特征点确定有匹配关系存在,那么这个特征点在空间中的深度将会被计算,并且和左特征点相对应的右特征点的坐标将会被存储. 
     * \n 说白了，就是为左图的每一个特征点在右图中找到匹配点
     * \n 方法是根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 
     * \n 这里所说的SAD是一种双目立体视觉匹配算法，可参考[https://blog.csdn.net/u012507022/article/details/51446891]
     * \n 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 
     * \n 这里所谓的亚像素精度，就是使用这个拟合得到一个小于一个单位像素的修正量，这样可以取得更好的估计结果，计算出来的点的深度也就越准确
     * \n 匹配成功后会更新 Frame::mvuRight (ur) 和 Frame::mvDepth (Z)
     */
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
	
    /**
     * @brief 对于RGBD输入,如果某个特征点的深度值有效,那么这里将会反向计算出假想的"右目图像中对应特征点的坐标". 
     * @detials 博客[https://www.cnblogs.com/panda1/p/7001052.html]中说，这个是将地图点和其深度对应起来.
	 * 是不是可以这样理解，为了能够将特征点反投影到三维空间中得到其在相机坐标系以及在世界坐标系下的坐标，我们需要获得它
	 * 在当前相机下的深度。对于双目相机，我们是通过计算左侧图像中特征点在右图中的坐标，然后计算其深度；对于RGBD图像我们可以直接
	 * 从深度图像上获得特征点的深度，不过为了处理上的一致这里使用这个深度计算了彩色图像（左图）中的特征点在假想的“右图”中的
	 * 坐标。这就是这个函数的工作.     
     * 
     * @param[in] imDepth 深度图像
     */
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    /**
     * @brief 当某个特征点的深度信息或者双目信息有效时,将它反投影到三维世界坐标系中
     * 
     * @param[in] i     特征点的ID
     * @return cv::Mat  反投影后得到的特征点的反投影点的世界坐标
     */
    cv::Mat UnprojectStereo(const int &i);

public:

    // 用于重定位的ORB特征字典
    ORBVocabulary* mpORBvocabulary;

    // ORB特征提取器, 其中右侧的提取器句柄只会在双目输入的情况中才会被用到
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // 帧的时间戳
    double mTimeStamp;


    //*** 相机模型相关参数 ***//

    //相机的内参数矩阵
    cv::Mat mK;

	// 注意这里的相机内参数其实都是类的静态成员变量, 但相机的内参数矩阵和矫正参数矩阵却是普通的成员变量
	// ??是否不合理??
    static float fx;        ///<x轴方向焦距
    static float fy;        ///<y轴方向焦距
    static float cx;        ///<x轴方向光心偏移
    static float cy;        ///<y轴方向光心偏移
    static float invfx;     ///<x轴方向焦距的逆
    static float invfy;     ///<x轴方向焦距的逆

    // 去畸变矩阵
    cv::Mat mDistCoef;

    // 相机基线*相机焦距
    float mbf;

    // 相机基线, 单位为米
    float mb;


    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    // ??作者原注释不太理解??

    //判断远点和近点的深度阈值
    float mThDepth;

    // 特征点数量
    int N; 

    //*** 特征点相关 ***//

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    
    // mvKeys:原始左图像提取出的特征点（未校正）
    // mvKeysRight:原始右图像提取出的特征点（未校正）
    // mvKeysUn:校正mvKeys后的特征点，对于双目摄像头，一般得到的图像都是校正好的，再校正一次有点多余
    
    // 原始左图像提取出的特征点（未校正）
    std::vector<cv::KeyPoint> mvKeys;
    // 原始右图像提取出的特征点（未校正）
    std::vector<cv::KeyPoint> mvKeysRight;
	// 对mvKeys校正后的特征点
    std::vector<cv::KeyPoint> mvKeysUn;

    

    ///@note 之所以对于双目摄像头只保存左图像矫正后的特征点,是因为对于双目摄像头,一般得到的图像都是矫正好的,这里再矫正一次有些多余.\n
    ///校正操作是在帧的构造函数中进行的。
    
    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    // 对于双目，mvuRight存储了左目像素点在右目中的对应点的横坐标 （因为纵坐标是一样的）
    // mvDepth对应的深度
    // 单目摄像头，这两个容器中存的都是-1

    ///@note 对于单目摄像头，这两个容器中存的都是-1
    ///对于双目相机,存储左目像素点在右目中的对应点的横坐标 （因为纵坐标是一样的）
    

    // m-member v-vector u-指横坐标
    // 因为最后这个坐标是通过各种拟合方法逼近出来的, 所以使用float存储
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;     //对应的深度
    
    // Bag of Words Vector structures.
    // 内部实际存储的是std::map<WordId, WordValue>
    // WordId 和 WordValue 表示Word在叶子中的id 和权重
    DBoW2::BowVector mBowVec;
    // 内部实际存储 std::map<NodeId, std::vector<unsigned int> >
    // NodeId 表示节点id，std::vector<unsigned int> 中实际存的是该节点id下所有特征点在图像中的索引
    DBoW2::FeatureVector mFeatVec;

    // 左目摄像头特征点对应的描述子 mDescriptors
    // 右目摄像头特征点对应的描述子 mDescriptorsRight
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    /// 每个特征点对应的MapPoint.如果特征点没有对应的地图点,那么将存储一个空指针
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    // 观测不到Map中的3D点
    /// 属于外点的特征点标记,在 Optimizer::PoseOptimization 使用了
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
	//原来通过对图像分区域还能够降低重投影地图点时候的匹配复杂度啊。。。。。
    ///@note 注意到上面也是类的静态成员变量， 有一个专用的标志mbInitialComputations用来在帧的构造函数中标记这些静态成员变量是否需要被赋值
    /// 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
    static float mfGridElementWidthInv;
    /// 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
    static float mfGridElementHeightInv;
    

    // 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀
    // FRAME_GRID_ROWS 48
    // FRAME_GRID_COLS 64
	///这个向量中存储的是每个图像网格内特征点的id（左图）
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];


    // Camera pose.
    cv::Mat mTcw; ///< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵,是我们常规理解中的相机位姿


    // 类的静态成员变量
    // 这些变量则是在整个系统开始执行的时候被初始化的, 它在全局区被初始化
    // 所有类的实例共享一份static
    static long unsigned int nNextId;   // 下一帧ID
    long unsigned int mnId;             // 当前帧ID

    // 参考关键帧
    // 普通帧与自己共视程度最高的关键帧作为参考关键帧
    KeyFrame* mpReferenceKF;


    //*** 图像金字塔相关 ***//

    int mnScaleLevels;                  // 图像金字塔的层数
    float mfScaleFactor;                // 图像金字塔的尺度因子
    float mfLogScaleFactor;             // 图像金字塔的尺度因子的对数值，用于仿照特征点尺度预测地图点的尺度

    // 以下四个vector在ORBextractor中也有相同                              
    vector<float> mvScaleFactors;		// 图像金字塔每一层的缩放因子
    vector<float> mvInvScaleFactors;	// 缩放因子的倒数
    vector<float> mvLevelSigma2;		// 原始缩放因子的平方
    vector<float> mvInvLevelSigma2;		// 原始缩放因子的平方的倒数


    // 未校正图像的边界, 只需要计算一次, 因为是类的静态成员变量
    // 用于确定画格子时的边界 
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;



    // 一个标志，标记是否需要进行初始化运算, 用于标志第一帧, 或SLAM系统进行重新校正后的第一帧会有一些特殊的初始化处理操作
    // true -> 再下一帧的帧构造函数中要进行这个“特殊的初始化操作”
    // false -> 如果没有被置位则不用。
    static bool mbInitialComputations;

private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
	/**
     * @brief 用内参对特征点去畸变，结果报存在mvKeysUn中
     * 
     */
    void UndistortKeyPoints();

    /**
     * @brief 计算去畸变图像的边界
     * 
     * @param[in] imLeft            需要计算边界的图像
     */
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    /**
     * @brief 将提取到的特征点分配到图像网格中 \n
     * @details 该函数由构造函数调用
     * 
     */
    void AssignFeaturesToGrid();

    
    //*** 相机位姿相关 ***//

    // Rotation, translation and camera center
    // 注意 T:变换矩阵 R:旋转矩阵 t:平移向量
    cv::Mat mRcw; // Rotation: world -> camera
    cv::Mat mtcw; // Translation: world -> camera
    cv::Mat mRwc; // Rotation: camera -> world
    cv::Mat mOw;  // mtwc,Translation: camera -> world

};

}// namespace ORB_SLAM

#endif // FRAME_H
