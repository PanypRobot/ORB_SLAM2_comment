/**
 * @file Map.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 地图的实现
 * @version 0.1
 * @date 2019-02-26
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


#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

//构造函数,
//当前地图中关键帧的最大关键帧id归0
Map::Map():mnMaxKFid(0)
{
}

/*
 * @brief Insert KeyFrame in the map
 * @param pKF KeyFrame
 */
//在地图中插入关键帧,同时更新关键帧的最大id
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap); //操作类成员变量时的互斥锁
    mspKeyFrames.insert(pKF);   //当前关键帧加入关键帧set
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

/*
 * @brief Insert MapPoint in the map
 * @param pMP MapPoint
 */
//向地图中插入地图点
void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap); //操作类成员变量时的互斥锁
    mspMapPoints.insert(pMP);   //当前地图点加入地图点set
}

/**
 * @brief 从地图中删除地图点,但是其实这个地图点所占用的内存空间并没有被释放
 * 
 * @param[in] pMP 
 */
void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap); //操作类成员变量时的互斥锁
    mspMapPoints.erase(pMP);    //从地图点set中删去当前地图点

    // 实际上只是从std::set中删除了地图点的指针
    // 原先地图点占用的内存区域并没有得到释放
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/**
 * @brief Erase KeyFrame from the map
 * @param pKF KeyFrame
 */
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap); //操作类成员变量时的互斥锁
    mspKeyFrames.erase(pKF);    //从关键帧set中删去当前关键帧

    // 原先关键帧占用的内存区域并没有得到释放
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/*
 * @brief 设置参考MapPoints，将用于DrawMapPoints函数画图
 * @param vpMPs Local MapPoints
 */
// 设置参考地图点用于绘图显示局部地图点（红色）
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}


//****原版的泡泡机器人注释的版本中没有下面两个函数****//
//REVIEW 这个好像没有用到
void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

//REVIEW 目测也是当前在程序中没有被被用到过
int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}
//****原版的泡泡机器人注释的版本中没有上面两个函数****//


//获取地图中的所有关键帧
vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    //返回一个vector迭代器, 包含mspKeyFrames set中所有元素, 即所有关键帧
    //注意这里是调用了vector的构造函数
}

//获取地图中的所有地图点
vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    //返回一个vector迭代器, 包含mspMapPoints set中的所有元素, 即所有地图点
}

//获取地图点数目
long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size(); //返回地图点个数
}

//获取地图中的关键帧数目
long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size(); //返回关键帧个数
}

//获取参考地图点
vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints; //返回存储参考地图点的vector
}

//获取地图中最大的关键帧id
long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid; //这个值表示地图中最大关键帧的id, 会在每次插入关键帧操作后更新为新插入关键帧的id
}

//清空地图中的数据
void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;    //用迭代器遍历mspMapPoints, 并逐个释放内存

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;    //用迭代器遍历mspKeyFrames, 并逐个释放内存

    mspMapPoints.clear();   //清除地图点集合并将其大小转换为0
    mspKeyFrames.clear();   //清除关键帧集合并将其大小转换为0
    mnMaxKFid = 0;          //关键帧最大id为0, 因为清除所有关键帧
    mvpReferenceMapPoints.clear();  //清除参考地图点
    mvpKeyFrameOrigins.clear();     //清除最初始关键帧
}

} //namespace ORB_SLAM
