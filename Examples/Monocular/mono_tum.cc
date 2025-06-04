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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<unistd.h>
using namespace std;


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    //运行指令 xxx/mono_tum xxx/ORBvoc.txt xxx.yaml xxx/rgbd_dataset
    //argc = 4
    //程序名 mono_tum, mono模式下跑tum数据集
    //argv[1] 字典路径
    //argv[2] 配置文件路径
    //argv[3] 图片数据集文件夹路径

    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // 检索图片路径
    // strFile 图片数据集文件夹路径下的"/rgb.txt"
    // vstrImageFilenames 图片名
    // vTimestamps 图片时间戳
    // 图片名和时间戳由LoadImages函数得到
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps); // 从数据集strFile从取出图片名和时间戳

    int nImages = vstrImageFilenames.size(); // nImages为图片数量

    // 创建SLAM系统和几个线程
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    //参数1: 字典路径
    //参数2: 配置文件路径
    //参数3: 传感器类型
    //参数4: 开启可视化界面

    vector<float> vTimesTrack; //用于记录跟踪时间的vector
    vTimesTrack.resize(nImages); //vTimesTrack的大小为图片数量

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++) //遍历每个图片
    {
        // 从文件读取图片, im是每个图片
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED); //以原始图像读取(包括alpha通道)
        double tframe = vTimestamps[ni]; //每个tracking time

        // 该次图片读取失败
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

// C11的系统时钟定义区别
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // 将图片传给SLAM系统, 参数1为图片, 参数2为时间戳
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        //计算SLAM系统处理该次图片的duration系统时间ttrack
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        //将跟踪时间ttrack记录进vTimesTrack
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // 关闭SLAM系统,关闭所有线程
    SLAM.Shutdown();

    // 跟踪时间统计
    sort(vTimesTrack.begin(),vTimesTrack.end());
    // totaltime是处理完所有图片的总的跟踪时间
    float totaltime = 0; 
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // 保存相机轨迹
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

/**
 * @brief 导入图片
 * 
 * @param[in] strFile                   读入的文件名称
 * @param[in&out] vstrImageFilenames    彩色图片名称
 * @param[in&out] vTimestamps           记录时间戳
 */
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    // 前三行是注释，跳过
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t); // 时间戳
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB); // 文件名
        }
    }
}
