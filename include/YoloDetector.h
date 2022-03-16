#ifndef YOLODETECTOR_H
#define YOLODETECTOR_H


#include "benchmark.h"
#include "cpu.h"
#include "datareader.h"
#include "net.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

namespace ORB_SLAM2
{

class YoloBoundingBox{
private:
    cv::Rect2f rect;
    std::string label;

    //This id was used for multi object tracking, but in experiment I found that MOT cost too much time, so I delete the MOT module
    //Right now, this id only used for draw colors of bbox
    int id;
public:
    YoloBoundingBox(cv::Rect2f input_rect, std::string input_label);
    YoloBoundingBox(float x1, float y1, float x2, float y2, std::string input_label);
    std::string GetLabel(){return this->label;}
    cv::Rect2f GetRect(){return this->rect;}
    int GetId(){return this->id;}
    void SetId(int inputId){this->id = inputId;}
};

class YoloDetector
{
private:
    int cpuThreadNum;
    ncnn::Net detector;
    
    std::vector<std::string> class_names;
public:
    YoloDetector(int cpuNum);
    void YoloObjectDetect(cv::Mat image, std::vector<YoloBoundingBox>& yoloBoundingBoxList, int detector_size_width, int detector_size_height);
};

}

#endif