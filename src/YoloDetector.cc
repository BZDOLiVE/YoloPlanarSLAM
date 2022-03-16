#include "YoloDetector.h"

namespace ORB_SLAM2
{

YoloDetector::YoloDetector(int cpuNum)
{
    std::cout << "loading yolo model ..." << std::endl;
    this->detector.load_param("model/yolo-fastest-1.1_body.param");
    this->detector.load_model("model/yolo-fastest-1.1_body.bin");
    this->class_names = {"background", "person"};
    this->cpuThreadNum = cpuNum;

    // this->detector.load_param("model/yolo-fastest.param");
    // this->detector.load_model("model/yolo-fastest.bin");
    // this->class_names = {"background", 
    //                      "airplane", "bicycle", "bird", "boat",
    //                      "bottle", "bus", "car", "cat", "chair",
    //                      "cow", "diningtable", "dog", "horse",
    //                      "motorbike", "person", "pottedplant",
    //                      "sheep", "sofa", "train", "tvmonitor"
    //                     };
    std::cout << "load model success" << std::endl;
}

void YoloDetector::YoloObjectDetect(cv::Mat image, std::vector<YoloBoundingBox>& yoloBoundingBoxList, int detector_size_width, int detector_size_height)
{
    cv::Mat bgr = image.clone();
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, bgr.cols, bgr.rows, detector_size_width, detector_size_height);

    //Data Preprocessing
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = this->detector.create_extractor();
    ex.set_num_threads(this->cpuThreadNum);
    ex.input("data", in);
    ncnn::Mat out;
    ex.extract("output", out);

    for (int i = 0; i < out.h; i++)
    {
        int label;
        float x1, y1, x2, y2, score;
        const float* values = out.row(i);
        
        x1 = values[2] * img_w;
        y1 = values[3] * img_h;
        x2 = values[4] * img_w;
        y2 = values[5] * img_h;

        score = values[1];
        label = values[0];

        //Deal with coordinate out of range
        if(x1<0) x1=0;
        if(y1<0) y1=0;
        if(x2<0) x2=0;
        if(y2<0) y2=0;

        if(x1>img_w) x1=img_w;
        if(y1>img_h) y1=img_h;
        if(x2>img_w) x2=img_w;
        if(y2>img_h) y2=img_h;

        if(label >= class_names.size()){label = 0;}
        
        yoloBoundingBoxList.push_back(YoloBoundingBox(x1, y1, x2, y2, class_names[label]));
    }
    return;
}


YoloBoundingBox::YoloBoundingBox(cv::Rect2f input_rect, std::string input_label){
    this->rect = input_rect;
    this->label = input_label;
    this->id = 0;
}

YoloBoundingBox::YoloBoundingBox(float x1, float y1, float x2, float y2, std::string input_label){
    cv::Point2i p1 = cv::Point2i(x1, y1);
    cv::Point2i p2 = cv::Point2i(x2, y2);
    this->rect = cv::Rect2i(p1, p2);
    this->label = input_label;
    this->id = 0;
}

}