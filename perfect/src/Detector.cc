/* This file is part of ORB-SLAM2-SSD-Semantic.
* 2d目标检测
*/
#include "Detector.h"

// ncnn注册新层=====
class Noop : public ncnn::Layer {};
DEFINE_LAYER_CREATOR(Noop)

//class Detector
//{
// 初始化
Detector::Detector()
{
// 这里的 两个文件地址需要传入
  det_net_ptr = new(ncnn::Net); // 模型
  net_in_ptr  = new(ncnn::Mat); // 网络输入
  // 载入模型   新层注册 Silence 层
  det_net_ptr->register_custom_layer("Silence", Noop_layer_creator);// 避免在log中打印并没有使用的blobs的信息。
    // original pretrained model from https://github.com/chuanqi305/MobileNetv2-SSDLite
    // https://github.com/chuanqi305/MobileNetv2-SSDLite/blob/master/ssdlite/voc/deploy.prototxt
  det_net_ptr->load_param("/home/ewenwan/ewenwan/project/ORB_SLAM2-src-flow--geom-sem-octo/Thirdparty/ncnn/model/mobilenetv2_ssdlite_voc.param");
  det_net_ptr->load_model("/home/ewenwan/ewenwan/project/ORB_SLAM2-src-flow--geom-sem-octo/Thirdparty/ncnn/model/mobilenetv2_ssdlite_voc.bin");
  
}

void Detector::Run(const cv::Mat& bgr_img, std::vector<Object>& objects)
{
// 格式化网络输入
    const int target_size = 300;

    int src_img_w = bgr_img.cols;
    int src_img_h = bgr_img.rows;

   // *net_in_ptr = ncnn::Mat::from_pixels_resize(bgr_img.data, ncnn::Mat::PIXEL_BGR, bgr_img.cols, bgr_img.rows, target_size, target_size);
    // 需要确定图像的格式
    *net_in_ptr = ncnn::Mat::from_pixels_resize(bgr_img.data, ncnn::Mat::PIXEL_RGB, bgr_img.cols, bgr_img.rows, target_size, target_size);

    const float mean_vals[3] = {127.5f, 127.5f, 127.5f};
    const float norm_vals[3] = {1.0/127.5,1.0/127.5,1.0/127.5};
    net_in_ptr->substract_mean_normalize(mean_vals, norm_vals);// 去均值 归一化
//  网络运行获取结果
    ncnn::Extractor net_extractor = det_net_ptr->create_extractor();// 从 网络模型差创建模型提取器
    net_extractor.set_light_mode(true);        
    // 开启轻模式省内存，可以在每一层运算后自动回收中间结果的内存
    net_extractor.set_num_threads(4);// omp 线程数
    // printf("run ... ");
    net_extractor.input("data", *net_in_ptr);
    ncnn::Mat out;
    net_extractor.extract("detection_out",out);

	static const char* class_names[] = {"background",
	"aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair",
	"cow", "diningtable", "dog", "horse",
	"motorbike", "person", "pottedplant",
	"sheep", "sofa", "train", "tvmonitor"};

    objects.clear();
    for (int i=0; i<out.h; i++)
    {
        const float* values = out.row(i);// 每个检测结果

        Object object;// 一个
        object.class_id = int(values[0]);
        object.object_name = std::string(class_names[int(values[0])]);
        object.prob = values[1];
        object.rect.x = values[2] * src_img_w;
        object.rect.y = values[3] * src_img_h;
        object.rect.width = values[4]  * src_img_w - object.rect.x;
        object.rect.height = values[5] * src_img_h - object.rect.y;

        objects.push_back(object);
    }
}

cv::Mat Detector::Show(const cv::Mat& bgr_img, std::vector<Object>& objects)
{
    cv::Mat image = bgr_img.clone(); // 深拷贝===

    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];

       // fprintf(stderr, "%s = %.5f at %.2f %.2f %.2f x %.2f\n", obj.object_name.c_str(), obj.prob,
       //         obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);

        cv::rectangle(image, obj.rect, cv::Scalar(255, 0, 0));// 矩形框
        char text[256];
        sprintf(text, "%s %.1f%%", obj.object_name.c_str(), obj.prob * 100);// 显示的字符

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);// 字符大小
        // 矩形框范围限制
        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > image.cols)
            x = image.cols - label_size.width;
        
        // 图上显示矩阵框 ====
        cv::rectangle(image, cv::Rect(cv::Point(x, y),
                                      cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), CV_FILLED);
        // 图像显示文字 =====
        cv::putText(image, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
    return image;
    // imwrite("result.jpg", image);
    // cv::imshow("image", image);
    // cv::waitKey(0); // 等待按键，窗口不会立即退出
}


// 析构函数
Detector::~Detector()
{
    delete det_net_ptr;
    delete net_in_ptr;
}

