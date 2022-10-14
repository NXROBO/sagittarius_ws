#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <sagittarius_object_color_detector/HSVParamsConfig.h>

#define WINDOW_NAME "HSV Filter"

class HSVParamsNode
{
public:
    struct HSV_range
    {
        int hmin = 0;
        int hmax = 360;
        int smin = 0;
        int smax = 255;
        int vmin = 0;
        int vmax = 255;
    };
    HSV_range hsv_tmp;
    HSV_range hsv[4];
    std::string color_name[4];
    int color_type;
    bool filter_enable = true;
    image_transport::Subscriber image_sub_;
    dynamic_reconfigure::Server<sagittarius_object_color_detector::HSVParamsConfig> server;

    HSVParamsNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~HSVParamsNode();
    void HSVParamsDyRecfgCallBack(sagittarius_object_color_detector::HSVParamsConfig &config, uint32_t level);
    void HSVFilterCallback(const sensor_msgs::ImageConstPtr &msg);
    void loadVisionConfigHSV();
    void saveVisionConfigHSV();
};

HSVParamsNode::HSVParamsNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // dynamic_reconfigure::Server<sagittarius_object_color_detector::HSVParamsConfig> server;
    dynamic_reconfigure::Server<sagittarius_object_color_detector::HSVParamsConfig>::CallbackType f;
    f = boost::bind(&HSVParamsNode::HSVParamsDyRecfgCallBack, this, _1, _2);
    server.setCallback(f);

    image_transport::ImageTransport it_(pnh);
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &HSVParamsNode::HSVFilterCallback, this);
    cv::namedWindow(WINDOW_NAME);

    color_name[0] = "blue";
    color_name[1] = "green";
    color_name[2] = "red";
    color_name[3] = "customize";
    loadVisionConfigHSV();
}

HSVParamsNode::~HSVParamsNode()
{
    saveVisionConfigHSV();
}

void HSVParamsNode::HSVParamsDyRecfgCallBack(sagittarius_object_color_detector::HSVParamsConfig &config, uint32_t level)
{
    if (color_type != config.color_type)
    {
        hsv_tmp.hmax = config.H_Max = hsv[config.color_type].hmax;
        hsv_tmp.hmin = config.H_Min = hsv[config.color_type].hmin;
        hsv_tmp.smax = config.S_Max = hsv[config.color_type].smax;
        hsv_tmp.smin = config.S_Min = hsv[config.color_type].smin;
        hsv_tmp.vmax = config.V_Max = hsv[config.color_type].vmax;
        hsv_tmp.vmin = config.V_Min = hsv[config.color_type].vmin;
        server.updateConfig(config);
    }
    else
    {
        hsv_tmp.hmax = config.H_Max;
        hsv_tmp.hmin = config.H_Min;
        hsv_tmp.smax = config.S_Max;
        hsv_tmp.smin = config.S_Min;
        hsv_tmp.vmax = config.V_Max;
        hsv_tmp.vmin = config.V_Min;

        if (config.groups.hsv_params.red_hue_extension.state && color_type == sagittarius_object_color_detector::HSVParams_red)
            hsv_tmp.hmax = config.groups.hsv_params.red_hue_extension.H_Extend;

        memcpy(&hsv[color_type], &hsv_tmp, sizeof(HSVParamsNode::HSV_range));
    }

    color_type = config.color_type;
    filter_enable = config.HSV_Enable;
    ROS_INFO("Color Type: %s:(max, min)  H:(%d, %d)  S:(%d, %d) V:(%d, %d)",
             color_name[color_type].c_str(),
             hsv_tmp.hmax,
             hsv_tmp.hmin,
             hsv_tmp.smax,
             hsv_tmp.smin,
             hsv_tmp.vmax,
             hsv_tmp.vmin);
}

void HSVParamsNode::HSVFilterCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // 图像裁剪
    cv::Rect select = cv::Rect(0, 0, 640, 480);
    cv::Mat &img_input = cv_ptr->image;

    img_input = img_input(select);

    // 彩色图像的灰度值归一化，颜色空间转换，输出为HSV格式图像
    cv::Mat image2hsv, bgr;
    img_input.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
    cv::cvtColor(bgr, image2hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask = cv::Mat::zeros(bgr.size(), CV_32FC3);
    if (hsv_tmp.hmin > hsv_tmp.hmax)
    {
        cv::Mat mask1;
        cv::Mat mask2;
        cv::inRange(image2hsv,
                    cv::Scalar(0, hsv_tmp.smin / float(255), hsv_tmp.vmin / float(255)),
                    cv::Scalar(hsv_tmp.hmax, hsv_tmp.smax / float(255), hsv_tmp.vmax / float(255)),
                    mask1);
        cv::inRange(image2hsv,
                    cv::Scalar(hsv_tmp.hmin, hsv_tmp.smin / float(255), hsv_tmp.vmin / float(255)),
                    cv::Scalar(360, hsv_tmp.smax / float(255), hsv_tmp.vmax / float(255)),
                    mask2);
        mask = mask1 | mask2;
    }
    else
    {
        cv::inRange(image2hsv,
                    cv::Scalar(hsv_tmp.hmin, hsv_tmp.smin / float(255), hsv_tmp.vmin / float(255)),
                    cv::Scalar(hsv_tmp.hmax, hsv_tmp.smax / float(255), hsv_tmp.vmax / float(255)),
                    mask);
    }

    if (filter_enable)
    {
        cv::Mat output = cv::Mat::zeros(bgr.size(), CV_32FC3);
        // for (int r = 0; r < bgr.rows; r++)
        //     for (int c = 0; c < bgr.cols; c++)
        //         if (mask.at<uchar>(r, c) == 255)
        //             output.at<cv::Vec3f>(r, c) = bgr.at<cv::Vec3f>(r, c);
        bgr.copyTo(output, mask);
        cv::imshow(WINDOW_NAME, output);
    }
    else
    {
        cv::imshow(WINDOW_NAME, bgr);
    }

    cv::waitKey(1);
}

void HSVParamsNode::loadVisionConfigHSV()
{
    ros::NodeHandle nh;

    for (int i = 0; i < end(color_name) - begin(color_name); i++)
    {
        nh.param(color_name[i] + "/hmin", hsv[i].hmin, 0);
        nh.param(color_name[i] + "/hmax", hsv[i].hmax, 360);
        nh.param(color_name[i] + "/smin", hsv[i].smin, 0);
        nh.param(color_name[i] + "/smax", hsv[i].smax, 255);
        nh.param(color_name[i] + "/vmin", hsv[i].vmin, 0);
        nh.param(color_name[i] + "/vmax", hsv[i].vmax, 255);
    }

    memcpy(&hsv_tmp, &hsv[0], sizeof(HSVParamsNode::HSV_range));
    sagittarius_object_color_detector::HSVParamsConfig config;
    server.getConfigDefault(config);
    config.H_Max = hsv_tmp.hmax;
    config.H_Min = hsv_tmp.hmin;
    config.S_Max = hsv_tmp.smax;
    config.S_Min = hsv_tmp.smin;
    config.V_Max = hsv_tmp.vmax;
    config.V_Min = hsv_tmp.vmin;
    config.groups.hsv_params.red_hue_extension.state = false;
    server.updateConfig(config);

}

void HSVParamsNode::saveVisionConfigHSV()
{
    std::string path = ros::package::getPath("sagittarius_object_color_detector");
    path.append("/config/vision_config.yaml");
    ROS_INFO("YAML path \"%s\"", path.c_str());
    YAML::Node config = YAML::LoadFile(path);

    for (int i = 0; i < end(color_name) - begin(color_name); i++)
    {
        config[color_name[i]]["hmin"] = hsv[i].hmin;
        config[color_name[i]]["hmax"] = hsv[i].hmax;
        config[color_name[i]]["smin"] = hsv[i].smin;
        config[color_name[i]]["smax"] = hsv[i].smax;
        config[color_name[i]]["vmin"] = hsv[i].vmin;
        config[color_name[i]]["vmax"] = hsv[i].vmax;
    }

    std::ofstream fout(path);
    fout << config;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HSVParams_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    HSVParamsNode foo(nh, pnh);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
