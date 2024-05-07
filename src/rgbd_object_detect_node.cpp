#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>

class RgbdObjectDetection {
public:
    RgbdObjectDetection() :
                            pnh_("~"),
                            rgb_sub_(nh_, "/camera/rgb/image_raw", 2), 
                            depth_sub_(nh_, "/camera/depth/image_raw", 2),
                            camera_info_sub_(nh_, "/camera/rgb/camera_info", 2),
                            sync_(MySyncPolicy(5), rgb_sub_, depth_sub_, camera_info_sub_)
    {
        pnh_.param<std::string>("package_name", param_package_name_, "vision_robot");
        pnh_.param<std::string>("coco_path", param_coco_path_, "/config/detection/class_name_coco.txt");
        pnh_.param<std::string>("model_path", param_model_path_, "/config/detection/frozen_inference_graph.pb");
        pnh_.param<std::string>("config_path", param_config_path_, "/config/detection/ssd_mobilenet_v2_coco_config.pbtxt");

        package_path_ = ros::package::getPath(param_package_name_);
        coco_path_ = package_path_ + param_coco_path_;

        model_ = cv::dnn::readNet(package_path_ + param_model_path_,
                                   package_path_ + param_config_path_,
                                   "TensorFlow");

        loadClassNames(class_names_);
        colors_ = cv::Mat::zeros(class_names_.size(), 1, CV_8UC3);
        for (int i = 0; i < colors_.rows; ++i) {
            colors_.at<cv::Vec3b>(i) = cv::Vec3b(rand() % 255, rand() % 255, rand() % 255);
        }
        ids_ = cv::Mat::ones(class_names_.size(), 1, CV_32SC1);

        image_pub_ = nh_.advertise<sensor_msgs::Image>("/object_detection/rgb/image_raw", 1);
        object_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/object_detection/marker_array/single", 1);
        text_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/object_detection/marker_array/label", 1);

        sync_.registerCallback(boost::bind(&RgbdObjectDetection::syncImageCallback, this, _1, _2, _3));

        object_marker_.type = visualization_msgs::Marker::SPHERE;
        object_marker_.action = visualization_msgs::Marker::ADD;
        object_marker_.pose.orientation.w = 1.0;
        object_marker_.scale.x = 0.5;
        object_marker_.scale.y = 0.5;
        object_marker_.scale.z = 0.5;
        object_marker_.color.a = 1.0;
        object_marker_.lifetime = ros::Duration();
        object_marker_.frame_locked = false;

        text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker_.action = visualization_msgs::Marker::ADD;
        text_marker_.pose.orientation.w = 1.0;
        text_marker_.scale.x = 0.3;
        text_marker_.scale.y = 0.3;
        text_marker_.scale.z = 0.3;
        text_marker_.color.r = 0.0;
        text_marker_.color.g = 0.0;
        text_marker_.color.b = 0.0;
        text_marker_.color.a = 1.0;
        text_marker_.lifetime = ros::Duration();
        text_marker_.frame_locked = false;
    }

    void loadClassNames(std::vector<std::string>& class_names) {
        std::ifstream file(coco_path_);
        std::string line;
        while (std::getline(file, line)) {
            class_names.push_back(line);
        }
    }

    void targetDetection(cv::Mat& image, const cv::Mat& depth_image, const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg) {
        int image_height = image.rows;
        int image_width = image.cols;
        cv::Mat blob = cv::dnn::blobFromImage(image, 1.0, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5), true, false);
        model_.setInput(blob);
        cv::Mat output = model_.forward();
        cv::Mat detection(output.size[2], output.size[3], CV_32F, output.ptr<float>());
        //visualization_msgs::MarkerArray delete_marker_array;
        //ids_ = cv::Mat::ones(class_names_.size(), 1, CV_32SC1);
        object_marker_array_.markers.clear();
        text_marker_array_.markers.clear();

        for (int i = 0; i < detection.rows; ++i) {
            float confidence = detection.at<float>(i, 2);
            if (confidence > 0.6) {
                int class_id = static_cast<int>(detection.at<float>(i, 1));
                std::string class_name = class_names_[class_id - 1];

                int left_x = static_cast<int>(detection.at<float>(i, 3) * image_width);
                int top_y = static_cast<int>(detection.at<float>(i, 4) * image_height);
                int right_x = static_cast<int>(detection.at<float>(i, 5) * image_width);
                int bottom_y = static_cast<int>(detection.at<float>(i, 6) * image_height);

                int center_x = static_cast<int>((left_x + right_x) / 2);
                int center_y = static_cast<int>((top_y + bottom_y) / 2);

                float depth = depth_image.at<float>(cv::Point(center_x, center_y));
                if (std::isnan(depth) || std::isinf(depth)) continue;

                depth = std::max(std::min(depth, 10.0f), 0.1f);

                cv::rectangle(image, cv::Point(left_x, top_y), cv::Point(right_x, bottom_y), colors_.at<cv::Vec3b>(class_id - 1), 2);
                cv::circle(image, cv::Point(center_x, center_y), 15, colors_.at<cv::Vec3b>(class_id - 1), cv::FILLED);
                cv::putText(image, class_name, cv::Point(left_x, top_y - 5), cv::FONT_HERSHEY_SIMPLEX, 1, colors_.at<cv::Vec3b>(class_id - 1), 2);
                cv::putText(image, "Confidence: " + std::to_string(confidence), cv::Point(left_x, top_y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, colors_.at<cv::Vec3b>(class_id - 1), 2);

                object_marker_.ns = class_name;
                object_marker_.header.stamp = ros::Time::now();
                object_marker_.id = ids_.at<int>(class_id - 1);
                object_marker_.pose.position.x = (center_x - camera_info_msg->P[2]) * depth / camera_info_msg->P[0];
                object_marker_.pose.position.y = (center_y - camera_info_msg->P[6]) * depth / camera_info_msg->P[5];
                object_marker_.pose.position.z = depth;
                object_marker_.color.r = colors_.at<cv::Vec3b>(class_id - 1)[2] / 255.0;
                object_marker_.color.g = colors_.at<cv::Vec3b>(class_id - 1)[1] / 255.0;
                object_marker_.color.b = colors_.at<cv::Vec3b>(class_id - 1)[0] / 255.0;
                object_marker_array_.markers.push_back(object_marker_);

                text_marker_.ns = object_marker_.ns + "_text";
                text_marker_.header.stamp = object_marker_.header.stamp;
                text_marker_.id = object_marker_.id;
                text_marker_.pose.position.x = object_marker_.pose.position.x;
                text_marker_.pose.position.y = object_marker_.pose.position.y - 1.0;
                text_marker_.pose.position.z = object_marker_.pose.position.z;
                text_marker_.text = object_marker_.ns + "\nConfidence: " + std::to_string(confidence);
                text_marker_array_.markers.push_back(text_marker_);
                //ids_.at<int>(class_id - 1) += 1;
            }
        }
        object_markers_pub_.publish(object_marker_array_); // add new marker
        text_markers_pub_.publish(text_marker_array_);
    }


    void syncImageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
        try {
            object_marker_.header.frame_id = rgb_msg->header.frame_id;
            text_marker_.header.frame_id = object_marker_.header.frame_id;
            cv_bridge::CvImagePtr color_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

            targetDetection(color_ptr->image, depth_ptr->image, camera_info_msg);

            image_pub_.publish(color_ptr->toImageMsg());
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string package_path_;
    std::string coco_path_;
    cv::dnn::Net model_;
    std::vector<std::string> class_names_;
    cv::Mat colors_;
    cv::Mat ids_;

    std::string param_package_name_;
    std::string param_coco_path_;
    std::string param_model_path_;
    std::string param_config_path_;

    ros::Publisher image_pub_;
    ros::Publisher object_markers_pub_;
    ros::Publisher text_markers_pub_;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync_;

    visualization_msgs::MarkerArray object_marker_array_, text_marker_array_;
    visualization_msgs::Marker object_marker_, text_marker_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detection");
    RgbdObjectDetection rgbd_object_detection;
    ros::spin();
    return 0;
}
