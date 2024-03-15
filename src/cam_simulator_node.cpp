#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"

#include <eigen3/Eigen/Core>
using namespace std::chrono_literals;


using std::placeholders::_1;

class CamSimulatorNode : public rclcpp::Node
{
    public:
        CamSimulatorNode(): Node("cam_simulator_node")
        {
            sub_feature_measurements = this->create_subscription<std_msgs::msg::Int16MultiArray>("/scene_viewer/feature_measurements", 10, std::bind(&CamSimulatorNode::feature_measurements_cb, this, _1));
            sub_feature_colors = this->create_subscription<std_msgs::msg::Int16MultiArray>("/scene_viewer/feature_colors", 10, std::bind(&CamSimulatorNode::feature_colors_cb, this, _1));
            sub_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>("/scene_viewer/camera_info", 10, std::bind(&CamSimulatorNode::camera_info_cb, this, _1));
            sub_mark = this->create_subscription<std_msgs::msg::Float32>("/scene_viewer/mark", 10, std::bind(&CamSimulatorNode::mark_cb, this, _1));
            sub_index = this->create_subscription<std_msgs::msg::Int16>("/scene_viewer/index", 10, std::bind(&CamSimulatorNode::index_cb, this, _1));
            pub_image = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

            // timer to publish images
            timer = this->create_wall_timer(50ms, std::bind(&CamSimulatorNode::publish_images, this));
        }
    private:
        float mark;
        int index;
        sensor_msgs::msg::Image::SharedPtr output_img_msg;
        std::vector<cv::Mat> all_images;
       
        // Eigen::MatrixXf feature_measurements =  Eigen::MatrixXf(506,400);
        bool received_camera_info = false;
        int height;
        int width;
        std::string cam_id;
        bool received_feature_measurements = false;
        bool received_feature_colors = false;
        int num_images; // number of images
        int num_features; // number of features
        Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic> feature_measurements;
        Eigen::Matrix<int16_t, Eigen::Dynamic, 3> feature_colors;
        bool images_generated = false;
        
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_mark;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_index;
        rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_feature_measurements;
        rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_feature_colors;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
        rclcpp::TimerBase::SharedPtr timer;


        void feature_measurements_cb(const std_msgs::msg::Int16MultiArray::SharedPtr msg) 
        // receive updates a slow rate, store in array variable,
        // publish images at fixed rate, sync with trajectory 
        {
            const int stride0 = msg->layout.dim[0].stride;
            const int stride1 = msg->layout.dim[1].stride;
            const int n = msg->layout.dim[0].size;
            const int m = msg->layout.dim[1].size;
            this->received_feature_measurements = true;

            // https://gist.github.com/jarvisschultz/7a886ed2714fac9f5226?permalink_comment_id=2941877#file-matrix_receiver-cpp-L18
            std::vector<int16_t> data = msg->data;
            Eigen::Map<Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mat(data.data(), n, m);
            this->feature_measurements = mat;
            // std::cout << "received feature measurements " << std::endl << this->feature_measurements << std::endl;
            return;
        }
        void feature_colors_cb(const std_msgs::msg::Int16MultiArray::SharedPtr msg) 
        {
            const int n = msg->layout.dim[0].size;
            this->received_feature_colors = true;

            std::vector<int16_t> data = msg->data;
            Eigen::Map<Eigen::Matrix<int16_t, Eigen::Dynamic, 3, Eigen::RowMajor> > mat(data.data(), n, 3);
            this->feature_colors = mat;
            // std::cout << "received feature colors  " << std::endl << this->feature_colors << std::endl;
            return;
        }

        void camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) 
        {
           
            this->height = msg->height;
            this->width = msg->width;
            this->cam_id = msg->header.frame_id;
            // RCLCPP_INFO(this->get_logger(), "height: %d, width: %d, frame_id: %s", this->height, this->width, this->cam_id.c_str());

            if (this->received_feature_measurements and this->received_feature_colors)
            {
                if (!this->images_generated)
                {
                RCLCPP_INFO(this->get_logger(), "generating images...");
                this->generate_images(this->height, this->width, this->feature_measurements, this->feature_colors);
                }
            }
    
            return;
        }

        void mark_cb(const std_msgs::msg::Float32::SharedPtr msg)
        {
            this->mark = msg->data;
            // RCLCPP_INFO(this->get_logger(), "mark = %f", this->mark);
            return;
        }
        void index_cb(const std_msgs::msg::Int16::SharedPtr msg)
        {
            this->index = msg->data;
            // RCLCPP_INFO(this->get_logger(), "index = %d", this->index);
            return;
        }


        void generate_images(int height, int width, Eigen::Matrix<int16_t,Eigen::Dynamic,Eigen::Dynamic> feature_measurements, Eigen::Matrix<int16_t,Eigen::Dynamic,3> feature_colors)
        {

            this->num_images = feature_measurements.rows()/2;
            this->num_features = feature_measurements.cols();
            // for every other row in the matrix, generate an image
            // for every column in the matrix, draw a circle

            for(int i = 0; i < this->num_images; i++)
            {
                cv::Mat img(cv::Size(width, height), CV_8UC3, cv::Scalar(0, 0, 0));
                // cv::Mat img(cv::Size(height, width), CV_8UC3, cv::Scalar(0, 0, 0));
                // RCLCPP_INFO(this->get_logger(), "image %d generated, all images size %d", i, this->all_images.size());
                for(int j = 0; j < this->num_features; j++)
                {
                    int x = feature_measurements(2*i,j);
                    int y = feature_measurements(2*i+1,j);
                    auto color = cv::Scalar(feature_colors(j,2), feature_colors(j,1), feature_colors(j,0)); //bgr
                    // label = std::to_string(x) + "," + std::to_string(y);
                    cv::circle(img, cv::Point(x, y), 1, color, -1);

                    // put text in top left
                    cv::putText(img, std::to_string(i), cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                }
                cv::imwrite("/home/jesse/ros2_ws/src/motion_tools/data/images/" + std::to_string(i) + ".png", img);
                this->all_images.push_back(img);
            }
            this->images_generated = true;
            return;                 
        }

        void publish_images()
        {
            // publish the image corresponding to the most recent index value, at fixed rate (rate just for display purposes, not for actual camera rate)
            int index = this->index;
            int n = this->num_images;
            int i;               
               // cv::circle(img, cv::Point(feature_measurements(i,j), feature_measurements(i+1,j)), 5, cv::Scalar(0, 255, 0), -1);
                    // RCLCPP_INFO(this->get_logger(), "(i,j) = (%d,%d)", i, j);
            i = (int)round((this->mark)*n);

            // if received feature measurements, publish the image
            if (this->received_feature_measurements)
            {
                if (i >= this->all_images.size())
                {
                    RCLCPP_INFO(this->get_logger(), "index out of range: %d", i);
                    return;
                }
                else
                {
                cv::Mat img = this->all_images[i];
                RCLCPP_INFO(this->get_logger(), "publishing image: %d out of %d  (h,w): (%d,%d)", i, n, img.rows, img.cols);
                output_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
                pub_image->publish(*output_img_msg.get());
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "feature measurements not received yet");
                return;
            }

            return;
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}