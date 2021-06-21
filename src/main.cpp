#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

//Time Check
#include <chrono>
// Create directory
#include <sys/stat.h>
// Threading
#include <thread>

#include "camera.h"

#define PREFIX_PATH "/home/morin/Dataset/"


bool data_logging;
std::string data_prefix = "Stoplogging";
std::string stop_logging_msg = "Stoplogging";
std::string dir_name;
std::string dir_name_left;
std::string dir_name_right;

std::vector<std::thread> threads(30);
std::vector<bool> running_check(30, false);

std::vector<int> compression_params;

int data_no = 0;

FILE * ptr_time = nullptr;


void dataLoggingFlagCallback(const std_msgs::Bool::ConstPtr &msg){
    if(msg->data) {
        if(data_logging) {

        } else {
            data_logging = true;
            ROS_INFO("Data Logging Set True");
            data_no = 0;
        }
    } else {
        if(data_logging){
            data_logging = false;
            ROS_INFO("Data Logging Set False");

            for(int i = 0; i < threads.size(); ++i) {
                if(threads[i].joinable()) {
                    threads[i].join();
                }
            }

            threads.clear();
            running_check.clear();

        }
    }
}

void dataPrefixCallBack(const std_msgs::String::ConstPtr & msg) {
    if(msg->data.compare(stop_logging_msg)!=0) {
        if(data_prefix.compare(stop_logging_msg) == 0) {
            std::cout<<"Prefix changed to logging"<<std::endl;
            data_prefix = msg->data;
            dir_name = PREFIX_PATH + data_prefix + "_stereo";
            dir_name_left = dir_name + "/stereoleft";
            dir_name_right = dir_name + "/stereoright";

            mkdir(dir_name.c_str(), 0777);
            mkdir(dir_name_left.c_str(), 0777);
            mkdir(dir_name_right.c_str(), 0777);

            std::string timestampPath = dir_name + "/timestamp.txt";
            ptr_time = fopen(timestampPath.c_str(), "a");
            if(!ptr_time)
                printf("FAILED to open %s\n", timestampPath.c_str()); 

        }
    } else {
        if(data_prefix.compare(stop_logging_msg)!=0) {
            std::cout<<"Prefix changed to stop logging"<<std::endl;
            data_prefix = msg->data;
            fclose(ptr_time);
        }
    }
}

void save_image(std::vector<cv::Mat> image, 
                std::string filename_left, 
                std::string filename_right,
                int thread_no) {
    running_check[thread_no] = true;

    cv::imwrite(std::string(filename_left), image[0], compression_params);
    cv::imwrite(std::string(filename_right), image[1], compression_params);
    running_check[thread_no] = false;
}

int main(int argc, char ** argv) {
    data_no = 0;
    data_logging = false;

    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

    Camera cam;
    ros::init(argc, argv, "save_image");

    ros::NodeHandle nh;

    ros::Subscriber sub_bool = nh.subscribe("/datalogging", 1, dataLoggingFlagCallback);
    ros::Subscriber sub_prefix = nh.subscribe("/save_prefix", 1, dataPrefixCallBack);

    // cv_bridge::CvImage img_bridge;

    // sensor_msgs::Image img1;
    // sensor_msgs::Image img2;
    // std_msgs::Header header;

    // ros::Publisher pub_img1 = nh.advertise<sensor_msgs::Image>("camera1/image", 1);
    // ros::Publisher pub_img2 = nh.advertise<sensor_msgs::Image>("camera2/image", 1);

    // int count = 0;

    ros::Rate loop_rate(200);
    int thread_num = 0;

    while(ros::ok()){

        // Acquire Images
        std::vector<cv::Mat> acquired_image =  cam.acquire_image();

        // header.seq = count;
        // header.stamp = ros::Time::now();
        // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, acquired_image[0]);
        // img_bridge.toImageMsg(img1);
        // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, acquired_image[1]);
        // img_bridge.toImageMsg(img2);

        // pub_img1.publish(img1);
        // pub_img2.publish(img2);

        if(data_logging) {
            if(data_prefix.compare(stop_logging_msg)!=0) {
                char timestamp_buf[256];
                sprintf(timestamp_buf, "%06d\t%f\n", data_no, ros::Time::now().toSec());

                fwrite(timestamp_buf, 1, strlen(timestamp_buf), ptr_time);



                char filename_left[256];
                sprintf( filename_left, "%s/Image%06d_raw.png",dir_name_left.c_str(), data_no);

                char filename_right[256];
                sprintf( filename_right, "%s/Image%06d_raw.png",dir_name_right.c_str(), data_no);

                bool all_thread_running = true;
                int empty_thread = -1;
                for(int i = 0; i < thread_num; ++i) {
                    if(running_check[i]){

                    } else{
                        empty_thread = i;
                        all_thread_running = false;
                        if(threads[i].joinable()){
                            threads[i].join();
                        }
                    }
                }

                if(all_thread_running) {
                    running_check.push_back(false);
                    threads.emplace_back(save_image, 
                                         acquired_image, 
                                         filename_left,
                                         filename_right,
                                         running_check.size()-1);
                    threads[running_check.size()-1].detach();
                    ++thread_num;
                } else {
                    threads[empty_thread] = std::thread(save_image, 
                                                        acquired_image,
                                                        filename_left,
                                                        filename_right,
                                                        empty_thread);
                    threads[empty_thread].detach();    
                }

                int num_running_thread = 0;
                for(int i = 0; i < running_check.size(); ++i) {
                    if(running_check[i]) {
                        ++num_running_thread;
                    }
                }
                std::cout<<"running thread"<<num_running_thread<<std::endl;
                std::cout<<data_no<<std::endl;

                ++data_no;
            }
        }


        ros::spinOnce();
        loop_rate.sleep();
        // ++count;

    }

    for(int i = 0; i<threads.size(); ++i) {
        threads[i].join();
    }

    return 0;
}

// 데이터 저장 명령
// Topic name : /datalogging
// Type : std_msgs/Bool
// Data logging start : True
// Data logging stop : False
// Usage example : msg.data = True
// 데이터 저장 양식 ([실험번호]_날짜및시간)
// Topic Name : /save_prefix
// Type : std_msgs/String
// Usage example : msg.data = "[123]_2021_05_24_15_01_14"
