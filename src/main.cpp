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

#define PREFIX_PATH "/mnt/DataDisk/"


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

            for(size_t i = 0; i < threads.size(); ++i) {
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

    // cv::imwrite(std::string(filename_left), image[0], compression_params);
    // cv::imwrite(std::string(filename_right), image[1], compression_params);
    cv::imwrite(std::string(filename_left), image[0]);
    cv::imwrite(std::string(filename_right), image[1]);

    running_check[thread_no] = false;
}

int main(int argc, char ** argv) {
    data_no = 0;
    data_logging = false;

    Camera cam;
    ros::init(argc, argv, "save_image");

    ros::NodeHandle nh("~");

    bool image_show = false;
    bool ros_param_image_show;
    bool params_passed = nh.getParam("image_show", ros_param_image_show);    

    if(!params_passed) {
        std::cout<<"Input should be either true or false"<<std::endl;
        std::cout<<"ex) $ rosrun blackfly_stereo_save save_image"<<std::endl;
        std::cout<<"ex) $ rosrun blackfly_stereo_save save_image _image_show:=true"<<std::endl;
        std::cout<<"ex) $ rosrun blackfly_stereo_save save_image _image_show:=false"<<std::endl;
        std::cout<<"Image show is set to false by default"<<std::endl;
        image_show = false;
    } else {
        if(ros_param_image_show) {
            std::cout<<"Image show is set to true"<<std::endl;
            image_show = true;
        } else {
            std::cout<<"Image show is set to false"<<std::endl;
            image_show = false;
        }
    }

    ros::Subscriber sub_bool = nh.subscribe("/datalogging", 1, dataLoggingFlagCallback);
    ros::Subscriber sub_prefix = nh.subscribe("/save_prefix", 1, dataPrefixCallBack);

    ros::Rate loop_rate(200);
    int thread_num = 0;

    while(ros::ok()){

        // Acquire Images
        bool img_ok = false;
        double time;
        std::vector<cv::Mat> acquired_image =  cam.acquire_image(time, img_ok);

        if(data_logging && img_ok) {
            if(data_prefix.compare(stop_logging_msg)!=0) {
                char timestamp_buf[256];
                // sprintf(timestamp_buf, "%06d\t%f\n", data_no, ros::Time::now().toSec());
                sprintf(timestamp_buf, "%06d\t%f\n", data_no, time);

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
                for(size_t i = 0; i < running_check.size(); ++i) {
                    if(running_check[i]) {
                        ++num_running_thread;
                    }
                }
                std::cout<<"[STEREO CAMERA]"<<std::endl;
                std::cout<<"running thread: "<<num_running_thread<<std::endl;
                std::cout<<"data no.: "<<data_no<<std::endl;

                ++data_no;
            }
        }

        if(image_show && img_ok) {
            cv::Mat img_concat;
            cv::Mat img1 = acquired_image[0];
            cv::Mat img2 = acquired_image[1];
            cv::resize(img1, img1, cv::Size(img1.cols/4, img1.rows/4));
            cv::resize(img2, img2, cv::Size(img2.cols/4, img2.rows/4));

            cv::vconcat(img1, img2, img_concat);
            cv::imshow("Stereo images", img_concat);
            cv::waitKey(1);
        }


        ros::spinOnce();
        loop_rate.sleep();
        // ++count;

    }

    for(size_t i = 0; i<threads.size(); ++i) {
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
