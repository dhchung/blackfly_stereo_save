#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>
#include <chrono>
// Create directory
#include <sys/stat.h>

#include "camera.h"

#define PREFIX_PATH "/mnt/DataDisk/"


bool data_logging;
std::string data_prefix = "Stoplogging";
std::string stop_logging_msg = "Stoplogging";
std::string dir_name;
std::string dir_name_left;
std::string dir_name_right;

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

            int mkdir_status = mkdir(dir_name.c_str(), 0777);
            int mkdir_status_left = mkdir(dir_name_left.c_str(), 0777);
            int mkdir_status_right = mkdir(dir_name_right.c_str(), 0777);

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


int main(int argc, char ** argv) {
    data_no = 0;
    data_logging = false;

    Camera cam;
    ros::init(argc, argv, "send_image");

    ros::NodeHandle nh;
    image_transport::ImageTransport it_1(nh);
    image_transport::ImageTransport it_2(nh);

    image_transport::Publisher pub_1 = it_1.advertise("camera1/image", 1);
    image_transport::Publisher pub_2 = it_2.advertise("camera2/image", 1);

    ros::Subscriber sub_bool = nh.subscribe("/datalogging", 1, dataLoggingFlagCallback);
    ros::Subscriber sub_prefix = nh.subscribe("/save_prefix", 1, dataPrefixCallBack);

    ros::Rate loop_rate(200);

    while(ros::ok()){

        // Acquire Images
        std::vector<cv::Mat> acquired_image =  cam.acquire_image();

        // sensor_msgs::ImagePtr msg_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", acquired_image[0]).toImageMsg();
        // sensor_msgs::ImagePtr msg_2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", acquired_image[1]).toImageMsg();

        // pub_1.publish(msg_1);
        // pub_2.publish(msg_2);



        // cam.show_image(acquired_image[0]);

        if(data_logging) {
            if(data_prefix.compare(stop_logging_msg)!=0) {
                char timestamp_buf[256];
                sprintf(timestamp_buf, "%06d\t%f\n", data_no, ros::Time::now().toSec());

                fwrite(timestamp_buf, 1, strlen(timestamp_buf), ptr_time);



                char filename_left[256];
                sprintf( filename_left, "%s/Image%06d_raw.png",dir_name_left.c_str(), data_no);

                char filename_right[256];
                sprintf( filename_right, "%s/Image%06d_raw.png",dir_name_right.c_str(), data_no);


                cv::imwrite(std::string(filename_left), acquired_image[0]);
                cv::imwrite(std::string(filename_right), acquired_image[1]);

                std::cout<<data_no<<std::endl;

                ++data_no;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();

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
