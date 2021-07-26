#include "camera.h"
#include <ros/ros.h>

Camera::Camera(){
    system = Spinnaker::System::GetInstance();
    cam_1_serial = "20515474"; // Primary Camera
    cam_2_serial = "21102408"; // Secondary Camera
    camera_ready = false;
    cam_1 = nullptr;
    cam_2 = nullptr;
    set_camera();
}
Camera::~Camera(){

    if(camera_ready){
        cam_1->EndAcquisition();
        cam_2->EndAcquisition();
        cam_1->DeInit();
        cam_2->DeInit();
    }
    camList.Clear();

}

void Camera::print_spinnaker_version(){

    const Spinnaker::LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    std::cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
         << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << std::endl
         << std::endl;
}


void Camera::set_camera(){
    camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    std::cout<<"Number of cameras detected: "<<numCameras<<std::endl;

    cam_1 = camList.GetBySerial(cam_1_serial);
    cam_2 = camList.GetBySerial(cam_2_serial);


    if(!cam_1.IsValid() || !cam_2.IsValid()) {
        camera_ready = false;
    } else {
        camera_ready = true;

        cam_1->DeInit();
        cam_2->DeInit();


        cam_1->Init();



        cam_1->LineSelector.SetValue(Spinnaker::LineSelector_Line2);
        cam_1->V3_3Enable.SetValue(true);

        cam_2->Init();

        cam_2->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);
        cam_2->TriggerSource.SetValue(Spinnaker::TriggerSource_Line3);
        cam_2->TriggerOverlap.SetValue(Spinnaker::TriggerOverlap_ReadOut);
        cam_2->TriggerMode.SetValue(Spinnaker::TriggerMode_On);

        cam_1->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);
        cam_2->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

        if(cam_1->DecimationVertical.GetValue()!= 2){
            cam_1->DecimationVertical.SetValue(2);
        }
        if(cam_1->DecimationHorizontal.GetValue()!= 2){
            cam_1->DecimationHorizontal.SetValue(2);
        }
        if(cam_2->DecimationVertical.GetValue()!= 2){
            cam_2->DecimationVertical.SetValue(2);
        }
        if(cam_2->DecimationHorizontal.GetValue()!= 2){
            cam_2->DecimationHorizontal.SetValue(2);
        }

        Spinnaker::GenApi::CBooleanPtr ptrHandlingAcqFrameRateEnable_1 =  cam_1->GetNodeMap().GetNode("AcquisitionFrameRateEnable");
        Spinnaker::GenApi::CBooleanPtr ptrHandlingAcqFrameRateEnable_2 =  cam_2->GetNodeMap().GetNode("AcquisitionFrameRateEnable");

        if (!IsAvailable(ptrHandlingAcqFrameRateEnable_1) || !IsWritable(ptrHandlingAcqFrameRateEnable_1))
        {
            std::cout << "Camera1 "<< " Unable to enable Acquisition Frame Rate (node retrieval). Aborting..."
                    << std::endl;
        }
        if (!IsAvailable(ptrHandlingAcqFrameRateEnable_2) || !IsWritable(ptrHandlingAcqFrameRateEnable_2))
        {
            std::cout << "Camera2 "<< " Unable to enable Acquisition Frame Rate (node retrieval). Aborting..."
                    << std::endl;
        }

        // Enable  Acquisition Frame Rate Enable
        ptrHandlingAcqFrameRateEnable_1->SetValue(true);
        ptrHandlingAcqFrameRateEnable_2->SetValue(true);

        Spinnaker::GenApi::CFloatPtr ptrFrameRate_1 = cam_1->GetNodeMap().GetNode("AcquisitionFrameRate");
        Spinnaker::GenApi::CFloatPtr ptrFrameRate_2 = cam_2->GetNodeMap().GetNode("AcquisitionFrameRate");
        if (!IsAvailable(ptrFrameRate_1) || !IsWritable(ptrFrameRate_1))
        {
            std::cout << "Camera 1 Unable to set Acquisition Frame Rate (node retrieval). Aborting..." << std::endl;
        }
        if (!IsAvailable(ptrFrameRate_2) || !IsWritable(ptrFrameRate_2))
        {
            std::cout << "Camera 2 Unable to set Acquisition Frame Rate (node retrieval). Aborting..." << std::endl;
        }


        // Set 10fps for this example
        const float frameRate = 10.0f;
        ptrFrameRate_1->SetValue(frameRate);
        std::cout << "Camera 1 Frame rate is set to " << frameRate << std::endl;
        ptrFrameRate_2->SetValue(frameRate);
        std::cout << "Camera 2 Frame rate is set to " << frameRate << std::endl;

        Spinnaker::GenApi::INodeMap& sNodeMap_1 = cam_1->GetTLStreamNodeMap();
        Spinnaker::GenApi::INodeMap& sNodeMap_2 = cam_2->GetTLStreamNodeMap();


        Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode_1 = sNodeMap_1.GetNode("StreamBufferHandlingMode");
        if (!IsAvailable(ptrHandlingMode_1) || !IsWritable(ptrHandlingMode_1))
        {
            std::cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << std::endl;
        }

        Spinnaker::GenApi::CEnumEntryPtr ptrHandlingModeEntry_1 = ptrHandlingMode_1->GetCurrentEntry();
        if (!IsAvailable(ptrHandlingModeEntry_1) || !IsReadable(ptrHandlingModeEntry_1))
        {
            std::cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << std::endl;
        }

        Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode_2 = sNodeMap_2.GetNode("StreamBufferHandlingMode");
        if (!IsAvailable(ptrHandlingMode_2) || !IsWritable(ptrHandlingMode_2))
        {
            std::cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << std::endl;
        }

        Spinnaker::GenApi::CEnumEntryPtr ptrHandlingModeEntry_2 = ptrHandlingMode_2->GetCurrentEntry();
        if (!IsAvailable(ptrHandlingModeEntry_2) || !IsReadable(ptrHandlingModeEntry_2))
        {
            std::cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << std::endl;
        }



        ptrHandlingModeEntry_1 = ptrHandlingMode_1->GetEntryByName("NewestOnly");
        ptrHandlingMode_1->SetIntValue(ptrHandlingModeEntry_1->GetValue());
        std::cout << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry_1->GetDisplayName() << std::endl;

        ptrHandlingModeEntry_2 = ptrHandlingMode_2->GetEntryByName("NewestOnly");
        ptrHandlingMode_2->SetIntValue(ptrHandlingModeEntry_2->GetValue());
        std::cout << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry_2->GetDisplayName() << std::endl;

        cam_2->BeginAcquisition();
        cam_1->BeginAcquisition();
    }
}


std::vector<cv::Mat> Camera::acquire_image(double & time, bool & img_ok){

    std::vector<cv::Mat> image_vector;
    if(!camera_ready){
        std::cout<<"Camera is not ready"<<std::endl;
        img_ok = false;
        return image_vector;
    }
    img_ok = true;

    Spinnaker::ImagePtr img1 = cam_1->GetNextImage();
    Spinnaker::ImagePtr img2 = cam_2->GetNextImage();

    image_vector.resize(2);

    if (img1->IsIncomplete())
    {
        std::cout << "Image 1 incomplete: " << Spinnaker::Image::GetImageStatusDescription(img1->GetImageStatus())<< std::endl;
        return image_vector;
    }
    if (img2->IsIncomplete())
    {
        std::cout << "Image 2 incomplete: " << Spinnaker::Image::GetImageStatusDescription(img2->GetImageStatus())<< std::endl;
        return image_vector;
    }

    time = ros::Time::now().toSec();

    const size_t width_1 = img1->GetWidth();
    const size_t height_1 = img1->GetHeight();
    Spinnaker::ImagePtr convertedImage_1 = img1->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::BILINEAR);
    cv::Mat imgMat_1 = cv::Mat(cv::Size(width_1, height_1), CV_8UC3, convertedImage_1->GetData());


    const size_t width_2 = img2->GetWidth();
    const size_t height_2 = img2->GetHeight();
    Spinnaker::ImagePtr convertedImage_2 = img2->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::BILINEAR);
    cv::Mat imgMat_2 = cv::Mat(cv::Size(width_2, height_2), CV_8UC3, convertedImage_2->GetData());

    // cv::rotate(imgMat_1, imgMat_1, cv::ROTATE_180);
    // cv::rotate(imgMat_2, imgMat_2, cv::ROTATE_180);



    // I don't know what the f**k is going on, but it doesn't work if i don't do this
    // ******* Meaningless Resize ******* //
    float ratio = 0.5;
    cv::resize(imgMat_1, imgMat_1, cv::Size(imgMat_1.cols/ratio, imgMat_1.rows/ratio));
    cv::resize(imgMat_1, imgMat_1, cv::Size(imgMat_1.cols*ratio, imgMat_1.rows*ratio));
    cv::resize(imgMat_2, imgMat_2, cv::Size(imgMat_2.cols/ratio, imgMat_2.rows/ratio));
    cv::resize(imgMat_2, imgMat_2, cv::Size(imgMat_2.cols*ratio, imgMat_2.rows*ratio));
    // ********************************** //

    image_vector[0] = imgMat_1;
    image_vector[1] = imgMat_2;

    img1->Release();
    img2->Release();

    return image_vector;
}

void Camera::show_image(cv::Mat & img){
    imshow("asdf", img);
    cv::waitKey(1);
}