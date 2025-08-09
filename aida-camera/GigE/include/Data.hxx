#ifndef __DATA_HXX__
#define __DATA_HXX__

#include <string>
#include "CameraApi.h" //API header file of camera SDK

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/opencv.hpp"


struct Data{

    int iCameraCounts_{1};
    int iStatus_{-1};
    tSdkCameraDevInfo tCameraEnumList_[64];
    tSdkCameraDevInfo tCameraDevInfo_;
    int hCamera_{};
    tSdkCameraCapbility tCapability_;      //Device Description Information
    tSdkFrameHead sFrameInfo_;
    BYTE* pbyBuffer_;
    int channel_{3};
    unsigned char* g_pRgbBuffer_;     //Processed data cache area
    tSdkImageResolution *pImageSizeDesc;

    // Device config
    int saturation_{80}; // 0->100
    double exposure_time_{(5000) }; // Continuously adjustable. 16->2097140 numbers should be divisible by 16. [unit] microseconds.
    double auto_exposure_target_{50}; // 20 -> 160
    std::string resolution_{"640,480"};
    int analog_gain_{25}; // 5 -> 33
    int gamma_{100}; // 0 -> 240
    int contrast_{100}; // 0 -> 190
    BOOL monochrome_{false}; 
    bool auto_exposure_{true}; 
    int sharpness_{90}; // 0 -> 90
    BOOL invert_color_{false}; // It works only if the monochrome is enabled.
    BOOL anti_flick_{true};
    int light_frequency_{50}; // 50, 60
    int out_format_{};
    int fps_{30}; 
    int frame_speed_{2}; // 0, 1, 2 -> 0: low_fps, 2: high_fps
    std::string load_from_file_address_{};
    std::string index_;
    std::string serial_;
    bool is_calibrated_;

    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
    cv::Mat rectification_matrix_;
    cv::Mat projection_matrix_;

    std::vector<double> d, k, r, p;
    std::string distortion_model_;
    std::array<double, 3> initial_pose_;
    std::array<double, 3> initial_orientation_;

    cv::Mat distorted_frame_, undistorted_frame_;
    cv::Mat map1_, map2_;
    cv::Size image_size_;
    cv::Rect ROI_;

    bool load_file{};
};

#endif