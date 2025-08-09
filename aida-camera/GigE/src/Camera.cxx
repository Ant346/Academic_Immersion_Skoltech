
#include "Camera.hxx"
#include <sstream>

Camera::~Camera(){
    CameraUnInit(this -> data_.hCamera_);
    if (this -> data_.g_pRgbBuffer_ != NULL) {free(this -> data_.g_pRgbBuffer_); this -> data_.g_pRgbBuffer_ = NULL;};
    if (this -> data_.pbyBuffer_ != NULL) {free(this -> data_.pbyBuffer_); this -> data_.pbyBuffer_ = NULL;};
}
auto Camera::setIndex(std::string value) -> void{
    this -> data_.index_ = value;
}

auto Camera::setSerial(std::string value) -> void{
    if (value.size() > 0){value.pop_back();}
    this -> data_.serial_ = value;
}

auto Camera::setResolution(std::string value) -> void{
    std::vector<std::string> res{this->splitString(value, ',')};
    this -> data_.image_size_.height = std::stoul(res[1]);
    this -> data_.image_size_.width = std::stoul(res[0]);
    if (data_.image_size_.height < 2048 and data_.image_size_.width < 2048){
        data_.ROI_ = cv::Rect(1024-data_.image_size_.width/2, 1024-data_.image_size_.height /2, data_.image_size_.width, data_.image_size_.height);
    }
}

auto Camera::setFPS(unsigned value) -> void{
    this -> data_.fps_ = value;
    if (value <= 10) this -> data_.frame_speed_ = 0;
    else if(value <= 20) this -> data_.frame_speed_ = 1;
    else if(value <= 30) this -> data_.frame_speed_ = 2;
    else{throw std::runtime_error("FPS should be in range 1-30.");}
}

auto Camera::setCalibrationStatus(bool value) -> void{
    this -> data_.is_calibrated_ = value;
}

auto Camera::loadFromFile(std::string value) -> void{
    this -> data_.load_from_file_address_ = value;
    this -> data_.load_file = (this -> data_.load_from_file_address_.size() != 0);
}

auto Camera::setLightFrequency(int value) -> void{
    if (value == 50 or value == 60){
        this -> data_.light_frequency_ = value;
    }
    else{
        throw std::runtime_error("Image light frequency should be 50 or 60.");
    }
}

auto Camera::setAntiFlickFlag(bool value) -> void{
    this -> data_.anti_flick_ = value;
}

auto Camera::setMonochromeFlag(bool value) -> void{
    this -> data_.monochrome_ = value;
}

auto Camera::setColorInversionFlag(bool value) -> void{
    this -> data_.invert_color_ = value;
}

auto Camera::setSharpness(int value) -> void{
    if (value >= 0 and value <= 90){
        this -> data_.sharpness_ = value;
    }
    else{
        throw std::runtime_error("Sharpness Value should be in range 0-90");
    }
}

auto Camera::setAutoExposureFlag(bool value) -> void{
    this -> data_.auto_exposure_ = value;
}

auto Camera::setAutoExposureTarget(int value) -> void{
    if (value >= 20 and value <= 160){
        this -> data_.auto_exposure_target_ = value;
    }
    else{
        throw std::runtime_error("Auto-Exposure-Target Value should be in range 20-160");
    }
}

auto Camera::setContrast(int value) -> void{
    if (value >= 0 and value <= 190){
        this -> data_.contrast_ = value;
    }
    else{
        throw std::runtime_error("Contrast Value should be in range 0-190");
    }
}

auto Camera::setGamma(int value) -> void{
    if (value >= 0 and value <= 240){
        this -> data_.gamma_ = value;
    }
    else{
        throw std::runtime_error("Gamma Value should be in range 0-240");
    }
}

auto Camera::setAnalogGain(int value) -> void{
    if (value >= 5 and value <= 33){
        this -> data_.analog_gain_ = value;
    }
    else{
        throw std::runtime_error("Analog-Gain Value should be in range 5-33");
    }
}

auto Camera::setExposureTime(int value) -> void{
    if (value >= 16 and value <= 2097140){
        this -> data_.exposure_time_ = value;
    }
    else{
        throw std::runtime_error("Exposure-Time Value should be in range 16-2097140 microseconds.");
    }
}

auto Camera::setSaturation(int value) -> void{
    if (value >= 0 and value <= 100){
        this -> data_.saturation_ = value;
    }
    else{
        throw std::runtime_error("Saturation Value should be in range 16-2097140 microseconds.");
    }
}



auto Camera::setInitialPose(std::string initial_pose) -> void{
    std::vector<std::string> pose_vec{this->splitString(initial_pose, ',')};
    for(int i{}; i < 3; i++){
        this -> data_.initial_pose_.at(i) = std::stod(pose_vec.at(i));
    } 
}

auto Camera::setInitialOrientation(std::string initial_orientation) -> void{
    std::vector<std::string> orientation_vec{this->splitString(initial_orientation, ',')};
    for(int i{}; i < 3; i++){
        this -> data_.initial_orientation_.at(i) = std::stod(orientation_vec.at(i));
    } 
}

auto Camera::setCameraMatrix(std::vector<double> camera_matrix) -> void{
    data_.k = camera_matrix;
    this -> data_.camera_matrix_ = cv::Mat(camera_matrix, true).reshape(1,3);
    this -> data_.camera_matrix_.convertTo(this -> data_.camera_matrix_, CV_32FC1);
}

auto Camera::setDistortionModel(std::string value) -> void{
    data_.distortion_model_ = value;
}

auto Camera::setDistortionCoefficients(std::vector<double> dist_coeff) -> void{
    data_.d = dist_coeff;
    this -> data_.distortion_coefficients_ = cv::Mat(dist_coeff, true).reshape(1, 1);
    this -> data_.distortion_coefficients_.convertTo(this -> data_.distortion_coefficients_, CV_32FC1);
}

auto Camera::setRectificationMatrix(std::vector<double> rectification_matrix) -> void{
    data_.r = rectification_matrix;
    this -> data_.rectification_matrix_ = cv::Mat(rectification_matrix, true).reshape(1,3);
    this -> data_.rectification_matrix_.convertTo(this -> data_.rectification_matrix_, CV_32FC1);
}

auto Camera::setProjectionMatrix(std::vector<double> projection_matrix) -> void{
    data_.p = projection_matrix;
    this -> data_.projection_matrix_ = cv::Mat(projection_matrix, true).reshape(1,3);
    this -> data_.projection_matrix_.convertTo(this -> data_.projection_matrix_, CV_32FC1);
}

auto Camera::setSensorResolution() -> void{
    /*
        Levels:
        0: 2048x2048
        1: 2048x1536
        2: 1920x1200
        3: 1600x1200
        4: 640x480
        */
        std::unordered_map<std::string, int> levels{
            {"2048,2048", 0},
            {"2048,1536", 1},
            {"1920,1200", 2},
            {"1600,1200", 3},
            {"1280,1024", 4},
            {"640,480", 5}
        };

        // // std::replace(data_.resolution_.begin(), data_.resolution_.end(), ',', 'X');
        // if (auto search = levels.find(data_.resolution_); search != levels.end()){
        //     data_.tCapability_.pImageSizeDesc->iIndex = search -> second;
        // }        

        // data_.iStatus_ = CameraSetImageResolution(data_.hCamera_, data_.tCapability_.pImageSizeDesc);
        // if(data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetImageResolution failed.");}
        // CameraGetImageResolution(data_.hCamera_, data_.tCapability_.pImageSizeDesc);

}


auto Camera::getFPS() -> double{
    return this -> data_.fps_;
}

auto Camera::getIndex() -> std::string{
    return  this -> data_.index_;
}

auto Camera::splitString(std::string& input, char delimiter) -> std::vector<std::string>{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(input);
    while(std::getline(tokenStream, token, delimiter)){
        tokens.push_back(token);
    }
    return tokens;
}


auto Camera::warmUp() -> void{
    // SDK initialization
    CameraSdkInit(1);

    // Enumerate devices and create a device list
    this -> data_.iStatus_ = CameraEnumerateDevice(this -> data_.tCameraEnumList_, &this -> data_.iCameraCounts_);
    std::clog << "[INFO] Status: " << ((0 == this -> data_.iStatus_)?"Connected":"Disconnected") << "\n";
    std::clog << "[INFO] Camera Count: " <<  this -> data_.iCameraCounts_ << "\n";

    // No device connected
    if(this -> data_.iCameraCounts_==0){throw std::runtime_error("CameraEnumerateDevice failed.");}

    // Find the current srial number among GigE devices.
    bool found_flag{false};
    for (auto dev : this -> data_.tCameraEnumList_){
        std::string sn{dev.acSn};
        if (0 == sn.compare(this -> data_.serial_)){
            this -> data_.tCameraDevInfo_ = dev;
            found_flag = true;
            break;
        }
    }
    if(!found_flag){throw std::runtime_error("The GigE camera serial is not found among current devices.");}

    //Device initialization. Only after successful initialization can any other camera-related operation interfaces be called.
    this -> data_.iStatus_ = CameraInit(&this -> data_.tCameraDevInfo_, -1, -1, &this -> data_.hCamera_);
    std::clog << "[INFO] Status: " << ((0 == this -> data_.iStatus_)?"Connected":"Disconnected") << "\n";

    // Initialization failed
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraInit failed.");}

    /*Let the SDK enter working mode and start receiving images from the camera
    data. If the current camera is in trigger mode, it needs to receive.
    The image will not be updated until the frame is triggered.*/
    CameraPlay(this -> data_.hCamera_);

    /*Other camera parameter settings
    For example, CameraSetExposureTime CameraGetExposureTime sets/reads the exposure time
        CameraSetImageResolution CameraGetImageResolution set/read resolution
        CameraSetGamma, CameraSetConrast, CameraSetGain, etc. set image gamma, contrast, RGB digital gain, etc.
        This routine is just to demonstrate how to convert the image obtained in the SDK into the OpenCV image format so that the OpenCV image processing function can be called for subsequent development.
    */
    // Get the devices's feature description structure. This structure contains range information of 
    // various parameters that the camera can set. determines the parameters of the relevant functions

    CameraLoadParameter(this -> data_.hCamera_, PARAMETER_TEAM_A);
    
    // Set the trigger mode for the camera: continurous mode
    this -> data_.iStatus_ = CameraSetTriggerMode(this -> data_.hCamera_, 0);
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetTriggerMode failed.");}

    // Set auto exposure
    this -> data_.iStatus_ = CameraSetAeState(this -> data_.hCamera_, static_cast<int>(this -> data_.auto_exposure_));
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetAeState failed.");}

    if (this -> data_.auto_exposure_){
        // Set Anti-flick capability
        this -> data_.iStatus_ = CameraSetAntiFlick(this -> data_.hCamera_, this -> data_.anti_flick_);
        if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetAntiFlick failed.");}

        this -> data_.iStatus_ = CameraSetAeTarget(this -> data_.hCamera_, this -> data_.auto_exposure_target_ ); 
        if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetExposureTime failed.");}

        // Set light frequency 50Hz/60Hz -> it only works if the anti-flick is set.
        if(!this -> data_.anti_flick_){
            this -> data_.iStatus_ = CameraSetLightFrequency(this -> data_.hCamera_, static_cast<int>(50 == this -> data_.light_frequency_));
            if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetLightFrequency failed.");}
        }

    }
    else{
        // Get Exposure step size
        double m_fExpLineTime{};
        CameraGetExposureLineTime(this -> data_.hCamera_, &m_fExpLineTime);

        // Set Exposure time
        this -> data_.iStatus_ = CameraSetExposureTime(this -> data_.hCamera_, this -> data_.exposure_time_ - (static_cast<int>(this -> data_.exposure_time_) % static_cast<int>(m_fExpLineTime)) ); 
        if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetExposureTime failed.");}

        // Set Analog Gain
        if (!this -> data_.auto_exposure_){
            this -> data_.iStatus_ = CameraSetAnalogGain(this -> data_.hCamera_, this -> data_.analog_gain_);
            if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetAnalogGain failed.");}
        }

    }
    

    // Set Saturation
    this -> data_.iStatus_ = CameraSetSaturation(this -> data_.hCamera_, this -> data_.saturation_); 
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetSaturation failed.");}
    

    // Set Gamma
    this -> data_.iStatus_ = CameraSetGamma(this -> data_.hCamera_, this -> data_.gamma_);
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetGamma failed.");}

    // Set Monochrome capability
    this -> data_.iStatus_ = CameraSetMonochrome(this -> data_.hCamera_, this -> data_.monochrome_);
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetMonochrome failed.");}
    
    // Set Inverse color -> it works only if the monochrome is set.
    if (this -> data_.monochrome_){
        this -> data_.iStatus_ = CameraSetInverse(this -> data_.hCamera_, this -> data_.invert_color_);
        if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetInverse failed.");}
        
    }

    // Set Contrast
    this -> data_.iStatus_ = CameraSetContrast(this -> data_.hCamera_, this -> data_.contrast_);
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetContrast failed.");}
   
    // Setting the Resolution
    CameraGetCapability(this -> data_.hCamera_, &this -> data_.tCapability_);
    // this -> setSensorResolution();

    // Set Frame speed -> 0-10: 0, 11-20:1, 21-...:2
    this -> data_.iStatus_ = CameraSetFrameSpeed(this -> data_.hCamera_, this -> data_.frame_speed_);
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetFrameSpeed failed.");}
    
    // Set Sharpness
    this -> data_.iStatus_ = CameraSetSharpness(this -> data_.hCamera_, this -> data_.sharpness_);
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetSharpness failed.");}

    // Set Cam output format
    if(this -> data_.tCapability_.sIspCapacity.bMonoSensor or this -> data_.monochrome_){
        this -> data_.channel_= 1;
        this -> data_.iStatus_ = CameraSetIspOutFormat(this -> data_.hCamera_, CAMERA_MEDIA_TYPE_MONO8);
        this -> data_.out_format_ = CV_8UC1;
    }else{
        this -> data_.channel_= 3;
        this -> data_.iStatus_ = CameraSetIspOutFormat(this -> data_.hCamera_, CAMERA_MEDIA_TYPE_BGR8);
        this -> data_.out_format_ = CV_8UC3;
    }
    if(this -> data_.iStatus_!=CAMERA_STATUS_SUCCESS){throw std::runtime_error("CameraSetIspOutFormat failed.");}
    
    // Reserve Buffer to read from the cam
    int nSize{this -> data_.tCapability_.sResolutionRange.iHeightMax * this -> data_.tCapability_.sResolutionRange.iWidthMax * this -> data_.channel_}; 
    this -> data_.g_pRgbBuffer_ = (unsigned char*)malloc(nSize);

    cv::initUndistortRectifyMap(this -> data_.camera_matrix_, this -> data_.distortion_coefficients_, cv::Matx33f::eye(), this -> data_.camera_matrix_, this -> data_.image_size_, CV_32FC1, this -> data_.map1_, this -> data_.map2_);
    this -> logEverything();
}


auto Camera::prepareFrame() -> void{
    if(CameraGetImageBuffer(data_.hCamera_, &data_.sFrameInfo_,&data_.pbyBuffer_,1000) == CAMERA_STATUS_SUCCESS)
		{
		    CameraImageProcess(data_.hCamera_, data_.pbyBuffer_, data_.g_pRgbBuffer_, &data_.sFrameInfo_);
            cv::Mat matImage(data_.tCapability_.sResolutionRange.iWidthMax, data_.tCapability_.sResolutionRange.iHeightMax, data_.out_format_, (void*)data_.g_pRgbBuffer_);
            if(this -> data_.load_file){
                this -> data_.distorted_frame_ = matImage;
            }
            else{
            if (data_.image_size_.height < 2048 or data_.image_size_.width < 2048){
                this -> data_.distorted_frame_ = matImage(data_.ROI_);
            }
            else{
                this -> data_.distorted_frame_ = matImage;
            }
            }
            if(this -> data_.is_calibrated_){this -> undistortImage();}

            // cv::imshow("Opencv Demo", data_.distorted_frame_);
            // cv::waitKey(1);

            //After successfully calling CameraGetImageBuffer, CameraReleaseImageBuffer must be called to release the obtained buffer.
            //Otherwise, when CameraGetImageBuffer is called again, the program will be suspended and blocked until CameraReleaseImageBuffer is called in other threads to release the buffer.
			CameraReleaseImageBuffer(data_.hCamera_, data_.pbyBuffer_);
		}
}

auto Camera::undistortImage() -> void{
    this -> data_.undistorted_frame_ = cv::Mat::zeros(this -> data_.image_size_.height, this -> data_.image_size_.width, this -> data_.channel_);
    cv::remap(this -> data_.distorted_frame_, this -> data_.undistorted_frame_, this -> data_.map1_, this -> data_.map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
}

auto Camera::logEverything() -> void{
    CameraGetCapability(this -> data_.hCamera_, &this -> data_.tCapability_);

    std::clog << "[INFO] Product Series: " << this -> data_.tCameraDevInfo_.acProductSeries << std::endl;
    std::clog << "[INFO] Product Name: " << this -> data_.tCameraDevInfo_.acProductName << std::endl;
    std::clog << "[INFO] Friendly Name: " << this -> data_.tCameraDevInfo_.acFriendlyName << std::endl;
    std::clog << "[INFO] Link: " << this -> data_.tCameraDevInfo_.acLinkName << std::endl;
    std::clog << "[INFO] Driver Version: " << this -> data_.tCameraDevInfo_.acDriverVersion << std::endl;
    std::clog << "[INFO] Sensor Type: " << this -> data_.tCameraDevInfo_.acSensorType << std::endl;
    std::clog << "[INFO] Port: " << this -> data_.tCameraDevInfo_.acPortType << std::endl;
    std::clog << "[INFO] Serial Number: " << this -> data_.tCameraDevInfo_.acSn << std::endl;

    int ae_state{};
    CameraGetAeState(this -> data_.hCamera_, &ae_state);
    std::clog << "[INFO] Auto Exposure: " << (ae_state?"true":"false") << "\n";

    if(ae_state){
        int ae_target{};
        CameraGetAeTarget(this -> data_.hCamera_, &ae_target);
        std::clog << "[INFO] Auto Exposure Target: " << ae_target << "\n";

        CameraGetAntiFlick(this -> data_.hCamera_, &this -> data_.anti_flick_);
        std::clog << "[INFO] Anti-Flick: " << (this -> data_.anti_flick_ ? "true" : "false") << "\n";

        CameraGetLightFrequency(this -> data_.hCamera_, &this -> data_.light_frequency_);
        std::clog << "[INFO] Light Frequency: " << (this -> data_.light_frequency_ ? "50" : "60") << "Hz \n";
    }
    else{
        double m_fExpLineTime{};
        CameraGetExposureLineTime(this -> data_.hCamera_, &m_fExpLineTime);
        std::clog << "[INFO] Exposure Line Time: " << m_fExpLineTime << "\n";

        CameraGetExposureTime(this -> data_.hCamera_, &this -> data_.exposure_time_); 
        std::clog << "[INFO] Exposure Time: " << this -> data_.exposure_time_ << " micro_seconds\n";

        CameraGetAnalogGain(this -> data_.hCamera_, &this -> data_.analog_gain_);
        std::clog << "[INFO] Analog Gain: " << this -> data_.analog_gain_ << "\n";
    }

    CameraGetSaturation(this -> data_.hCamera_, &this -> data_.saturation_);
    std::clog << "[INFO] Saturation: " << this -> data_.saturation_ << "\n";

    CameraGetGamma(this -> data_.hCamera_, &this -> data_.gamma_);
    std::clog << "[INFO] Gamma: " << this -> data_.gamma_ << "\n";

    CameraGetMonochrome(this -> data_.hCamera_, &this -> data_.monochrome_);
    std::clog << "[INFO] Monochrome: " << (this -> data_.monochrome_?"true":"false") << "\n";

    CameraGetInverse(this -> data_.hCamera_, &this -> data_.invert_color_);
    std::clog << "[INFO] Invert color: " << (this -> data_.invert_color_?"true":"false") << "\n";

    CameraGetContrast(this -> data_.hCamera_, &this -> data_.contrast_);
    std::clog << "[INFO] Contrast: " << this -> data_.contrast_ << "\n";

    CameraGetFrameSpeed(this -> data_.hCamera_, &this -> data_.frame_speed_);
    std::clog << "[INFO] FPS: " << this -> data_.frame_speed_ << std::endl;

    CameraGetFrameSpeed(this -> data_.hCamera_, &this -> data_.sharpness_);
    std::clog << "[INFO] Sharpness: " << this -> data_.sharpness_ << std::endl;

    std::clog << "[INFO] Min Height: " << this -> data_.tCapability_.sResolutionRange.iHeightMin << ", Width: " << this -> data_.tCapability_.sResolutionRange.iWidthMin  << "\n";
    std::clog << "[INFO] Max Height: " << this -> data_.tCapability_.sResolutionRange.iHeightMax << ", Width: " << this -> data_.tCapability_.sResolutionRange.iWidthMax  << "\n";
    std::clog << "[INFO] uSkipModeMask: " << this -> data_.tCapability_.sResolutionRange.uSkipModeMask << ", uBinSumModeMask: " << this -> data_.tCapability_.sResolutionRange.uBinSumModeMask  << "\n";
    std::clog << "[INFO] uBinAverageModeMask: " << this -> data_.tCapability_.sResolutionRange.uBinAverageModeMask << ", uResampleMask: " << this -> data_.tCapability_.sResolutionRange.uResampleMask  << "\n";

    std::clog << "[INFO] iIndex: " << this -> data_.tCapability_.pImageSizeDesc->iIndex << ", acDescription: " << this -> data_.tCapability_.pImageSizeDesc->acDescription << "\n";
    std::clog << "[INFO] uBinSumMode: " << this -> data_.tCapability_.pImageSizeDesc->uBinSumMode << ", uBinAverageMode: " << this -> data_.tCapability_.pImageSizeDesc->uBinAverageMode << "\n";
    std::clog << "[INFO] uSkipMode: " << this -> data_.tCapability_.pImageSizeDesc->uSkipMode << ", uResampleMask: " << this -> data_.tCapability_.pImageSizeDesc->uResampleMask << "\n";
    std::clog << "[INFO] iHOffsetFOV: " << this -> data_.tCapability_.pImageSizeDesc->iHOffsetFOV << ", iVOffsetFOV: " << this -> data_.tCapability_.pImageSizeDesc->iVOffsetFOV << "\n";
    std::clog << "[INFO] iWidthFOV: " << this -> data_.tCapability_.pImageSizeDesc->iWidthFOV << ", iHeightFOV: " << this -> data_.tCapability_.pImageSizeDesc->iHeightFOV << "\n";
    std::clog << "[INFO] iWidth: " << this -> data_.tCapability_.pImageSizeDesc->iWidth << ", iHeight: " << this -> data_.tCapability_.pImageSizeDesc->iHeight << "\n";
    std::clog << "[INFO] ROI Width: " << this -> data_.image_size_.width << ", ROI Height: " << this -> data_.image_size_.height  << "\n";

    std::clog << "[INFO] iWidthZoomHd: " << this -> data_.tCapability_.pImageSizeDesc->iWidthZoomHd << ", iHeightZoomHd: " << this -> data_.tCapability_.pImageSizeDesc->iHeightZoomHd << "\n";
    std::clog << "[INFO] iWidthZoomSw: " << this -> data_.tCapability_.pImageSizeDesc->iWidthZoomSw << ", iHeightZoomSw: " << this -> data_.tCapability_.pImageSizeDesc->iHeightZoomSw << "\n";
    std::clog << "[INFO] Camera Matrix: " << this -> data_.camera_matrix_ << std::endl;

}