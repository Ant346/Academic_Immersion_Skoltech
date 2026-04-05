#include "GigEManager.hxx"
#include <iomanip>
#include <sstream>

GigE::GigE(std::string node_name): Node(node_name){
    this -> declare_parameter<std::string>("serial", "");
    this -> declare_parameter("index", "");
    this -> declare_parameter("resolution", "");
    this -> declare_parameter("fps", 0);
    this -> declare_parameter("is_calibrated", false);
    this -> declare_parameter("camera_matrix", std::vector<double>(9));
    this -> declare_parameter("distortion_model", "");
    this -> declare_parameter("distortion_coefficients", std::vector<double>(5));
    this -> declare_parameter("rectification_matrix", std::vector<double>(9));
    this -> declare_parameter("projection_matrix", std::vector<double>(12));
    this -> declare_parameter("initial_pose", "0.0,0.0,0.0");
    this -> declare_parameter("initial_orientation", "0.0,0.0,0.0");

    this -> declare_parameter("GigE_param_file", "");
    this -> declare_parameter("light_frequency", 50);
    this -> declare_parameter("antiflick", true);
    this -> declare_parameter("invert_color", false);
    this -> declare_parameter("sharpness", 50);
    this -> declare_parameter("auto_exposure", false);
    this -> declare_parameter("auto_exposure_min_time_us", 16.0);
    this -> declare_parameter("auto_exposure_max_time_us", 2097140.0);
    this -> declare_parameter("monochrome", false);
    this -> declare_parameter("contrast", 100);
    this -> declare_parameter("gamma", 100);
    this -> declare_parameter("analog_gain_x", 1.0);
    this -> declare_parameter("auto_exposure_min_gain_x", 1.0);
    this -> declare_parameter("auto_exposure_max_gain_x", 4.0);
    this -> declare_parameter("exposure_time", 500);
    this -> declare_parameter("auto_exposure_target", 50);
    this -> declare_parameter("saturation", 80);
    this -> declare_parameter("queue_size", 10);

    this -> cam_.setSerial(this->get_parameter("serial").as_string());
    this -> cam_.setIndex(this->get_parameter("index").as_string());
    this -> cam_.setResolution(this->get_parameter("resolution").as_string());
    this -> cam_.setFPS(this->get_parameter("fps").as_int());
    this -> cam_.setCalibrationStatus(this->get_parameter("is_calibrated").as_bool());
    this -> cam_.setCameraMatrix(this->get_parameter("camera_matrix").as_double_array());
    this -> cam_.setDistortionModel(this->get_parameter("distortion_model").as_string());
    this -> cam_.setDistortionCoefficients(this->get_parameter("distortion_coefficients").as_double_array());
    this -> cam_.setRectificationMatrix(this->get_parameter("rectification_matrix").as_double_array());
    this -> cam_.setProjectionMatrix(this->get_parameter("projection_matrix").as_double_array());
    this -> cam_.setInitialPose(this->get_parameter("initial_pose").as_string());
    this -> cam_.setInitialOrientation(this->get_parameter("initial_orientation").as_string());

    this -> cam_.loadFromFile(this->get_parameter("GigE_param_file").as_string());
    this -> cam_.setLightFrequency(this->get_parameter("light_frequency").as_int());
    this -> cam_.setAntiFlickFlag(this->get_parameter("antiflick").as_bool());
    this -> cam_.setColorInversionFlag(this->get_parameter("invert_color").as_bool());
    this -> cam_.setSharpness(this->get_parameter("sharpness").as_int());
    this -> cam_.setAutoExposureFlag(this->get_parameter("auto_exposure").as_bool());
    this -> cam_.setAutoExposureMinTime(this->get_parameter("auto_exposure_min_time_us").as_double());
    this -> cam_.setAutoExposureMaxTime(this->get_parameter("auto_exposure_max_time_us").as_double());
    this -> cam_.setMonochromeFlag(this->get_parameter("monochrome").as_bool());
    this -> cam_.setContrast(this->get_parameter("contrast").as_int());
    this -> cam_.setGamma(this->get_parameter("gamma").as_int());
    this -> cam_.setAnalogGainX(this->get_parameter("analog_gain_x").as_double());
    this -> cam_.setAutoExposureMinGainX(this->get_parameter("auto_exposure_min_gain_x").as_double());
    this -> cam_.setAutoExposureMaxGainX(this->get_parameter("auto_exposure_max_gain_x").as_double());
    this -> cam_.setExposureTime(this->get_parameter("exposure_time").as_int());
    this -> cam_.setAutoExposureTarget(this->get_parameter("auto_exposure_target").as_int());
    this -> cam_.setSaturation(this->get_parameter("saturation").as_int());

    this -> queue_size_ = this->get_parameter("queue_size").as_int();
    // this -> frame_id_ = node_name;
    this -> frame_id_ = "Camera_" + this -> cam_.data_.index_;
}

GigE::~GigE(){
    std::cout << "GigE Destructor!\n";
    this -> image_transport_publisher_.shutdown();
    this->sleep(500);
}

void GigE::sleep(unsigned millisec){
    std::this_thread::sleep_for(std::chrono::milliseconds(millisec));
}

void GigE::setPublisher(){
    this -> image_transport_ = std::make_shared<image_transport::ImageTransport>(this -> node_);
    this -> image_transport_publisher_ = this -> image_transport_->advertiseCamera("Camera_" + this -> cam_.data_.index_ + "/color", this -> queue_size_);
    this -> status_publisher_ = this -> node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("Camera_" + this -> cam_.data_.index_ + "/status", this -> queue_size_);
}

void GigE::warmUp(){
    node_ = this -> create_sub_node("gige");

    this -> setPublisher();
    this -> cam_.warmUp();
}


void GigE::getFrame(){
    this -> cam_.prepareFrame();
    this -> header_ = this -> getHeader();
    this -> getCameraInfo();
    const char *encoding = (this -> cam_.data_.channel_ == 1) ? "mono8" : "bgr8";

    if(this -> cam_.data_.is_calibrated_){
    this -> frame_ = cv_bridge::CvImage(this -> header_, encoding, this -> cam_.data_.undistorted_frame_).toImageMsg(); 
    }
    else{
        this -> frame_ = cv_bridge::CvImage(this -> header_, encoding, this -> cam_.data_.distorted_frame_).toImageMsg();
    }

}

auto GigE::getCameraInfo() -> void{
    this -> camera_info_.header = this -> header_;
    this -> camera_info_.height = this -> cam_.data_.image_size_.height;
    this -> camera_info_.width = this -> cam_.data_.image_size_.width;
    this -> camera_info_.distortion_model = this -> cam_.data_.distortion_model_;
    this -> camera_info_.d.resize(5);
    for(short i{}; i < 5; i++)
        this -> camera_info_.d[i] = cam_.data_.d[i];
    for(short i{}; i < 9; i++)
        this -> camera_info_.k[i] = cam_.data_.k[i];
    for(short i{}; i < 9; i++)
        this -> camera_info_.r[i] = cam_.data_.r[i];
    for(short i{}; i < 12; i++)
        this -> camera_info_.p[i] = cam_.data_.p[i];
}

std_msgs::msg::Header GigE::getHeader(){
    std_msgs::msg::Header header{};
    header.stamp = this->get_clock()->now();
    header.frame_id = this -> frame_id_;
    return header;
}

void GigE::loopForEver(){
    // std::chrono::milliseconds fps_duration{static_cast<unsigned>(1000.0/this -> cam_.getFPS())};
    if(rclcpp::ok()){
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1ms), std::bind(&GigE::operate, this));
    }
}

void GigE::operate(){
    this -> getFrame();
    this -> publishFrame();
    this -> publishMetrics();
}


void GigE::publishFrame(){
    // this -> image_transport_publisher_.publish(frame_);
    this -> image_transport_publisher_.publish(*this -> frame_, camera_info_);
    ++this -> published_frames_;
}

void GigE::publishMetrics(){
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - this -> fps_window_start_).count();
    if (elapsed < 1.0){
        return;
    }

    const auto to_string = [](auto value) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3) << value;
        return stream.str();
    };

    const double ros_publish_fps = static_cast<double>(this -> published_frames_) / elapsed;

    double exposure_time{};
    CameraGetExposureTime(this -> cam_.data_.hCamera_, &exposure_time);

    float analog_gain{};
    CameraGetAnalogGainX(this -> cam_.data_.hCamera_, &analog_gain);

    tSdkFrameStatistic frame_statistic{};
    CameraGetFrameStatistic(this -> cam_.data_.hCamera_, &frame_statistic);

    UINT resend_count{};
    CameraGetStatisticResend(this -> cam_.data_.hCamera_, &resend_count);

    const int total_delta = frame_statistic.iTotal - this -> previous_frame_total_;
    const int captured_delta = frame_statistic.iCapture - this -> previous_frame_captured_;
    const int lost_delta = frame_statistic.iLost - this -> previous_frame_lost_;
    const unsigned int resend_delta = resend_count - this -> previous_frame_resend_;

    const double camera_total_fps = static_cast<double>(std::max(0, total_delta)) / elapsed;
    const double camera_capture_fps = static_cast<double>(std::max(0, captured_delta)) / elapsed;

    int pack_index{-1};
    CameraGetTransPackLen(this -> cam_.data_.hCamera_, &pack_index);
    int pack_size_bytes{0};
    std::string pack_description{"unknown"};
    for (int i{}; i < this -> cam_.data_.tCapability_.iPackLenDesc; ++i){
        const auto &pack_desc = this -> cam_.data_.tCapability_.pPackLenDesc[i];
        if (pack_desc.iIndex == pack_index){
            pack_size_bytes = static_cast<int>(pack_desc.iPackSize);
            pack_description = pack_desc.acDescription;
            break;
        }
    }

    diagnostic_msgs::msg::DiagnosticStatus status{};
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "Camera_" + this -> cam_.data_.index_;
    status.hardware_id = this -> cam_.data_.serial_;
    status.message = "camera settings and runtime metrics";

    const auto append_value = [&status](const std::string &key, const std::string &value) {
        diagnostic_msgs::msg::KeyValue item{};
        item.key = key;
        item.value = value;
        status.values.push_back(item);
    };

    append_value("resolution", std::to_string(this -> cam_.data_.image_size_.width) + "x" + std::to_string(this -> cam_.data_.image_size_.height));
    append_value("encoding", (this -> cam_.data_.channel_ == 1) ? "mono8" : "bgr8");
    append_value("fps", to_string(camera_capture_fps));
    append_value("camera_total_fps", to_string(camera_total_fps));
    append_value("camera_capture_fps", to_string(camera_capture_fps));
    append_value("ros_publish_fps", to_string(ros_publish_fps));
    append_value("auto_exposure", this -> cam_.data_.auto_exposure_ ? "true" : "false");
    append_value("auto_exposure_target", to_string(this -> cam_.data_.auto_exposure_target_));
    append_value("exposure_us", to_string(exposure_time));
    append_value("auto_exposure_min_time_us", to_string(this -> cam_.data_.auto_exposure_min_time_us_));
    append_value("auto_exposure_max_time_us", to_string(this -> cam_.data_.auto_exposure_max_time_us_));
    append_value("analog_gain_x", to_string(static_cast<double>(analog_gain)));
    append_value("auto_exposure_min_gain_x", to_string(this -> cam_.data_.auto_exposure_min_gain_x_));
    append_value("auto_exposure_max_gain_x", to_string(this -> cam_.data_.auto_exposure_max_gain_x_));
    append_value("transport_pack_index", std::to_string(pack_index));
    append_value("transport_pack_size_bytes", std::to_string(pack_size_bytes));
    append_value("transport_pack_description", pack_description);
    append_value("frame_total", std::to_string(frame_statistic.iTotal));
    append_value("frame_captured", std::to_string(frame_statistic.iCapture));
    append_value("frame_lost", std::to_string(frame_statistic.iLost));
    append_value("frame_resend", std::to_string(resend_count));
    append_value("frame_total_delta", std::to_string(std::max(0, total_delta)));
    append_value("frame_captured_delta", std::to_string(std::max(0, captured_delta)));
    append_value("frame_lost_delta", std::to_string(std::max(0, lost_delta)));
    append_value("frame_resend_delta", std::to_string(resend_delta));
    append_value("contrast", std::to_string(this -> cam_.data_.contrast_));
    append_value("gamma", std::to_string(this -> cam_.data_.gamma_));
    append_value("saturation", std::to_string(this -> cam_.data_.saturation_));
    append_value("sharpness", std::to_string(this -> cam_.data_.sharpness_));
    append_value("light_frequency_hz", std::to_string(this -> cam_.data_.light_frequency_));
    append_value("antiflick", this -> cam_.data_.anti_flick_ ? "true" : "false");
    append_value("is_calibrated", this -> cam_.data_.is_calibrated_ ? "true" : "false");

    diagnostic_msgs::msg::DiagnosticArray array{};
    array.header = this -> header_;
    array.status.push_back(status);
    this -> status_publisher_->publish(array);

    this -> previous_frame_total_ = frame_statistic.iTotal;
    this -> previous_frame_captured_ = frame_statistic.iCapture;
    this -> previous_frame_lost_ = frame_statistic.iLost;
    this -> previous_frame_resend_ = resend_count;
    this -> published_frames_ = 0;
    this -> fps_window_start_ = now;
}

void GigE::log(std::string phrase){
    std::string final_string{"[INFO] " + phrase};
    RCLCPP_INFO(this->get_logger(), final_string.c_str());
}

void GigE::log_err(std::string phrase){
    std::string final_string{"[ERROR] " + phrase};
    RCLCPP_ERROR(this->get_logger(), final_string.c_str());
}


    
