#include "GigEManager.hxx"

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
    this -> declare_parameter("monochrome", false);
    this -> declare_parameter("contrast", 100);
    this -> declare_parameter("gamma", 100);
    this -> declare_parameter("analog_gain", 25);
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
    this -> cam_.setMonochromeFlag(this->get_parameter("monochrome").as_bool());
    this -> cam_.setContrast(this->get_parameter("contrast").as_int());
    this -> cam_.setGamma(this->get_parameter("gamma").as_int());
    this -> cam_.setAnalogGain(this->get_parameter("analog_gain").as_int());
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

    if(this -> cam_.data_.is_calibrated_){
    this -> frame_ = cv_bridge::CvImage(this -> header_, "bgr8", this -> cam_.data_.undistorted_frame_).toImageMsg(); 
    }
    else{
        this -> frame_ = cv_bridge::CvImage(this -> header_, "bgr8", this -> cam_.data_.distorted_frame_).toImageMsg();
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
    rclcpp::Time now = rclcpp::Clock().now();
    header.stamp.sec = now.seconds();
    header.stamp.nanosec = now.nanoseconds();
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
}


void GigE::publishFrame(){
    // this -> image_transport_publisher_.publish(frame_);
    this -> image_transport_publisher_.publish(*this -> frame_, camera_info_);
}

void GigE::log(std::string phrase){
    std::string final_string{"[INFO] " + phrase};
    RCLCPP_INFO(this->get_logger(), final_string.c_str());
}

void GigE::log_err(std::string phrase){
    std::string final_string{"[ERROR] " + phrase};
    RCLCPP_ERROR(this->get_logger(), final_string.c_str());
}


    
