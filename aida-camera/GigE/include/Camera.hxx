#ifndef __CAMERA_HXX__
#define __CAMERA_HXX__

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>
#include <exception>

// Extras
#include "Data.hxx"

class Camera {
public:
  ~Camera();

  auto setIndex(std::string) -> void ;
  auto setSerial(std::string)-> void;
  auto setResolution(std::string)-> void;
  auto setFPS(unsigned)-> void;
  auto setInitialPose(std::string)-> void;
  auto setInitialOrientation(std::string)-> void;
  auto setCalibrationStatus(bool)-> void;

  auto loadFromFile(std::string)-> void;
  auto setLightFrequency(int)-> void;
  auto setAntiFlickFlag(bool)-> void;
  auto setColorInversionFlag(bool)-> void;
  auto setSharpness(int)-> void;
  auto setAutoExposureFlag(bool)-> void;
  auto setAutoExposureTarget(int)-> void;
  auto setMonochromeFlag(bool)-> void;
  auto setContrast(int)-> void;
  auto setGamma(int)-> void;
  auto setAnalogGain(int)-> void;
  auto setExposureTime(int)-> void;
  auto setSaturation(int)-> void;



  auto setCameraMatrix(std::vector<double>)-> void;
  auto setDistortionModel(std::string)-> void;
  auto setDistortionCoefficients(std::vector<double>)-> void;
  auto setRectificationMatrix(std::vector<double>)-> void;
  auto setProjectionMatrix(std::vector<double>)-> void;

  auto setSensorResolution()-> void;



  // Getter
  auto getFPS() -> double;
  auto getIndex() -> std::string;

  // Extras
  auto splitString(std::string &, char) -> std::vector<std::string>;
  auto logEverything()-> void;

  // Operation
  auto warmUp()-> void;
  auto prepareFrame()-> void;
  auto undistortImage()-> void;

  // protected:
  Data data_;

};

#endif
