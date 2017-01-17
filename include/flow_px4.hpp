/*
*  flow_px4.hpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#pragma once

#include <cv.h>
#include "optical_flow.hpp"
#include "px4flow.hpp"

#define DEFAULT_IMAGE_WIDTH 64
#define DEFAULT_SEARCH_SIZE 6
#define DEFAULT_FLOW_FEATURE_THRESHOLD 30
#define DEFAULT_FLOW_VALUE_THRESHOLD 3000

class OpticalFlowPX4 : public OpticalFlow {

  private:
    //params which can be set
    PX4Flow *px4_flow;

  public:

    OpticalFlowPX4( float f_length_x, float f_length_y, int ouput_rate = DEFAULT_OUTPUT_RATE,
          int image_width = DEFAULT_IMAGE_WIDTH, int search_size = DEFAULT_SEARCH_SIZE,
          int flow_feature_threshold = DEFAULT_FLOW_FEATURE_THRESHOLD,
          int flow_value_threshold = DEFAULT_FLOW_VALUE_THRESHOLD );
    ~OpticalFlowPX4();

    int calcFlow(const cv::Mat &img_current, const uint32_t &img_time_us, int &dt_us,
      float &flow_x, float &flow_y);

};
