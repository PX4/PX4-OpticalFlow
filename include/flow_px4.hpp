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


class OpticalFlowPX4 : public OpticalFlow {

  private:
    //params which can be set
    PX4Flow *px4_flow;

  public:

    OpticalFlowPX4( float f_length_x, float f_length_y, int ouput_rate = DEFAULT_OUTPUT_RATE,
          int image_width = 64, int search_size = 5, int flow_feature_threshold = 30,
          int flow_value_threshold = 3000 );
    ~OpticalFlowPX4();

    int calcFlow(const cv::Mat &img_current, const uint32_t &img_time_us, int &dt_us,
      float &flow_x, float &flow_y);

};
