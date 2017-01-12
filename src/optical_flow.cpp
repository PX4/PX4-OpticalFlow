/*
*  optical_flow.cpp
*
*  Created on: Jan 12, 2017
*      Author: Christoph
*
****************************************************************************/

#include "optical_flow.hpp"

void OpticalFlow::initLimitRate()
{
  sum_flow_x = 0.0;
  sum_flow_y = 0.0;
  valid_frame_count = 0;
  sum_flow_quality = 0;
}

int OpticalFlow::limitRate(int flow_quality, const uint32_t frame_time_us, int *dt_us,
  float *flow_x, float *flow_y)
{

  static uint32_t time_last_pub = 0;

  if (output_rate <= 0) { //don't limit the rate
    *dt_us = frame_time_us - time_last_pub;
    time_last_pub = frame_time_us;
    return flow_quality;
  }

  if (flow_quality > 0) {
    sum_flow_x += *flow_x;
    sum_flow_y += *flow_y;
    sum_flow_quality += flow_quality;
    valid_frame_count++;
  }
  //limit rate according to parameter ouput_rate
  if ((frame_time_us - time_last_pub) > (1.0e6f/output_rate)) {

    int average_flow_quality = 0;

    //average the flow over the period since last update
    if (valid_frame_count > 0)
      average_flow_quality = std::floor(sum_flow_quality / valid_frame_count);

    *flow_x = sum_flow_x;
    *flow_y = sum_flow_y;

    //reset variables
    initLimitRate();
    *dt_us = frame_time_us - time_last_pub;
    time_last_pub = frame_time_us;

    return average_flow_quality;

  } else {

    return -1; //signaling that it should not yet publish the values

  }

}
