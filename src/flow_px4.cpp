/*
*  flow_px4.cpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*
****************************************************************************
* PX4Flow flow calculation
****************************************************************************/

#include "flow_px4.hpp"
#include <iostream>

OpticalFlowPX4::OpticalFlowPX4( float f_length_x, float f_length_y,
      int ouput_rate, int image_width, int search_size,
      int flow_feature_threshold, int flow_value_threshold)
{
  setFocalLengthX(f_length_x);
  setFocalLengthY(f_length_y);
  setOutputRate(ouput_rate);

  initLimitRate();

  //init the PX4Flow instance
  px4_flow = new PX4Flow(image_width, search_size, flow_feature_threshold, flow_value_threshold);
}

OpticalFlowPX4::~OpticalFlowPX4( void )
{

}

int OpticalFlowPX4::calcFlow(const cv::Mat &img_current, const uint32_t &img_time_us, int &dt_us,
  float &flow_x, float &flow_y){

  static cv::Mat img_old;

  if ( img_old.empty() ) {
    //first call of the function -> copy image for flow calculation
    img_current.copyTo(img_old);
    return 0;
  }

  //not needed
  float x_gyro_rate = 0;
  float y_gyro_rate = 0;
  float z_gyro_rate = 0;

  int flow_quality = px4_flow->compute_flow((uint8_t *)img_old.data, (uint8_t * )img_current.data,
    x_gyro_rate, y_gyro_rate, z_gyro_rate, &flow_x, &flow_y);

  img_current.copyTo(img_old);

  flow_quality = limitRate(flow_quality, img_time_us, &dt_us, &flow_x, &flow_y);

  flow_x = atan2(flow_x, focal_length_x); //convert pixel flow to angular flow
  flow_y = atan2(flow_y, focal_length_y); //convert pixel flow to angular flow

  return flow_quality;


}
