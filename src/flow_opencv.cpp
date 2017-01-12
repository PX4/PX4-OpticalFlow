/*
*  flow_opencv.cpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#include "flow_opencv.hpp"

/****************************************************************************
 * OpenCV optical flow calculation
 ****************************************************************************/

 OpticalFlowOpenCV::OpticalFlowOpenCV( float f_length_x, float f_length_y, int ouput_rate, int num_feat, float conf_multi ) :
   num_features(num_feat),
   confidence_multiplier(conf_multi)
{
   setFocalLengthX(f_length_x);
   setFocalLengthY(f_length_y);
   setOutputRate(ouput_rate);

   initLimitRate();
 }

OpticalFlowOpenCV::~OpticalFlowOpenCV( void )
{

}

int OpticalFlowOpenCV::calcFlow(const cv::Mat &img_current, const uint32_t &img_time_us, int &dt_us,
  float &flow_x, float &flow_y) {

  if (updateVector.empty())
    updateVector.resize(num_features, 2);

  int meancount = 0;
	float pixel_flow_x_mean = 0.0;
	float pixel_flow_y_mean = 0.0;
	float pixel_flow_x_stddev = 0.0;
	float pixel_flow_y_stddev = 0.0;

	trackFeatures( img_current, img_current, features_current, useless, updateVector, 0 );

  //TODO undistort points? not necessary if small field of view?

  if ( !features_current.empty() && !features_previous.empty() ) {
    //calculate pixel flow
    for ( int i = 0; i < updateVector.size(); i++ ) {
      //just use active features
      if (updateVector[i] == 1) {
        pixel_flow_x_mean += features_current[i].x - features_previous[i].x;
        pixel_flow_y_mean += features_current[i].y - features_previous[i].y;
        meancount++;
      }
    }
    //check if there are active features
    if ( meancount ) {
      pixel_flow_x_mean /= meancount;
      pixel_flow_y_mean /= meancount;

      //calculate variance
      for ( int i = 0; i < updateVector.size(); i++ ) {
        if (updateVector[i] == 1) {
          pixel_flow_x_stddev += pow(features_current[i].x - features_previous[i].x - pixel_flow_x_mean, 2);
          pixel_flow_y_stddev += pow(features_current[i].y - features_previous[i].y - pixel_flow_y_mean, 2);
        }
      }
      //convert to standard deviation
      pixel_flow_x_stddev = sqrt(pixel_flow_x_stddev / meancount);
      pixel_flow_y_stddev = sqrt(pixel_flow_y_stddev / meancount);

      //recalculate pixel flow with 90% confidence interval
      float temp_flow_x_mean = 0.0;
      float temp_flow_y_mean = 0.0;
      meancount = 0;

      for ( int i = 0; i < updateVector.size(); i++ ) {
        //check if active
        if ( updateVector[i] == 1 ) {
          //flow of feature i
          float temp_flow_x = features_current[i].x - features_previous[i].x;
          float temp_flow_y = features_current[i].y - features_previous[i].y;
          //check if inside confidence interval

          if ( fabs(temp_flow_x - pixel_flow_x_mean) < pixel_flow_x_stddev*confidence_multiplier &&
               fabs(temp_flow_y - pixel_flow_y_mean) < pixel_flow_y_stddev*confidence_multiplier ) {
            temp_flow_x_mean += temp_flow_x;
            temp_flow_y_mean += temp_flow_y;
            meancount++;
          } else {
            updateVector[i] = 0;
          }
        }
      }
      if ( meancount ) {
        //new mean
        pixel_flow_x_mean = temp_flow_x_mean / meancount;
        pixel_flow_y_mean = temp_flow_y_mean / meancount;
      }
    }
  }

  //remember features
  features_previous = features_current;
  //update feature status
  for (int i = 0; i < updateVector.size(); i++) {
    //new and now active
    if (updateVector[i] == 2) {
      updateVector[i] = 1;
    }
    //inactive
    if (updateVector[i] == 0) {
      updateVector[i] = 2;
    }
  }

  //output
  flow_x = pixel_flow_x_mean;
  flow_y = pixel_flow_y_mean;

  int flow_quality = round(255.0 * meancount / updateVector.size());

  flow_quality = limitRate(flow_quality, img_time_us, &dt_us, &flow_x, &flow_y);

  flow_x = atan2(flow_x, focal_length_x); //convert pixel flow to angular flow
  flow_y = atan2(flow_y, focal_length_y); //convert pixel flow to angular flow

  return flow_quality;
}
