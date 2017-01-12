/*
*  optical_flow.h
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#include <stdint.h>
#include <iostream>
#include <cmath>

#pragma once

class OpticalFlow {

  protected:
    //params which can be set
    int image_width;
    int image_height;
    float focal_length_x; //[pixel]
    float focal_length_y; //[pixel]
    int output_rate;
    float sum_flow_x;
    float sum_flow_y;
    int sum_flow_quality;
    int valid_frame_count;

    //consts
    static const int DEFAULT_OUTPUT_RATE = 15;

    void initLimitRate();
    int limitRate(int flow_quality, const uint32_t frame_time_us, int *dt_us,
      float *flow_x, float *flow_y);

  public:

    inline void setImageWidth( int img_width ) { image_width = img_width; };
    inline void setImageHeight( int img_height ) { image_height = img_height; };
    inline void setFocalLengthX( float f_lengh ) { focal_length_x = f_lengh; };
    inline void setFocalLengthY( float f_lengh ) { focal_length_y = f_lengh; };
    inline void setOutputRate( int out_rate ) { output_rate = out_rate; }; //TODO check valid range 10-20?

    inline int getImageWidth() { return image_width; };
    inline int getImageHeight() { return image_height; };
    inline int getFocalLengthX() { return focal_length_x; };
    inline int getFocalLengthy() { return focal_length_y; };
    inline int getOutputRate() { return output_rate; };

    // virtual int calcPixelFlow(const cv::Mat &img_current, float &flow_x, float &flow_y);

};
