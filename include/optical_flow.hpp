/*
*  optical_flow.h
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#pragma once

class OpticalFlow {

  protected:
    //params which can be set
    int image_width;
    int image_height;
    float focal_length_x; //[pixel]
    float focal_length_y; //[pixel]

  public:

    inline void setImageWidth( int img_width ) { image_width = img_width; };
    inline void setImageHeight( int img_height ) { image_height = img_height; };
    inline void setFocalLengthX( float f_lengh ) { focal_length_x = f_lengh; };
    inline void setFocalLengthY( float f_lengh ) { focal_length_y = f_lengh; };

    inline int getImageWidth() { return image_width; };
    inline int getImageHeight() { return image_height; };
    inline int getFocalLengthX() { return focal_length_x; };
    inline int getFocalLengthy() { return focal_length_y; };

    // virtual int calcPixelFlow(const cv::Mat &img_current, float &flow_x, float &flow_y);

};
