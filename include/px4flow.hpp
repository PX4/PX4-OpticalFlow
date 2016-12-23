/*
*  px4flow.hpp
*
*  Created on: Dec 21, 2016
*      Author: Christoph
*/

#pragma once

#include <stdint.h>

class PX4Flow {

  private:
    //params which can be set
    uint32_t image_width;
    uint32_t search_size;
    uint32_t flow_feature_threshold;
    uint32_t flow_value_threshold;

    uint32_t __USAD8(uint32_t val1, uint32_t val2);
    uint32_t __USADA8(uint32_t val1, uint32_t val2, uint32_t val3);
    uint32_t __UHADD8(uint32_t val1, uint32_t val2);
    uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size);
    uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y,
        uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size);
    uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y,
        uint16_t off2X, uint16_t off2Y, uint16_t row_size);


  public:

    PX4Flow( uint32_t image_width_, uint32_t search_size_,
      uint32_t flow_feature_threshold_, uint32_t flow_value_threshold_);
    uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate,
        float z_rate, float *pixel_flow_x, float *pixel_flow_y);

};
