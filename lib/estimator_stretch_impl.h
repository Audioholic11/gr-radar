/* -*- c++ -*- */
/*
 * Copyright 2018 Erik Moore DU2SRI.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_RADAR_ESTIMATOR_STRETCH_IMPL_H
#define INCLUDED_RADAR_ESTIMATOR_STRETCH_IMPL_H

#include <radar/estimator_stretch.h>

namespace gr {
  namespace radar {

    class estimator_stretch_impl : public estimator_stretch
    {
     private:
       int d_samp_rate;
       int d_samp_up;
       int d_samp_down;
       float d_center_freq;
       float d_bandwidth;
       float d_freq_offset;
       int d_chirp_len;

     public:
      estimator_stretch_impl(int samp_rate, float center_freq, float bandwidth, int chirp_len, int samp_up, int samp_down, float DC_freq_offset );
      ~estimator_stretch_impl();


       // Methods
       void handle_msg_stretch(pmt::pmt_t msg);
       void estimate();
       void set_dc_freq_offset(float freq_offset);

       //Constant Variables
       float d_const_stretch;
       constexpr static float c_light = 299792458;

       //Message Variables
       bool d_msg_stretch_in;
       pmt::pmt_t d_port_id_in_stretch,d_port_id_out;
       pmt::pmt_t d_msg_stretch;
    };

  } // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_ESTIMATOR_STRETCH_IMPL_H */
