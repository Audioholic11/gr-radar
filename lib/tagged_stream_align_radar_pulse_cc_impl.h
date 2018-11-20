/* -*- c++ -*- */
/*
 * Copyright 2018 Erik Moore.
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

#ifndef INCLUDED_RADAR_TAGGED_STREAM_ALIGN_RADAR_PULSE_CC_IMPL_H
#define INCLUDED_RADAR_TAGGED_STREAM_ALIGN_RADAR_PULSE_CC_IMPL_H

#include <radar/tagged_stream_align_radar_pulse_cc.h>

#include <colors.h>

namespace gr {
  namespace radar {

    class tagged_stream_align_radar_pulse_cc_impl : public tagged_stream_align_radar_pulse_cc
    {
     private:
      int d_chirp_len;
      int d_samp_rate;
      int d_debug_print;

      pmt::pmt_t d_key_len;
      pmt::pmt_t d_value_len;
      pmt::pmt_t d_srcid;

      int d_chirp_len_tag_value;
      float d_input_chirps;
      float d_output_chirps;
      float d_in_out_chirps_ratio;

      uint64_t d_last_chirp_len_offset;
      uint64_t d_chirp_len_offset_period;
      uint64_t d_last_input_sample_used;

      int d_overflow;

      uint64_t d_input_tag_shift;
      int d_output_tag_shift;

      //debug
      int workNum;
      int forcastNum;

     public:
      tagged_stream_align_radar_pulse_cc_impl(int samp_rate, const std::string& len_key, int chirp_len,int debug_print);
      ~tagged_stream_align_radar_pulse_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      void set_debug_print(int debug_print);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_TAGGED_STREAM_ALIGN_RADAR_PULSE_CC_IMPL_H */
