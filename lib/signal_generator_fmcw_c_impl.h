/* -*- c++ -*- */
/*
 * Copyright 2014 Communications Engineering Lab, KIT.
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

#ifndef INCLUDED_RADAR_SIGNAL_GENERATOR_FMCW_C_IMPL_H
#define INCLUDED_RADAR_SIGNAL_GENERATOR_FMCW_C_IMPL_H

#include <radar/signal_generator_fmcw_c.h>

//colors
#include <colors.h>

namespace gr {
  namespace radar {

    class signal_generator_fmcw_c_impl : public signal_generator_fmcw_c
    {
     public:
      signal_generator_fmcw_c_impl(
        const int samp_rate,
        const int packet_len,
        int samp_up,
        int samp_up_hold,
        int samp_down,
        int samp_down_hold,
        int samp_cw,
        int samp_dead,
        const float freq_cw,
        const float freq_sweep,
        const float amplitude,
        const std::string& len_key,
        const std::string& chirp_len_key,
        const std::string& total_len_key
      );
      ~signal_generator_fmcw_c_impl();

      //Aux methods: Set Members (perform calcuations if necessary)
      void set_d_chirp_len(int chirp_length);
      void set_d_total_len(int total_length);
      void set_waveform();

      void set_d_samp_dead(int dead_samples);
      void set_d_samp_dead_round(int dead_samples);
      void set_chirp(int up_samples, int up_hold_samples,
                     int down_samples, int down_hold_samples,
                     int cw_samples);

      //work function
      int work(int noutput_items,
          gr_vector_const_void_star &input_items,
          gr_vector_void_star &output_items
      );

     private:


      const int d_samp_rate; //!< Sample rate of signal (sps)
      const int d_packet_len; //!< process packet length (samples)
      int d_samp_up; //!< Number of samples on the up-sweep
      int d_samp_up_hold; //!< Number of samples hold at top of ramp
      int d_samp_down; //!< Number of samples on the down-sweep
      int d_samp_down_hold; //!< Number of samples hold at buttom of ramp
      int d_samp_cw; //!< Number of samples on the CW part
      int d_samp_dead; //!< Number of unsent samples
      int d_chirp_len; //!< Total length of packet (up, down, CW)
      const float d_freq_cw; //!< Frequency of the CW part
      const float d_freq_sweep; //!< Sweep frequency
      const float d_amplitude; //!< Amplitude
      int d_total_len;

      //tags
      const pmt::pmt_t d_srcid; //!< srcid for tags

      const pmt::pmt_t d_key_len; //!< Tag identifier for TSB length tags
      const pmt::pmt_t d_value_len; //!< Precalculated value of TSB tag

      const pmt::pmt_t d_key_chirp_len; //!< Tag identifier for chirp len
      pmt::pmt_t d_value_chirp_len; //!< Precalculated value of chirp len tag

      const pmt::pmt_t d_key_total_len; //!< Tag identifier for total len
      pmt::pmt_t d_value_total_len; //!< Precalculated value of total len tag

      const pmt::pmt_t d_key_deadtime; //!< Tag identifier for deadtime
      pmt::pmt_t d_value_deadtime; //!< Precalculated value of deadtime tag
      bool d_counter_deadtime;

      std::complex<float> d_phase; //!< Store phase state
      std::complex<float> d_amp;
      std::vector<float> d_waveform; //!< Phase increment of the waveform
      std::vector<float> d_waveform_amp; //!< Amplitude increment of the waveform
      int d_wv_counter; //!< Stores the current position in the waveform

      //debug
      std::vector<int> process_aff;
      std::vector<int> process_aff_set;
    };

  } // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_SIGNAL_GENERATOR_FMCW_C_IMPL_H */
