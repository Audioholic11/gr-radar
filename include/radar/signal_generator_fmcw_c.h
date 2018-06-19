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

#ifndef INCLUDED_RADAR_SIGNAL_GENERATOR_FMCW_C_H
#define INCLUDED_RADAR_SIGNAL_GENERATOR_FMCW_C_H

#include <radar/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace radar {

    /*! Generates a signal for FMCW radar in baseband.
     *
     * The generated signal consists of three parts, in this order:
     *
     * 1. CW part with constant frequency
     * 2. Up-chirp
     * 3. Down-chirp
     *
     * The up-chirp goes from CW frequency to CW frequency plus sweep frequency
     * and the down-chirp goes in the opposite direction. All of these parts
     * can be disabled by setting the corresponding length to zero.
     *
     * The packet length for subsequent tagged streams is calculated by the sum
     * of the number of samples of the single modulations parts.
     *
     * \ingroup radar
     */
    class RADAR_API signal_generator_fmcw_c : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<signal_generator_fmcw_c> sptr;

      /*!
       * \param samp_rate Signal sample rate (samples per second)
       * \param packet_len Processing packet length
       * \param samp_up Number samples of up-chirp part
       * \param samp_up_hold Number of samples of up_chirp hold part
       * \param samp_down Number samples of down-chirp part
       * \param samp_down_hold Number of samples of down_chirp hold part
       * \param samp_cw Number samples of CW part
       * \param samp_dead Number of dead samples
       * \param freq_cw CW signal frequency in baseband
       * \param freq_sweep Sweep frequency of up- and down-chirp
       * \param amplitude Signal amplitude
       * \param len_key Processing Packet length key for tagged stream
       * \param chirp_len_key Chirp Length Key for tagged stream
       */
      static sptr make(
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
        const std::string& len_key="packet_len",
        const std::string& chirp_len_key="chirp_len",
        const std::string& total_len_key="total_len"
      );

      //Member Set
      virtual void set_d_chirp_len(int chirp_length) = 0;
      virtual void set_d_total_len(int total_length) = 0;
      virtual void set_waveform() = 0;
      virtual void set_d_samp_dead(int dead_samples) = 0;
      virtual void set_d_samp_dead_round(int dead_samples) = 0;
      virtual void set_chirp(int up_samples, int up_hold_samples,
                     int down_samples, int down_hold_samples,
                     int cw_samples) = 0;
    };

  } // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_SIGNAL_GENERATOR_FMCW_C_H */
