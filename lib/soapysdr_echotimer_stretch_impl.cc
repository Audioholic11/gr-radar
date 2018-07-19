/* -*- c++ -*- */
/*
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "soapysdr_echotimer_stretch_impl.h"

namespace gr {
  namespace radar {

    soapysdr_echotimer_stretch::sptr
    soapysdr_echotimer_stretch::make(int samp_rate, float center_freq, int num_delay_samps,
                              std::string args,
                              std::string antenna_tx, float gain_tx, float bw_tx,
                              float timeout_tx, float wait_tx, float lo_offset_tx,
                              std::string antenna_rx, float gain_rx, float bw_rx,
                              float timeout_rx, float wait_rx, float lo_offset_rx,
                              const std::string& len_key)
    {
      return gnuradio::get_initial_sptr
      ( new soapysdr_echotimer_impl (samp_rate, center_freq, num_delay_samps,
                                        args,
                                        antenna_tx, gain_tx, bw_tx,
                                        timeout_tx, wait_tx, lo_offset_tx,
                                        antenna_rx, gain_rx, bw_rx,
                                        timeout_rx, wait_rx, lo_offset_rx,
                                        len_key));
    }

    /*
     * The private constructor
     */
    soapysdr_echotimer_stretch_impl::soapysdr_echotimer_stretch_impl(int samp_rate, float center_freq, int num_delay_samps,
                                                      std::string args,
                                                      std::string antenna_tx, float gain_tx, float bw_tx,
                                                      float timeout_tx, float wait_tx, float lo_offset_tx,
                                                      std::string antenna_rx, float gain_rx, float bw_rx,
                                                      float timeout_rx, float wait_rx, float lo_offset_rx,
                                                      const std::string& len_key)
      : gr::tagged_stream_block("soapysdr_echotimer_stretch",
      gr::io_signature::make(1, 1, sizeof(gr_complex)),
      gr::io_signature::make(1, 1, sizeof(gr_complex)), len_key),
      //Parameters
        //EchoParameters
        d_num_delay_samps(num_delay_samps),
        //Common
        d_args(args),
        d_samp_rate(samp_rate),
        d_center_freq(center_freq),
        //Rx
        d_timeout_rx(timeout_rx),
        d_wait_rx(wait_rx),
        d_lo_offset_rx(lo_offset_rx),
        d_antenna_rx(antenna_rx),
        d_gain_rx(gain_rx),
        d_bw_rx(bw_rx),
        //Tx
        d_timeout_tx(timeout_tx),
        d_wait_tx(wait_tx),
        d_lo_offset_tx(lo_offset_tx),
        d_antenna_tx(antenna_tx),
        d_gain_tx(gain_tx),
        d_bw_tx(bw_tx)


    {

    }

    /*
     * Our virtual destructor.
     */
    soapysdr_echotimer_stretch_impl::~soapysdr_echotimer_stretch_impl()
    {
    }

    int
    soapysdr_echotimer_stretch_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = /* <+set this+> */;
      return noutput_items ;
    }

    int
    soapysdr_echotimer_stretch_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const <+ITYPE+> *in = (const <+ITYPE+> *) input_items[0];
      <+OTYPE+> *out = (<+OTYPE+> *) output_items[0];

      // Do <+signal processing+>

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace radar */
} /* namespace gr */
