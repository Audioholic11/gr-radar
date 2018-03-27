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

#ifndef INCLUDED_RADAR_SOAPYSDR_ECHOTIMER_IMPL_H
#define INCLUDED_RADAR_SOAPYSDR_ECHOTIMER_IMPL_H

#include <radar/soapysdr_echotimer.h>

//#include <uhd/utils/thread_priority.hpp>
//#include <uhd/usrp/multi_usrp.hpp>

//soapy
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Time.hpp>//?
#include <SoapySDR/Version.hpp>

//local
//#include "soapy_source_c.h"
//#include "soapy_common.h"
//#include "arg_helpers.h"
//#include "source_iface.h"
//#include "arg_helpers.h"


//boost
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/thread/thread.hpp>
//#include <boost/thread/mutex.hpp>
//#include <boost/assign.hpp>
//#include <boost/format.hpp>
//#include <boost/lexical_cast.hpp>

//osmosdr
//#include <osmosdr/ranges.h>
//#include "osmosdr/source.h"//why local?

//gnuradio
//#include <gnuradio/blocks/null_source.h>
//#include <gnuradio/blocks/null_sink.h>
//#include <gnuradio/blocks/throttle.h>
//#include <gnuradio/constants.h>
//#include <gnuradio/block.h>
//#include <gnuradio/sync_block.h>



namespace gr {
  namespace radar {

    class soapysdr_echotimer_impl : public soapysdr_echotimer
    {

    protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);


    private:
      //variables
      //common
      SoapySDR::Kwargs d_kw;
      int d_samp_rate;
      float d_center_freq;
      int d_num_delay_samps;
      long long d_timeNs;
      long d_timeoutUs;
      pmt::pmt_t d_time_key, d_time_val, d_srcid;

      // Tx/Rx
      SoapySDR::Device *d_soapysdr_tx, *d_soapysdr_rx;
      SoapySDR::Stream *d_tx_stream, *d_rx_stream;

      size_t d_chan_tx, d_chan_rx;
      int flagsTx, flagsRx;

      std::string d_args_tx, d_args_rx;
      std::string d_clock_source_tx, d_clock_source_rx;
      std::string d_wire_tx, d_wire_rx;
      std::string d_antenna_tx, d_antenna_rx;
      std::string d_time_source_tx, d_time_source_rx;

      double d_lo_offset_tx, d_lo_offset_rx;
      float d_timeout_tx, d_timeout_rx;
      float d_wait_tx, d_wait_rx;
      float d_gain_tx, d_gain_rx;

      long long d_time_now_tx, d_time_now_rx;

      //receive stream variables
      gr::thread::thread d_thread_recv;
      gr_vector_void_star d_out_buffer;//std::vector<gr_complex>
      gr_vector_void_star d_out_recv;//input to "readStream"
      int d_noutput_items_recv;


      //send stream variables
      gr::thread::thread d_thread_send;
      gr_vector_const_void_star d_in_send;//input to "writeStream"
      int d_noutput_items_send;

    public:

        soapysdr_echotimer_impl(int samp_rate, float center_freq, int num_delay_samps,
        std::string args_tx, std::string wire_tx, std::string clock_source_tx, std::string time_source_tx,
        std::string antenna_tx, float gain_tx, float timeout_tx, float wait_tx, float lo_offset_tx,
        std::string args_rx, std::string wire_rx, std::string clock_source_rx, std::string time_source_rx,
        std::string antenna_rx, float gain_rx, float timeout_rx, float wait_rx, float lo_offset_rx,
        const std::string& len_key);

        ~soapysdr_echotimer_impl();


        // methods
        void send();
        void receive();
        void set_num_delay_samps(int num_samps);
        void set_rx_gain(size_t chan, float gain);
        void set_tx_gain(size_t chan, float gain);


        // Where all the action really happens
        int work(int noutput_items,
          gr_vector_int &ninput_items,
          gr_vector_const_void_star &input_items,
          gr_vector_void_star &output_items);
        };
      } // namespace radar
    } // namespace gr

    #endif /* INCLUDED_RADAR_SOAPYSDR_ECHOTIMER_IMPL_H */
