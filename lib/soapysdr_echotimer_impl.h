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
        //timing
        volatile long long d_time_now_tx, d_time_now_rx,d_time_now_rx_HW;
        long long d_timeNs_tx, d_timeNs_rx;
        long long d_timeNs_rx_return;
        volatile long long d_wait_rx_TEST;
        long d_timeoutUs;
        pmt::pmt_t d_time_key, d_time_val, d_srcid;
        //EchoParameters
        int d_num_delay_samps;
        //common
        std::string d_args;
        SoapySDR::Kwargs d_kw;
        int d_samp_rate;
        float d_center_freq;
        //Aux
        std::vector<std::string> d_listRegInterfaces;
        std::vector<std::string> d_GPIOBanks;
        volatile unsigned d_register;
        volatile unsigned d_value;
        volatile float d_time_packet;
        long long d_extra_work_time;
        int d_workNum;
        bool d_SendPacket;
        pmt::pmt_t DeadtimeKey = pmt::string_to_symbol("Deadtime");
        volatile int firstPacket_Rx;
        volatile int firstPacket;
        int ERRORS;
        int Rx_Skip_Packets = 0;
        bool Chirp_Sync;



        // Tx/Rx
        SoapySDR::Device *d_soapysdr_tx, *d_soapysdr_rx, *d_soapysdr;
        SoapySDR::Stream *d_tx_stream, *d_rx_stream;

        size_t d_chan_tx, d_chan_rx;
        int d_flagsTx, d_flagsRx,d_flagsRx_return;

        float d_timeout_tx, d_timeout_rx;
        float d_wait_tx, d_wait_rx;
        double d_lo_offset_tx, d_lo_offset_rx;
        std::string d_antenna_tx, d_antenna_rx;
        float d_gain_tx, d_gain_rx;
        float d_bw_tx, d_bw_rx;

        volatile int d_num_rx_samps, d_num_tx_samps;

        //receive stream variables
        gr::thread::thread d_thread_recv;
        gr_vector_void_star d_out_buffer;//std::vector<gr_complex>
        //std::vector<gr_complex *> d_out_buffer;
        gr_complex *  d_out_recv;//input to "readStream"
        int d_noutput_items_recv;

        //send stream variables
        gr::thread::thread d_thread_send;
        gr_vector_const_void_star d_in_buffer;//input to "writeStream"
        int d_noutput_items_send;

    public:

        soapysdr_echotimer_impl(int samp_rate, float center_freq, int num_delay_samps,
                                  std::string args,
                                  std::string antenna_tx, float gain_tx, float bw_tx,
                                  float timeout_tx, float wait_tx, float lo_offset_tx,
                                  std::string antenna_rx, float gain_rx, float bw_rx,
                                  float timeout_rx, float wait_rx, float lo_offset_rx,
                                  const std::string& len_key);

        ~soapysdr_echotimer_impl();


        // methods
        void send();
        void receive();
        //
        void set_rx_gain(size_t chan, float gain);
        void set_tx_gain(size_t chan, float gain);

        //variable set and gets
        int num_delay_samps() { return d_num_delay_samps; }
        void set_num_delay_samps(int num_delay_samps) { d_num_delay_samps = num_delay_samps; }

        float wait_rx() { return d_wait_rx; }
        float wait_tx() { return d_wait_tx; }
        void set_wait(float wait_rx, float wait_tx) { d_wait_rx = wait_rx; d_wait_tx = wait_tx; }

        //int calculate_output_stream_length(const gr_vector_int &ninput_items);

        // Where all the action really happens
        int work(int noutput_items,
          gr_vector_int &ninput_items,
          gr_vector_const_void_star &input_items,
          gr_vector_void_star &output_items);
        };
      } // namespace radar
    } // namespace gr

    #endif /* INCLUDED_RADAR_SOAPYSDR_ECHOTIMER_IMPL_H */
