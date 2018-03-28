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
#include "soapysdr_echotimer_impl.h"

#include <iostream>

namespace gr {
  namespace radar {
    soapysdr_echotimer::sptr
    soapysdr_echotimer::make(int samp_rate, float center_freq, int num_delay_samps,
                              std::string args, std::string antenna_tx, float gain_tx,
                              float timeout_tx, float wait_tx, float lo_offset_tx,
                              std::string antenna_rx, float gain_rx,
                              float timeout_rx, float wait_rx, float lo_offset_rx,
                              const std::string& len_key)
      {
        return gnuradio::get_initial_sptr
        ( new soapysdr_echotimer_impl (samp_rate, center_freq, num_delay_samps,
                                          args, antenna_tx, gain_tx,
                                          timeout_tx, wait_tx, lo_offset_tx,
                                          antenna_rx, gain_rx,
                                          timeout_rx, wait_rx, lo_offset_rx,
                                          len_key));
        }

      /*
      * The private constructor
      */
      soapysdr_echotimer_impl::soapysdr_echotimer_impl(int samp_rate, float center_freq, int num_delay_samps,
                                                        std::string args, std::string antenna_tx, float gain_tx,
                                                        float timeout_tx, float wait_tx, float lo_offset_tx,
                                                        std::string antenna_rx, float gain_rx,
                                                        float timeout_rx, float wait_rx, float lo_offset_rx,
                                                        const std::string& len_key)
      : gr::tagged_stream_block("soapysdr_echotimer",
      gr::io_signature::make(1, 1, sizeof(gr_complex)),
      gr::io_signature::make(1, 1, sizeof(gr_complex)), len_key)
      {
        d_samp_rate = samp_rate;
        d_center_freq = center_freq;
        d_num_delay_samps = num_delay_samps;
        d_args = args;
        d_out_buffer.resize(0);


        d_timeNs = 0;
        d_timeoutUs = 0;


      //***** Setup Soapy / gr-osmosdr TX *****//
      d_antenna_tx = antenna_tx;
      d_lo_offset_tx = lo_offset_tx;
      d_gain_tx = gain_tx;
      d_timeout_tx = timeout_tx; // timeout for sending
      d_wait_tx = wait_tx; // secs to wait befor sending
      d_chan_tx = 0;

      //d_kw = SoapySDR::Device::enumerate();
      d_kw = SoapySDR::KwargsFromString(d_args);
      // Setup Soapysdr TX: args (addr,...)
      //d_soapysdr = SoapySDR::Device::make(params_to_dict(d_args_tx));
      d_soapysdr = SoapySDR::Device::make(d_kw);
      std::cout << "Using Soapy Device (TX): " << std::endl << d_soapysdr->getHardwareKey() << std::endl;

      // Setup Soapysdr TX: sample rate
      std::cout << "Setting TX Rate: " << d_samp_rate << std::endl;
      //setSampleRate(const int direction, const size_t channel, const double rate);
      d_soapysdr->setSampleRate(SOAPY_SDR_TX, d_chan_tx, d_samp_rate);
      //getSampleRate(const int direction, const size_t channel);
      std::cout << "Actual TX Rate: " << d_soapysdr->getSampleRate(SOAPY_SDR_TX, d_chan_tx) << std::endl;

      // Setup Soapysdr TX: gain
      set_tx_gain(d_chan_tx, d_gain_tx);

      // Setup Soapysdr TX: set frequency (tune?)
      //setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const Kwargs &args = Kwargs());
      d_soapysdr->setFrequency(SOAPY_SDR_TX, d_chan_tx, d_center_freq);
      std::cout << "Set TX Frequency: " << std::endl << d_soapysdr->getFrequency(SOAPY_SDR_TX, d_chan_tx) << std::endl;

      // Setup Soapysdr TX: antenna
      //setAntenna(const int direction, const size_t channel, const std::string &name);
      d_soapysdr->setAntenna(SOAPY_SDR_TX, d_chan_tx, d_antenna_tx);
      std::cout << "Set TX Antenna: " << std::endl << d_soapysdr->getAntenna(SOAPY_SDR_TX, d_chan_tx) << std::endl;


      // Setup Soapysdr TX: timestamp
      //setHardwareTime(const long long timeNs, const std::string &what = "");
      d_soapysdr->setHardwareTime(0.0); // Do set time on startup if not gpsdo is activated.

      // Setup transmit streamer
      //Stream *setupStream( const int direction, const std::string &format, const std::vector<size_t> &channels = std::vector<size_t>(), const Kwargs &args = Kwargs());
      d_tx_stream = d_soapysdr->setupStream(SOAPY_SDR_TX, "CF32");//might need channels

      //uhd::stream_args_t stream_args_tx("fc32", d_wire_tx); // complex floats
      //d_tx_stream = d_soapysdr->get_tx_stream(stream_args_tx);

      //***** Setup USRP RX *****//

      d_antenna_rx = antenna_rx;
      d_lo_offset_rx = lo_offset_rx;
      d_gain_rx = gain_rx;
      d_timeout_rx = timeout_rx; // timeout for receiving
      d_wait_rx = wait_rx; // secs to wait befor receiving
      d_chan_rx = 0;

      //d_kw = SoapySDR::KwargsFromString(d_args);
      // Setup USRP RX: args (addr,...)
      //d_soapysdr_rx = SoapySDR::Device::make(d_kw);
      //std::cout << "Using Soapy Device (RX): " << std::endl << d_soapysdr_rx->getHardwareKey() << std::endl;

      // Setup USRP RX: sample rate
      std::cout << "Setting RX Rate: " << d_samp_rate << std::endl;
      //setSampleRate(const int direction, const size_t channel, const double rate);
      d_soapysdr->setSampleRate(SOAPY_SDR_RX, d_chan_rx, d_samp_rate);
      //getSampleRate(const int direction, const size_t channel);
      std::cout << "Actual RX Rate: " << d_soapysdr->getSampleRate(SOAPY_SDR_RX, d_chan_rx) << std::endl;

      // Setup USRP RX: gain
      set_rx_gain(d_chan_rx, d_gain_rx);

      // Setup Soapysdr RX: set frequency (tune?)
      //setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const Kwargs &args = Kwargs());
      d_soapysdr->setFrequency(SOAPY_SDR_RX, d_chan_rx, d_center_freq);
      std::cout << "Set RX Frequency: " << std::endl << d_soapysdr->getFrequency(SOAPY_SDR_RX, d_chan_rx) << std::endl;

      // Setup Soapysdr RX: antenna
      //setAntenna(const int direction, const size_t channel, const std::string &name);
      d_soapysdr->setAntenna(SOAPY_SDR_RX, d_chan_rx, d_antenna_rx);
      std::cout << "Set RX Antenna: " << std::endl << d_soapysdr->getAntenna(SOAPY_SDR_RX, d_chan_rx) << std::endl;


      d_soapysdr->setHardwareTime(0.0);
      // Setup receive streamer

      d_rx_stream = d_soapysdr->setupStream(SOAPY_SDR_RX, "CF32");//might need channels
      // d_soapysdr->activateStream(d_rx_stream)


      //***** Misc *****//

      // Setup rx_time pmt
      d_time_key = pmt::string_to_symbol("rx_time");
      d_srcid = pmt::string_to_symbol("soapysdr_echotimer");

      // Setup thread priority
      //uhd::set_thread_priority_safe(); // necessary? doesnt work...

      // Sleep to get sync done
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000)); // FIXME: necessary?

    }

    /*
    * Our virtual destructor.
    */
    soapysdr_echotimer_impl::~soapysdr_echotimer_impl()
    {
      d_soapysdr->closeStream(d_rx_stream);
      d_soapysdr->closeStream(d_tx_stream);
    }

    int
    soapysdr_echotimer_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = ninput_items[0];
      return noutput_items ;
    }

    void
    soapysdr_echotimer_impl::set_num_delay_samps(int num_samps)
    {
      d_num_delay_samps = num_samps;
    }

    void
    soapysdr_echotimer_impl::set_rx_gain( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_RX, chan, gain);
      std::cout << "RX Gain: " << std::endl << d_soapysdr->getGain(SOAPY_SDR_RX, chan) << std::endl;
    }

    void
    soapysdr_echotimer_impl::set_tx_gain( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_TX, chan, gain);
      std::cout << "TX Gain: " << std::endl << d_soapysdr->getGain(SOAPY_SDR_TX, chan) << std::endl;
    }

    void
    soapysdr_echotimer_impl::send()
    {
      // Send input buffer
      size_t num_tx_samps, total_num_samps;
      total_num_samps = d_noutput_items_send;
      //Data to Soapy _device
      //virtual int writeStream(
      //        Stream *stream,
      //        const void * const *buffs,
      //        const size_t numElems,
      //        int &flags,
      //        const long long timeNs = 0,
      //        const long timeoutUs = 100000);


      //SoapyLMS7->streaming->fifo.h
      //SYNC_TIMESTAMP = 1,
      //END_BURST = 2,
      //OVERWRITE_OLD = 4,

      //soapyflags
      //SOAPY_SDR_HAS_TIME = 1;
      //SOAPY_SDR_END_BURST = 2;
      flagsTx = SOAPY_SDR_HAS_TIME;
      d_timeNs = (d_time_now_tx+d_wait_tx)*1E9;
      d_timeoutUs = (total_num_samps/(float)d_samp_rate+d_timeout_tx)*1E6;

      num_tx_samps = d_soapysdr->writeStream(d_tx_stream, &d_in_send[0], total_num_samps, flagsTx, d_timeNs, d_timeoutUs);

      // Get timeout
      if (num_tx_samps < total_num_samps) std::cerr << "Send timeout..." << std::endl;

      //send a mini EOB packet
      int flagsTx = SOAPY_SDR_END_BURST;
      d_soapysdr->writeStream(d_tx_stream, 0, 0, flagsTx);
      //d_soapysdr->writeStream(d_tx_stream, 0, 0, flags, timeNs, timeoutUs);
    }

    void
    soapysdr_echotimer_impl::receive()
    {
      // Setup RX streaming
      size_t total_num_samps = d_noutput_items_recv;

      size_t num_rx_samps;
      // Receive a packet
      //virtual int readStream(
      //    Stream *stream,
      //    void * const *buffs,
      //    const size_t numElems,
      //    int &flags,
      //    long long &timeNs,
      //    const long timeoutUs = 100000);


      d_timeNs = (d_time_now_rx+d_wait_rx)*1E9;
      d_timeoutUs = (total_num_samps/(float)d_samp_rate+d_timeout_rx)*1E6;
      num_rx_samps = d_soapysdr->readStream(d_rx_stream, &d_out_recv[0], total_num_samps, flagsRx, d_timeNs, d_timeoutUs);

      // Save timestamp
      d_time_val = pmt::make_tuple
      (pmt::from_uint64(d_timeNs/1E9));

      // Handle the error code
      //if (d_metadata_rx.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
      //	throw std::runtime_error(str(boost::format("Receiver error %s") % d_metadata_rx.strerror()));
      //}

      if (num_rx_samps < total_num_samps) std::cerr << "Receive timeout before all samples received..." << std::endl;

    }

  // Where the work is done
    int
    soapysdr_echotimer_impl::work (int noutput_items,
    gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
    {
      //gr_complex *in = (gr_complex *) input_items[0]; // remove const
      gr_complex *out = (gr_complex *) output_items[0];

      std::cout << "Start Work routine"<< std::endl;
      // Set output items on packet length
      noutput_items = ninput_items[0];

      // Resize output buffer
      if(d_out_buffer.size()!=noutput_items) d_out_buffer.resize(noutput_items);

      // Get time from Soapys TX
      d_time_now_tx = d_soapysdr->getHardwareTime();
      d_time_now_rx = d_time_now_tx;

      // Send thread
      //d_in_send = in;//input to "writeStream"
      d_in_send = input_items;
      d_noutput_items_send = noutput_items;
      d_thread_send = gr::thread::thread(boost::bind(&soapysdr_echotimer_impl::send, this));

      // Receive thread
      d_out_recv = d_out_buffer;//input to "readStream"
      d_noutput_items_recv = noutput_items;
      d_thread_recv = gr::thread::thread(boost::bind(&soapysdr_echotimer_impl::receive, this));

      // Wait for threads to complete
      d_thread_send.join();
      d_thread_recv.join();

      // Shift of number delay samples (fill with zeros)
      memcpy(out,&d_out_buffer[0]+d_num_delay_samps,(noutput_items-d_num_delay_samps)*sizeof(gr_complex)); // push buffer to output
      memset(out+(noutput_items-d_num_delay_samps),0,d_num_delay_samps*sizeof(gr_complex)); // set zeros

      // Setup rx_time tag
      add_item_tag(0, nitems_written(0), d_time_key, d_time_val, d_srcid);

      std::cout << "End Work routine"<< std::endl;
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }
  } /* namespace radar */
} /* namespace gr */
