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
                              std::string args,
                              std::string antenna_tx, float gain_tx, float bw_tx,
                              float timeout_tx, float wait_tx, float lo_offset_tx,
                              std::string antenna_rx, float gain_rx, float bw_rx,
                              float timeout_rx, float wait_rx, float lo_offset_rx,
                              const std::string& len_key)
      {return gnuradio::get_initial_sptr
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
      soapysdr_echotimer_impl::soapysdr_echotimer_impl(int samp_rate, float center_freq, int num_delay_samps,
                                                        std::string args,
                                                        std::string antenna_tx, float gain_tx, float bw_tx,
                                                        float timeout_tx, float wait_tx, float lo_offset_tx,
                                                        std::string antenna_rx, float gain_rx, float bw_rx,
                                                        float timeout_rx, float wait_rx, float lo_offset_rx,
                                                        const std::string& len_key)
      : gr::tagged_stream_block("soapysdr_echotimer",
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

      //#define BOARD_GPIO_OVRD (6 << 5)
      //SoapySDRDevice_writeRegister(sdr, *registers, BOARD_GPIO_DIR, 0x00FF);
      d_out_buffer.resize(0);
      d_timeNs_tx = 0;
      d_timeNs_rx = 0;
      d_timeoutUs = 10000000;
      d_time_now_rx = 0;
      d_time_now_tx = 0;
      d_timeNs_rx_return = 1e9;

      //******************* Setup Soapy RX *******************//
      d_chan_rx = 0;

      d_kw = SoapySDR::KwargsFromString(d_args);
      // Setup USRP RX: args (addr,...)
      d_soapysdr = SoapySDR::Device::make(d_kw);
      SoapySDR::Kwargs  HWINFO = d_soapysdr->getHardwareInfo();
      SoapySDR::ArgInfoList StreamArg = d_soapysdr->getStreamArgsInfo(SOAPY_SDR_RX,d_chan_rx);
      std::vector<std::string> StreamFormats = d_soapysdr->getStreamFormats(SOAPY_SDR_RX,d_chan_rx);



      std::cout << " Using Soapy Device (RX): " << SoapySDR::KwargsToString(HWINFO) << std::endl;
      std::cout << " Using Driver (RX): " << d_soapysdr->getDriverKey() << std::endl;
      std::cout << " Using HW (RX): " << d_soapysdr->getHardwareKey() << std::endl;
      std::cout << " has HW Time? (RX): " << d_soapysdr->hasHardwareTime() << std::endl;
      std::cout << " clock rate (RX): " << d_soapysdr->getMasterClockRate() << std::endl;
      std::cout << " num channels (RX): " << d_soapysdr->getNumChannels(SOAPY_SDR_RX) << std::endl;
      //std::cout << "stream format (RX): " << StreamFormats[0]<< StreamFormats[1] << std::endl;
      //std::cout << "stream native (RX): " << d_soapysdr->getNativeStreamFormat(SOAPY_SDR_RX,d_chan_rx,1) << std::endl <<
      //std::cout << "stream args (RX): " << SoapySDR::KwargsToString(StreamArg.key[1]) << std::endl;

      //Setup Register Devices
      d_listRegInterfaces = d_soapysdr->listRegisterInterfaces();
      for(int i = 0; i < d_listRegInterfaces.size(); i ++)
        std::cout << "Register Interfaces: " << i << " " << d_listRegInterfaces[i] << std::endl;

      //Turn on Fan
      d_register = 0x00CC;
      d_value = 1;
      d_soapysdr->writeRegister(d_listRegInterfaces[0],d_register,d_value);
      d_register = 0x00CD;
      d_soapysdr->writeRegister(d_listRegInterfaces[0],d_register,d_value);

      //GPIO Setup
      //Setup GPIO OVERLOAD
      d_register = 6<<5;

      //d_value = 0xFFE0;// GPIO(0-4) overload bits = 0
      d_value = 0xFF80;// GPIO(0-6) overload bits = 0


      d_soapysdr->writeRegister(d_listRegInterfaces[0], d_register,d_value);
      d_value = d_soapysdr->readRegister(d_listRegInterfaces[0],d_register);
      std::cout << std::hex << "Register GPIO Overload: 0x"  << d_register << " 0x"  <<  d_value << std::endl;
      std::cout << std::dec;

      //GPIO Output Setup
      d_GPIOBanks = d_soapysdr->listGPIOBanks();
      for(int i = 0; i < d_GPIOBanks.size(); i ++)
        std::cout << "GPIO Banks: " << i << " " << d_GPIOBanks[i] << std::endl;

      d_value = 0x00; d_soapysdr->writeGPIO(d_GPIOBanks[0],d_value);

      //All Pins Outputs
      //d_value = 0xFB;//all out but bit 6
      d_value = 0x00;//all in (overwrite)
      d_soapysdr->writeGPIODir(d_GPIOBanks[0],d_value);

      // Setup Soapy RX: sample rate
      std::cout << "Setting RX Rate: " << d_samp_rate << std::endl;
      //setSampleRate(const int direction, const size_t channel, const double rate);
      d_soapysdr->setSampleRate(SOAPY_SDR_RX, d_chan_rx, d_samp_rate);
      //getSampleRate(const int direction, const size_t channel);
      std::cout << "Actual RX Rate: " << d_soapysdr->getSampleRate(SOAPY_SDR_RX, d_chan_rx) << std::endl;
      std::cout << "Actual Master Clock Rate: " << d_soapysdr->getMasterClockRate() << std::endl;

      // Setup USRP RX: gain
      set_rx_gain(d_chan_rx, d_gain_rx);

      // Setup Soapysdr RX: set frequency (tune?)
      //setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const Kwargs &args = Kwargs());
      d_soapysdr->setFrequency(SOAPY_SDR_RX, d_chan_rx, d_center_freq);
      std::cout << "Set RX Frequency: "  << d_soapysdr->getFrequency(SOAPY_SDR_RX, d_chan_rx) << std::endl;

      // Setup Soapysdr RX: antenna
      //setAntenna(const int direction, const size_t channel, const std::string &name);
      d_soapysdr->setAntenna(SOAPY_SDR_RX, d_chan_rx, d_antenna_rx);
      std::cout << "Set RX Antenna: "  << d_soapysdr->getAntenna(SOAPY_SDR_RX, d_chan_rx) << std::endl;

      //void setBandwidth(const int direction, const size_t channel, const double bw);
      d_soapysdr->setBandwidth(SOAPY_SDR_RX, d_chan_rx, d_bw_rx);
      std::cout << "Set RX BW: "  << d_soapysdr->getBandwidth(SOAPY_SDR_RX, d_chan_rx) << std::endl;

      //d_soapysdr->setHardwareTime(0.0);
      // Setup receive streamer

      d_rx_stream = d_soapysdr->setupStream(SOAPY_SDR_RX, "CF32");//might need channels
      d_soapysdr->setHardwareTime(0);



      //std::cout << "Set RX Antenna: " << getStreamMTU(d_rx_stream) << std::endl;
      //std::cout << "List TX GPIO Banks: " << d_soapysdr->listGPIOBanks() << std::endl;

      //******************* Setup Soapy TX *******************//
      d_chan_tx = 0;

      //d_kw = SoapySDR::Device::enumerate();
      d_kw = SoapySDR::KwargsFromString(d_args);
      // Setup Soapysdr TX: args (addr,...)
      //d_soapysdr = SoapySDR::Device::make(params_to_dict(d_args_tx));
      //d_soapysdr = SoapySDR::Device::make(d_kw);
      d_soapysdr = d_soapysdr;
      std::cout << "Using Soapy Device (TX): "  << d_soapysdr->getHardwareKey() << std::endl;

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
      std::cout << "Set TX Frequency: "  << d_soapysdr->getFrequency(SOAPY_SDR_TX, d_chan_tx) << std::endl;

      // Setup Soapysdr TX: antenna
      //setAntenna(const int direction, const size_t channel, const std::string &name);
      d_soapysdr->setAntenna(SOAPY_SDR_TX, d_chan_tx, d_antenna_tx);
      std::cout << "Set TX Antenna: "  << d_soapysdr->getAntenna(SOAPY_SDR_TX, d_chan_tx) << std::endl;


      d_soapysdr->setBandwidth(SOAPY_SDR_TX, d_chan_tx, d_bw_tx);
      std::cout << "Set TX BW: "  << d_soapysdr->getBandwidth(SOAPY_SDR_TX, d_chan_tx) << std::endl;

      // Setup Soapysdr TX: timestamp
      //setHardwareTime(const long long timeNs, const std::string &what = "");
      //d_soapysdr->setHardwareTime(0.0); // Do set time on startup if not gpsdo is activated.

      // Setup transmit streamer
      //Stream *setupStream( const int direction, const std::string &format, const std::vector<size_t> &channels = std::vector<size_t>(), const Kwargs &args = Kwargs());
      d_tx_stream = d_soapysdr->setupStream(SOAPY_SDR_TX, "CF32");//might need channels
      //d_soapysdr->activateStream(d_tx_stream);
      //uhd::stream_args_t stream_args_tx("fc32", d_wire_tx); // complex floats
      //d_tx_stream = d_soapysdr->get_tx_stream(stream_args_tx);
      //std::cout << "List TX GPIO Banks: " << d_soapysdr->listGPIOBanks() << std::endl;


      //***** Misc *****//

      // Setup rx_time pmt
      d_time_key = pmt::string_to_symbol("rx_time");
      d_srcid = pmt::string_to_symbol("soapysdr_echotimer");

      // Setup thread priority
      //uhd::set_thread_priority_safe(); // necessary? doesnt work...
      std::cout << "Actual Master Clock Rate: [MHz]" << d_soapysdr->getMasterClockRate()/1e6 << std::endl;
      // Sleep to get sync done
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000)); // FIXME: necessary?

      //Aux variables start default
      d_extra_work_time = 0;
      d_workNum = 0;
      d_SendPacket = 1;
      firstPacket_Rx = 0;
      firstPacket = 1;
      d_flagsRx_return = 0;

      d_flagsTx = SOAPY_SDR_HAS_TIME;
      d_flagsTx = 0;
      d_soapysdr->activateStream(d_tx_stream,d_flagsTx);
      //d_flagsRx = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
      //d_flagsRx = SOAPY_SDR_HAS_TIME;
      //d_soapysdr->activateStream(d_rx_stream,d_flagsRx,0);

    }

    /*
    * Our virtual destructor.
    */
    soapysdr_echotimer_impl::~soapysdr_echotimer_impl()
    {
      d_soapysdr->writeRegister(d_listRegInterfaces[1], 6<<5, 0xFFFF);
      d_soapysdr->deactivateStream(d_rx_stream);
      d_soapysdr->deactivateStream(d_tx_stream);
      d_soapysdr->closeStream(d_rx_stream);
      d_soapysdr->closeStream(d_tx_stream);
    }

    // Input based block needs to know output stream length
    // This case is based on input port 0 (only input port)
    int
    soapysdr_echotimer_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = ninput_items[0];
      return noutput_items ;
    }


    // ***************  Implemented Variable set and gets *********************
    //int soapysdr_echotimer_impl::num_delay_samps() { return d_num_delay_samps; }
    //void soapysdr_echotimer_impl::set_num_delay_samps(int num_delay_samps) { d_num_delay_samps = num_delay_samps; }


    /*
    void
    soapysdr_echotimer_impl::set_rx_freq( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_RX, chan, gain);
      std::cout << "RX Gain: "  << d_soapysdr->getGain(SOAPY_SDR_RX, chan) << std::endl;
    }

    void
    soapysdr_echotimer_impl::set_tx_freq( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_TX, chan, gain);
      std::cout << "TX Gain: "  << d_soapysdr->getGain(SOAPY_SDR_TX, chan) << std::endl;
    }*/

    void
    soapysdr_echotimer_impl::set_rx_gain( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_RX, chan, gain);
      std::cout << "RX Gain: "  << d_soapysdr->getGain(SOAPY_SDR_RX, chan) << std::endl;
    }

    void
    soapysdr_echotimer_impl::set_tx_gain( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_TX, chan, gain);
      std::cout << "TX Gain: "  << d_soapysdr->getGain(SOAPY_SDR_TX, chan) << std::endl;
    }

    void
    soapysdr_echotimer_impl::send()
    {
      // Send input buffer
      size_t total_num_samps = d_noutput_items_send;
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

      d_timeNs_tx = d_time_now_tx + ((long long) (d_wait_tx*1000000000));

      d_timeoutUs = d_timeout_tx*1000000;

      //d_flagsTx = 0;
      //d_soapysdr->activateStream(d_tx_stream,d_flagsTx);

      //d_flagsTx =  SOAPY_SDR_HAS_TIME  | SOAPY_SDR_ONE_PACKET;
      d_flagsTx =  SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
      d_num_tx_samps = d_soapysdr->writeStream(d_tx_stream, &d_in_buffer[0], total_num_samps, d_flagsTx, d_timeNs_tx, d_timeoutUs);
      if (d_num_tx_samps != total_num_samps)
      {
        std::cout << "Tx Samples: " << d_num_tx_samps << std::endl;
      }
      //d_flagsTx = SOAPY_SDR_END_BURST;
      //d_num_tx_samps = d_soapysdr->writeStream(d_tx_stream, &d_in_buffer[0], 0, d_flagsTx, 0, d_timeoutUs);

    }

    void
    soapysdr_echotimer_impl::receive()
    {
      // Setup RX streaming
      size_t total_num_samps = d_noutput_items_recv;

      d_timeNs_rx = d_time_now_rx + ((long long)(d_wait_rx*1000000000));

      d_timeoutUs = d_timeout_rx*10000000;

      //d_flagsRx =  SOAPY_SDR_HAS_TIME | SOAPY_SDR_ONE_PACKET;
      //d_flagsRx =  SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
      d_flagsRx =  0;

      d_soapysdr->activateStream(d_rx_stream, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST , d_timeNs_rx, total_num_samps);
      d_num_rx_samps = d_soapysdr->readStream(d_rx_stream, &d_out_buffer[0], total_num_samps, d_flagsRx, d_timeNs_rx_return, d_timeoutUs);
      //d_timeNs_rx_return = d_timeNs_rx;
      if (d_num_rx_samps != total_num_samps)
      {
      std::cout << "Dropped Rx Samples: " << d_num_rx_samps << std::endl;
      }
    }

  // Where the work is done
    int
    soapysdr_echotimer_impl::work (int noutput_items,
    gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
    {

      gr_complex *in = (gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      //std::cout << "Start Work routine"<< std::endl;

      // Set output items on packet length
      noutput_items = ninput_items[0];
      d_time_packet = (noutput_items*1e9/d_samp_rate);
      d_SendPacket = 1;

      // tags
      std::vector<tag_t> tags;
      const uint64_t nread = nitems_read(0); //number of items read on port 0
      get_tags_in_range(tags, 0, nread, nread+noutput_items);

      std::sort(tags.begin(), tags.end(), tag_t::offset_compare);

      std::vector<tag_t>::iterator vitr = tags.begin();

      //std::cout << "packet offset: " << nread << std::endl;

      while(vitr != tags.end())
      {

        //std::cout << "tags offset: " << (*vitr).offset << " key: " << (*vitr).key << " value: " << (*vitr).value << std::endl;
        if((pmt::eqv((*vitr).key, DeadtimeKey)))
            d_SendPacket = 0;
        vitr++;
      }

      // Resize output buffer
      if(d_out_buffer.size()!=noutput_items) d_out_buffer.resize(noutput_items);
      if(d_in_buffer.size()!=noutput_items) d_in_buffer.resize(noutput_items);



      if (firstPacket)
      {
        Chirp_Sync = d_soapysdr->readGPIO(d_GPIOBanks[0]) && (1<<6);
        std::cout << " readGPIO: " << Chirp_Sync << std::endl;
        firstPacket = 0;
      }


      //Get New Packet Time
      d_time_now_rx = d_soapysdr->getHardwareTime();
      d_time_now_tx = d_time_now_rx;

      if (d_SendPacket)// Tx and Rx Packet
      {



        // ************** Send thread *********************
        d_in_buffer = input_items;
        d_noutput_items_send = noutput_items;
        d_thread_send = gr::thread::thread(boost::bind(&soapysdr_echotimer_impl::send, this));

        // ************** Receive thread *********************
        //gr_vector_void_star d_out_buffer (initialize with output_items)
        d_out_buffer = output_items;
        d_noutput_items_recv = noutput_items;
        d_thread_recv = gr::thread::thread(boost::bind(&soapysdr_echotimer_impl::receive, this));

        //d_value = d_value ^ 0x80; d_soapysdr->writeGPIO(d_GPIOBanks[0],d_value);
        // Wait for threads to complete
        d_thread_send.join();
        d_thread_recv.join();

        d_out_recv = (gr_complex *) d_out_buffer[0];

        memcpy(out,&d_out_recv[0]+d_num_delay_samps,(noutput_items-d_num_delay_samps)*sizeof(gr_complex)); // push buffer to output
        memset(out+(noutput_items-d_num_delay_samps),0,d_num_delay_samps*sizeof(gr_complex)); // set zeros

        //memset(out,0,noutput_items*sizeof(gr_complex));
        firstPacket_Rx = 0;
      }
      else//Rx only Packet
      {
        if(firstPacket_Rx == Rx_Skip_Packets)//skip return packets: set by Rx_Skip_Packets
        {
          // ************** Receive thread *********************
          //gr_vector_void_star d_out_buffer (initialize with output_items)
          d_out_buffer = output_items;
          d_noutput_items_recv = noutput_items;
          d_thread_recv = gr::thread::thread(boost::bind(&soapysdr_echotimer_impl::receive, this));


          d_thread_recv.join();
          d_out_recv = (gr_complex *) d_out_buffer[0];


          memcpy(out,&d_out_recv[0]+d_num_delay_samps,(noutput_items-d_num_delay_samps)*sizeof(gr_complex)); // push buffer to output
          memset(out+(noutput_items-d_num_delay_samps),0,d_num_delay_samps*sizeof(gr_complex)); // set zeros

          firstPacket_Rx = 0;
        }
        else
        {


          memset(out,0,noutput_items*sizeof(gr_complex));
          firstPacket_Rx++;
        }

      }

      // Setup rx_time tag
      d_time_val = pmt::make_tuple
      (pmt::from_uint64(d_time_now_rx/1E9));
      // Save timestamp
      add_item_tag(0, nitems_written(0), d_time_key, d_time_val, d_srcid);



      //d_value = 0; d_soapysdr->writeGPIO(d_GPIOBanks[0],d_value);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }
  } /* namespace radar */
} /* namespace gr */
