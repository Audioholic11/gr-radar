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

#include <iostream>

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
      ( new soapysdr_echotimer_stretch_impl (samp_rate, center_freq, num_delay_samps,
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

      //process_aff_set.push_back(0);
      //process_aff_set.push_back(1);
      //set_processor_affinity(process_aff_set);

      //set_max_noutput_items(d_packet_len);
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


      //Chirp setting
      d_register = 0x001D;
      d_value = 0x1F;
      //d_soapysdr->writeRegister(d_listRegInterfaces[0],d_register,d_value);

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


      //Sensors
      sensors = d_soapysdr->listSensors();

      for(int i=0; i<sensors.size();i++)
      {
        std::cout << "Sensors Banks: " << i << " " << sensors[i] << std::endl;
      }

      // Setup Soapy RX: sample rate
      std::cout << "Setting RX Rate: " << d_samp_rate << std::endl;
      //setSampleRate(const int direction, const size_t channel, const double rate);
      d_soapysdr->setSampleRate(SOAPY_SDR_RX, d_chan_rx, d_samp_rate);
      //getSampleRate(const int direction, const size_t channel);
      std::cout << "Actual RX Rate: " << d_soapysdr->getSampleRate(SOAPY_SDR_RX, d_chan_rx) << std::endl;
      std::cout << "Actual Master Clock Rate: " << d_soapysdr->getMasterClockRate() << std::endl;

      // Setup SoapySDR RX: gain
      set_rx_gain(d_chan_rx, 40);

      // Setup Soapysdr RX: set frequency (tune?)
      //setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const Kwargs &args = Kwargs());
      d_soapysdr->setFrequency(SOAPY_SDR_RX, d_chan_rx, d_center_freq);
      std::cout << "Set RX Frequency: "  << d_soapysdr->getFrequency(SOAPY_SDR_RX, d_chan_rx) << std::endl;

      // Setup Soapysdr RX: antenna
      //setAntenna(const int direction, const size_t channel, const std::string &name);
      d_soapysdr->setAntenna(SOAPY_SDR_RX, d_chan_rx, d_antenna_rx);
      std::cout << "Set RX Antenna: "  << d_soapysdr->getAntenna(SOAPY_SDR_RX, d_chan_rx) << std::endl;

      //void setBandwidth(const int direction, const size_t channel, const double bw);
      //d_soapysdr->setBandwidth(SOAPY_SDR_RX, d_chan_rx, d_bw_rx);
      std::cout << "Set RX BW: "  << d_soapysdr->getBandwidth(SOAPY_SDR_RX, d_chan_rx) << std::endl;

      //d_soapysdr->setHardwareTime(0.0);
      // Setup receive streamer
      d_rx_stream = d_soapysdr->setupStream(SOAPY_SDR_RX, "CF32");//might need channels



      //******************* Setup Soapy TX *******************//
      d_chan_tx = d_chan_rx;


      // Setup Soapysdr TX: sample rate
      std::cout << "Setting TX Rate: " << d_samp_rate << std::endl;

      //setSampleRate(const int direction, const size_t channel, const double rate);
      d_soapysdr->setSampleRate(SOAPY_SDR_TX, d_chan_tx, d_samp_rate);

      //getSampleRate(const int direction, const size_t channel)
      std::cout << "Actual TX Rate: " << d_soapysdr->getSampleRate(SOAPY_SDR_TX, d_chan_tx) << std::endl;


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

      // Setup Soapysdr TX: gain
      set_tx_gain(d_chan_tx, 50);

      // Setup Soapysdr TX: timestamp
      //setHardwareTime(const long long timeNs, const std::string &what = "");
      //d_soapysdr->setHardwareTime(0.0); // Do set time on startup if not gpsdo is activated.

      // Setup transmit streamer
      //Stream *setupStream( const int direction, const std::string &format, const std::vector<size_t> &channels = std::vector<size_t>(), const Kwargs &args = Kwargs());
      d_tx_stream = d_soapysdr->setupStream(SOAPY_SDR_TX, "CF32");//might need channels


      //***** Misc *****//

      // Setup rx_time pmt
      d_time_key = pmt::string_to_symbol("rx_time");
      d_srcid = pmt::string_to_symbol("soapysdr_echotimer");

      std::cout << "Actual Master Clock Rate: [MHz]" << d_soapysdr->getMasterClockRate()/1e6 << std::endl;



      //Aux variables start default
      d_extra_work_time = 0;
      d_workNum = 0;
      errorNum = 0;
      firstPacket_Rx = 0;
      firstPacket = 10;
      d_flagsRx_return = 0;
      chirp_lag = 0;


      //d_flagsTx = SOAPY_SDR_HAS_TIME;
      //d_flagsTx = 0;
      //d_soapysdr->activateStream(d_tx_stream,0,0,1);
      d_soapysdr->activateStream(d_tx_stream, 0 , 0, 0);

      //d_flagsRx = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
      //d_flagsRx = SOAPY_SDR_HAS_TIME;
      d_soapysdr->activateStream(d_rx_stream,0,0,1);

      // Sleep to get sync done
      boost::this_thread::sleep(boost::posix_time::milliseconds(2000)); // FIXME: necessary?


      set_rx_gain(d_chan_rx, d_gain_rx);
      set_tx_gain(d_chan_tx, d_gain_tx);
    }

    /*
     * Our virtual destructor.
     */
    soapysdr_echotimer_stretch_impl::~soapysdr_echotimer_stretch_impl()
    {

      set_tx_gain(d_chan_tx, 0);
      set_rx_gain(d_chan_rx, 0);
      d_soapysdr->writeRegister(d_listRegInterfaces[1], 6<<5, 0xFFFF);
      d_soapysdr->deactivateStream(d_rx_stream);
      d_soapysdr->deactivateStream(d_tx_stream);
      d_soapysdr->closeStream(d_rx_stream);
      d_soapysdr->closeStream(d_tx_stream);
    }

    int
    soapysdr_echotimer_stretch_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = ninput_items[0];
      return noutput_items ;
    }

    void
    soapysdr_echotimer_stretch_impl::set_rx_gain( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_RX, chan, gain);
      std::cout << "RX Gain: "  << d_soapysdr->getGain(SOAPY_SDR_RX, chan) << std::endl;
    }

    void
    soapysdr_echotimer_stretch_impl::set_tx_gain( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_TX, chan, gain);
      std::cout << "TX Gain: "  << d_soapysdr->getGain(SOAPY_SDR_TX, chan) << std::endl;
    }

    int
    soapysdr_echotimer_stretch_impl::send()
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
      //d_flagsTx = 0;
      //d_soapysdr->activateStream(d_tx_stream,d_flagsTx);
      d_flagsTx =  SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
      //d_soapysdr->activateStream(d_tx_stream, 0 , d_timeNs_tx, total_num_samps);
      d_num_tx_samps = d_soapysdr->writeStream(d_tx_stream, &d_in_buffer[0], total_num_samps, d_flagsTx, d_timeNs_tx, d_timeoutUs);
      if (d_num_tx_samps != total_num_samps)
      {
        std::cout << "Tx Samples: " << d_num_tx_samps << std::endl;
      }
      //d_soapysdr->deactivateStream(d_tx_stream);
      //d_flagsTx = SOAPY_SDR_END_BURST;
      //d_num_tx_samps = d_soapysdr->writeStream(d_tx_stream, &d_in_buffer[0], 0, d_flagsTx, 0, d_timeoutUs);
      return 0;
    }

    int
    soapysdr_echotimer_stretch_impl::receive()
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

      if(d_timeNs_rx < d_timeNs_rx_return)
      {
        //std::cout << FRED(" d_timeNs_rx:        ") << d_timeNs_rx << std::endl;
        //std::cout << FRED(" d_timeNs_rx_return: ") << d_timeNs_rx_return << std::endl;
        //std::cout << FRED(" difference:         ") << d_timeNs_rx_return-d_timeNs_rx << std::endl;
        //std::cout << FRED(" difference:         ") << (float)(d_timeNs_rx_return-d_timeNs_rx)/268000 << std::endl;
      }
      if (d_num_rx_samps < total_num_samps/10*9)
      {
      std::cout << showpoint;
      std::cout << FRED(" Dropped 90percent Rx Samples: ") << d_num_rx_samps << std::endl;
      }
      else if(d_num_rx_samps < 0)
      {
        //std::cout << FRED(" Rx Error: ") << d_num_rx_samps <<std::endl;
        errorNum ++;
        std::cout << FRED(" Rx Error Average: ")<< (float) errorNum / (float) d_workNum <<std::endl<<std::endl;
      }

      d_sampleTime_rx = d_timeNs_rx*d_samp_rate/1e9;
      d_sampleTime_rx_return = d_timeNs_rx_return*d_samp_rate/1e9;

      return 0;
    }

    int
    soapysdr_echotimer_stretch_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      gr_complex *in = (gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      d_SendPacket = 1;
      // Set output items on packet length
      noutput_items = ninput_items[0];
      d_time_packet = (noutput_items*1e9/d_samp_rate);

      // tags
      std::vector<tag_t> tags;
      const uint64_t nread = nitems_read(0); //number of items read on port 0
      get_tags_in_range(tags, 0, nread, nread+noutput_items);

      std::sort(tags.begin(), tags.end(), tag_t::offset_compare);

      std::vector<tag_t>::iterator vitr = tags.begin();


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


      if(firstPacket)
      {
        firstPacket --;
        memset(out,0,noutput_items*sizeof(gr_complex));
        return noutput_items;
      }

      //Get New Packet Time
      d_time_now_rx = d_soapysdr->getHardwareTime();
      d_time_now_tx = d_time_now_rx;


      if (d_SendPacket)// Tx and Rx Packet
      {
        //std::cout << "Tx Send" << std::endl;
        // ************** Send thread *********************
        d_in_buffer = input_items;
        d_noutput_items_send = noutput_items;
        d_thread_send = gr::thread::thread(boost::bind(&soapysdr_echotimer_stretch_impl::send, this));

        // ************** Receive thread *********************
        //gr_vector_void_star d_out_buffer (initialize with output_items)
        d_out_buffer = output_items;
        d_noutput_items_recv = noutput_items;
        d_thread_recv = gr::thread::thread(boost::bind(&soapysdr_echotimer_stretch_impl::receive, this));



        chirp_timeStamp = std::stoll (d_soapysdr->readSensor("chirp_time"));
        chirp_timePeriod = std::stoll (d_soapysdr->readSensor("chirp_period"));


        // Wait for threads to complete
        d_thread_send.join();
        d_thread_recv.join();

        d_out_recv = (gr_complex *) d_out_buffer[0];

        memcpy(out,&d_out_recv[0]+d_num_delay_samps,(noutput_items-d_num_delay_samps)*sizeof(gr_complex)); // push buffer to output
        memset(out+(noutput_items-d_num_delay_samps),0,d_num_delay_samps*sizeof(gr_complex)); // set zeros


        //memset(out,0,noutput_items*sizeof(gr_complex));
        firstPacket_Rx = 0;

      }
      else
      {
        //std::cout << "Rx Send" << std::endl;
        // ************** Receive thread *********************
        //gr_vector_void_star d_out_buffer (initialize with output_items)
        d_out_buffer = output_items;
        d_noutput_items_recv = noutput_items;
        d_thread_recv = gr::thread::thread(boost::bind(&soapysdr_echotimer_stretch_impl::receive, this));


        chirp_timeStamp = std::stoll (d_soapysdr->readSensor("chirp_time"));
        chirp_timePeriod = std::stoll (d_soapysdr->readSensor("chirp_period"));



        d_thread_recv.join();


        d_out_recv = (gr_complex *) d_out_buffer[0];


        memcpy(out,&d_out_recv[0]+d_num_delay_samps,(noutput_items-d_num_delay_samps)*sizeof(gr_complex)); // push buffer to output
        memset(out+(noutput_items-d_num_delay_samps),0,d_num_delay_samps*sizeof(gr_complex)); // set zeros


      }
      //---------------END TX/RX threads---------------------

      // Setup rx_time tag
      d_time_val = pmt::make_tuple(pmt::from_uint64(d_time_now_rx));
      // Save timestamp
      add_item_tag(0, nitems_written(0), d_time_key, d_time_val, d_srcid);


      //chirp_lag = d_time_now_rx + ((long long)(d_wait_rx*1000000000)) - chirp_timeStamp;


      chirp_sample_time = (chirp_timeStamp)*d_samp_rate/1e9;
      chirp_sample_period = (chirp_timePeriod)*d_samp_rate/1e9;
      //d_sampleTime_rx_end = d_sampleTime_rx + d_noutput_items_recv;
      chirp_lag = d_sampleTime_rx_return - chirp_sample_time;


      if(-d_noutput_items_recv < chirp_lag < chirp_sample_period)
      {
        // Setup chirp lag
        chirp_len_value = pmt::make_tuple(pmt::from_uint64(chirp_timePeriod));

        while(chirp_lag  > ((-d_noutput_items_recv) + chirp_sample_period))
        {
          chirp_lag = chirp_lag - chirp_sample_period;

          // Save chirp_lag
          if(chirp_lag < 0)
            add_item_tag(0, nitems_written(0) -  chirp_lag, chirp_len_key, chirp_len_value, d_srcid);
        }

        /*time_sample_difference = d_sampleTime_rx - nitems_written(0);

              std::cout << " chirp sample time:          = " << chirp_sample_time << std::endl;
              std::cout << " sample time:                = " << d_sampleTime_rx << std::endl;
              std::cout << " nitems_read:                = " << nitems_read(0) << std::endl;
              std::cout << " nitem_written:              = " << nitems_written(0) << std::endl;
              std::cout << " difference:                 = " << time_sample_difference << std::endl;
              std::cout << " difference change:          = " << time_sample_difference - prev_time_sample_difference << std::endl;
              std::cout << KGRN " chirp lag:                  = " << chirp_lag << RST << std::endl << std::endl;

        prev_time_sample_difference = time_sample_difference;
        //std::cout << " packet_len: " << d_time_packet << std::endl;
        std::cout << FGRN("-------------------SEND CHIRP---------------------") << nitems_written(0) -  chirp_lag << std::endl;
        std::cout << FGRN(" chirp_sample_period: ") << chirp_sample_period << std::endl;
        std::cout << FGRN(" chirpTimeFrequency: ") << 1/ ((float)chirp_timePeriod/1e9) << std::endl<<std::endl<<std::endl;

        */


      }



      if (d_workNum > 50000)
      {
        d_workNum = 0;
        errorNum = 0;
      }
      d_workNum++;
      //std::cout << "End work packet stretch: " <<noutput_items << std::endl << std::endl;

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace radar */
} /* namespace gr */
