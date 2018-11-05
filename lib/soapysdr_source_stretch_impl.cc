/* -*- c++ -*- */
/*
 * Copyright 2018 DU2SRI.
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
#include "soapysdr_source_stretch_impl.h"

namespace gr {
  namespace radar {

    soapysdr_source_stretch::sptr
    soapysdr_source_stretch::make(int samp_rate, float center_freq, int num_delay_samps,
                          std::string args,
                          std::string antenna_tx, float gain_tx, float bw_tx,
                          float timeout_tx, float wait_tx, float lo_offset_tx,
                          std::string antenna_rx, float gain_rx, float bw_rx,
                          float timeout_rx, float wait_rx, float lo_offset_rx,
                          const std::string& len_key,int packet_len)
    {
      return gnuradio::get_initial_sptr
        (new soapysdr_source_stretch_impl(samp_rate, center_freq, num_delay_samps,
                                      args,
                                      antenna_tx, gain_tx, bw_tx,
                                      timeout_tx, wait_tx, lo_offset_tx,
                                      antenna_rx, gain_rx, bw_rx,
                                      timeout_rx, wait_rx, lo_offset_rx,
                                      len_key,packet_len));
    }

    /*
     * The private constructor
     */
    soapysdr_source_stretch_impl::soapysdr_source_stretch_impl(int samp_rate, float center_freq,
                                                int num_delay_samps, std::string args,
                                                std::string antenna_tx, float gain_tx, float bw_tx,
                                                float timeout_tx, float wait_tx, float lo_offset_tx,
                                                std::string antenna_rx, float gain_rx, float bw_rx,
                                                float timeout_rx, float wait_rx, float lo_offset_rx,
                                                const std::string& len_key,int packet_len)
      : gr::sync_block("soapysdr_source_stretch",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),

      //Parameters
        //EchoParameters
        d_num_delay_samps(num_delay_samps),
        d_packet_len(packet_len),
        d_key_len(pmt::string_to_symbol(len_key)),
        d_value_len(pmt::from_long(d_packet_len)),
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
      d_out_buffer.resize(d_packet_len);
      d_packet_buffer.resize(d_packet_len*sizeof(gr_complex));
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


      //***** Misc *****//

      // Setup rx_time pmt
      d_time_key = pmt::string_to_symbol("rx_time");
      d_srcid = pmt::string_to_symbol("soapysdr_echotimer");

      std::cout << "Actual Master Clock Rate: [MHz]" << d_soapysdr->getMasterClockRate()/1e6 << std::endl;



      //Aux variables start default
      d_workNum = 0;
      d_packetNum = 0;
      errorNum = 0;
      firstPacket = 0;
      debug = 0;

      chirp_lag = 0;
      prev_chirp_lag=0;

      chirp_sample_period = 0;
      prev_chirp_sample_period = 0;

      chirp_sample_time = 0;
      prev_chirp_sample_time = 0;

      d_sampleTime_rx = 0;
      prev_d_sampleTime_rx = 0;

      d_sampleTime_rx_end = 0;
      prev_d_sampleTime_rx_end = 0;

      d_sampleTime_rx_return = 0;
      prev_d_sampleTime_rx_return = 0;

      prev_written = 0;

      //d_flagsRx = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
      //d_flagsRx = SOAPY_SDR_HAS_TIME;
      d_soapysdr->activateStream(d_rx_stream,0,0,d_packet_len);

      // Sleep to get sync done
      boost::this_thread::sleep(boost::posix_time::milliseconds(2000)); // FIXME: necessary?


      set_rx_gain(d_chan_rx, d_gain_rx);
      set_tx_gain(d_chan_tx, d_gain_tx);

      // Setup Soapysdr RX: timestamp
      //setHardwareTime(const long long timeNs, const std::string &what = "");
      d_soapysdr->setHardwareTime(0.0); // Do set time on startup if not gpsdo is activated.
    }

    /*
     * Our virtual destructor.
     */
    soapysdr_source_stretch_impl::~soapysdr_source_stretch_impl()
    {
      set_tx_gain(d_chan_tx, 0);
      set_rx_gain(d_chan_rx, 0);
      d_soapysdr->writeRegister(d_listRegInterfaces[1], 6<<5, 0xFFFF);
      d_soapysdr->deactivateStream(d_rx_stream);
      d_soapysdr->deactivateStream(d_tx_stream);
      d_soapysdr->closeStream(d_rx_stream);
      d_soapysdr->closeStream(d_tx_stream);
    }

    void
    soapysdr_source_stretch_impl::set_rx_gain( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_RX, chan, gain);
      std::cout << "RX Gain: "  << d_soapysdr->getGain(SOAPY_SDR_RX, chan) << std::endl;
    }

    void
    soapysdr_source_stretch_impl::set_tx_gain( size_t chan, float gain)
    {
      //setGain(const int direction, const size_t channel, const double value);
      d_soapysdr->setGain(SOAPY_SDR_TX, chan, gain);
      std::cout << "TX Gain: "  << d_soapysdr->getGain(SOAPY_SDR_TX, chan) << std::endl;
    }


    int
    soapysdr_source_stretch_impl::send()
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
    soapysdr_source_stretch_impl::receive()
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

      prev_d_sampleTime_rx = d_sampleTime_rx; d_sampleTime_rx = d_timeNs_rx*d_samp_rate/1e9;
      prev_d_sampleTime_rx_return = d_sampleTime_rx_return; d_sampleTime_rx_return = d_timeNs_rx_return*d_samp_rate/1e9;

      if(d_timeNs_rx < d_timeNs_rx_return)
      {
        //std::cout << FMAG(BOLD("-> d_packetNum: ")) << d_packetNum <<std::endl;
        //std::cout << FRED(" d_sampleTime_rx:        ") << d_sampleTime_rx << std::endl;
        //std::cout << FRED(" d_sampleTime_rx_return: ") << d_sampleTime_rx_return << std::endl;
        //std::cout << FRED(" written:                ") <<  nitems_written(0) << std::endl;
        //std::cout << FRED(" difference:         ") << d_timeNs_rx_return-d_timeNs_rx << std::endl;
        //std::cout << FRED(" difference:         ") << (float)(d_timeNs_rx_return-d_timeNs_rx)*(float)d_samp_rate/1e9/d_packet_len << std::endl;
      }
      if (d_num_rx_samps < total_num_samps/100*95)
      {
      //std::cout << showpoint;
      std::cout << FRED(" Dropped 95 percent Rx Samples: ") << d_num_rx_samps << std::endl << std::endl;
      }
      else if(d_num_rx_samps < 0)
      {
        //std::cout << FRED(" Rx Error: ") << d_num_rx_samps;
        errorNum ++;
        //std::cout << FRED(" Rx Error Average: ")<< (float) errorNum / (float) d_packetNum <<std::endl<<std::endl;
      }



      return 0;
    }

    int
    soapysdr_source_stretch_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      gr_complex *out = (gr_complex *) output_items[0];



      // Set output items on packet length
      d_time_packet = (d_packet_len*1e9/d_samp_rate);


      // Resize output buffer
      if(d_packet_buffer.size()!=d_packet_len) d_packet_buffer.resize(sizeof(gr_complex)*d_packet_len);
      if(d_out_buffer.size()!=d_packet_len) d_out_buffer.resize(d_packet_len);

      //copy leftover read packet data from previous work function
      //memset(out,0,noutput_items*sizeof(gr_complex));
      memcpy(out,&d_out_recv[0]+d_noutput_items_recv-d_out_leftover_num,d_out_leftover_num*sizeof(gr_complex));

      //Find last Chirp data
      chirp_timeStamp = std::stoll (d_soapysdr->readSensor("chirp_time"));
      chirp_timePeriod = std::stoll (d_soapysdr->readSensor("chirp_period"));

      prev_chirp_sample_time = chirp_sample_time; chirp_sample_time = (chirp_timeStamp)*d_samp_rate/1e9;
      prev_chirp_sample_period = chirp_sample_period; chirp_sample_period = (chirp_timePeriod)*d_samp_rate/1e9;

      //d_sampleTime_rx_end = d_sampleTime_rx + d_noutput_items_recv;
      prev_chirp_lag = chirp_lag; chirp_lag = d_sampleTime_rx_return - chirp_sample_time;

      d_time_now_rx = d_soapysdr->getHardwareTime();

      //first packet send zeros
      if(firstPacket)
      {
        firstPacket --;
        //d_soapysdr->setHardwareTime(0.0);
        memset(out,0,noutput_items*sizeof(gr_complex));
        chirp_len_value = pmt::from_long((long) chirp_timePeriod);
        //add_item_tag(0, nitems_written(0), d_key_len, d_value_len, d_srcid);
        add_item_tag(0, nitems_written(0), chirp_len_key, chirp_len_value, d_srcid);
        return noutput_items;
      }


      //-------------------------debug--------------------
      if(debug && (d_workNum % 100 == 0 || d_workNum % 100 == 1))
      {
        std::cout << FGRN(BOLD("-> workNum: ")) << d_workNum << FGRN(" noutput_items: ") << noutput_items << std::endl;
        //debug
        for(int i=0;i<output_items.size();i++)
        {
          //std::cout << " output_item: " << i << " -> "<< output_items[i] << std::endl;
        }

        //std::cout << " out: " << out << std::endl;

        //std::cout << " size of d_packet_buffer: " << d_packet_buffer.size() << std::endl;
        //std::cout << " addr of d_packet_buffer: " << d_out_buffer[0]        << std::endl;
        std::cout << FCYN(" d_packet_len            : ") << d_packet_len           << std::endl;
        //std::cout << " chirp_timeStamp        : " << chirp_timeStamp        << std::endl;
        //std::cout << " chirp_timePeriod       : " << chirp_timePeriod       << std::endl;
        std::cout << FCYN(" chirp_sample_period     : ") << chirp_sample_period<<   FBLU(" diff: ") << chirp_sample_period - prev_chirp_sample_period <<std::endl;
        std::cout << FCYN(" chirp_sample_time       : ") << chirp_sample_time  <<   FBLU(" diff: ") << chirp_sample_time - prev_chirp_sample_time << std::endl;
        std::cout << FCYN(" d_sampleTime_rx         : ") << d_sampleTime_rx    <<   FBLU(" diff: ") << d_sampleTime_rx - prev_d_sampleTime_rx << std::endl;
        std::cout << FCYN(" d_sampleTime_rx_return  : ") << d_sampleTime_rx_return<<FBLU(" diff: ") << d_sampleTime_rx_return - prev_d_sampleTime_rx_return << std::endl;
        std::cout << FCYN(" written                 : ") <<  nitems_written(0)    <<FBLU(" diff: ") << nitems_written(0) - prev_written << std::endl;
        std::cout << FCYN(" chirp_lag               : ") << chirp_lag          <<   FBLU(" diff: ") << chirp_lag - prev_chirp_lag << std::endl;
      }
      //------------------------debug---------------------


      //--------------BUFFER LOOP-------------------------
      for (int sampi=0; sampi<noutput_items; sampi++)
      {
        if((nitems_written(0)+sampi) % d_packet_len == 0)
        {
          add_item_tag(0, nitems_written(0)+sampi, d_key_len, d_value_len, d_srcid);



          //Make sure read packet fits into the current work function buffer
          if(d_packet_len > noutput_items-sampi)
          {
            d_out_leftover_num = d_packet_len - (noutput_items - sampi);
          }
          else
          {
            d_out_leftover_num = 0;
          }


          // ************** Receive thread *********************
          //gr_vector_void_star d_out_buffer (initialize with output_items)

            //Set up buffer pointers
          d_out_buffer[0] = &d_packet_buffer[0];

            //read packet length
          d_noutput_items_recv = d_packet_len;

            //read thread
          d_thread_recv = gr::thread::thread(boost::bind(&soapysdr_source_stretch_impl::receive, this));


            //End receive thread
          d_thread_recv.join();
          // ************** End Receive thread ******************

          // Setup rx_time tag
          //d_time_val = pmt::make_tuple(pmt::from_uint64(d_time_now_rx));
          d_time_val = pmt::from_long((long) d_time_now_rx);

          //Get New Packet Time
          d_time_now_rx = d_soapysdr->getHardwareTime();

          //pointer cast
          d_out_recv = (gr_complex *) d_out_buffer[0];

          //Chirp Stuff

          //chirp_timeStamp = std::stoll (d_soapysdr->readSensor("chirp_time"));
          //chirp_timePeriod = std::stoll (d_soapysdr->readSensor("chirp_period"));
          //prev_chirp_sample_time = chirp_sample_time; chirp_sample_time = (chirp_timeStamp)*d_samp_rate/1e9;
          //prev_chirp_sample_period = chirp_sample_period; chirp_sample_period = (chirp_timePeriod)*d_samp_rate/1e9;

          //d_sampleTime_rx_end = d_sampleTime_rx + d_noutput_items_recv;
          prev_chirp_sample_time = chirp_sample_time;

          prev_chirp_lag = chirp_lag; chirp_lag = d_sampleTime_rx_return - chirp_sample_time;




          //copy read packet to proper gnuradio buffer position
          memcpy(out+sampi,&d_out_recv[0],(d_noutput_items_recv-d_out_leftover_num)*sizeof(gr_complex));

          //d_time_now_rx = d_timeNs_rx + d_time_packet;


          if(debug && (d_workNum % 100 == 0 || d_workNum % 100 == 1))
          {
            std::cout << FYEL(BOLD(" packet in work number: ")) << sampi << FYEL(" d_noutput_items_recv: ") << d_noutput_items_recv << std::endl;
            //debug
            std::cout << FCYN("  d_out_leftover_num     : ") << d_out_leftover_num << std::endl;
            std::cout << FCYN("  chirp_sample_time      : ") << chirp_sample_time  <<   FBLU(" diff: ") << chirp_sample_time - prev_chirp_sample_time << std::endl;
            std::cout << FCYN("  d_sampleTime_rx        : ") << d_sampleTime_rx    <<   FBLU(" diff: ") << d_sampleTime_rx - prev_d_sampleTime_rx << std::endl;
            std::cout << FCYN("  d_sampleTime_rx_return : ") << d_sampleTime_rx_return<<FBLU(" diff: ") << d_sampleTime_rx_return - prev_d_sampleTime_rx_return << std::endl;
            std::cout << FCYN("  written                : ") <<  nitems_written(0)    <<FBLU(" diff: ") << nitems_written(0) - prev_written << std::endl;
            std::cout << FCYN("  chirp_lag              : ") << chirp_lag          <<   FBLU(" diff: ") << chirp_lag - prev_chirp_lag << std::endl;
          }

          chirp_lag_write = chirp_lag;
          //-(noutput_items-sampi) < chirp_lag_write < sampi

          // Setup chirp lag
          //chirp_len_value = pmt::make_tuple(pmt::from_uint64(chirp_timePeriod));
          chirp_len_value = pmt::from_long((long) chirp_timePeriod);


          do {
            if(sampi-noutput_items < chirp_lag_write < sampi)
              {
                add_item_tag(0, nitems_written(0) + sampi -  chirp_lag_write, chirp_len_key, chirp_len_value, d_srcid);


                // Save timestamp
                add_item_tag(0, nitems_written(0)+sampi, d_time_key, d_time_val, d_srcid);
              }


            chirp_lag_write = chirp_lag_write - chirp_sample_period;

          } while((sampi-noutput_items) < chirp_lag_write);

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





          if (d_packetNum > 50000)
          {
            d_packetNum = 0;
            errorNum = 0;
          }
          d_packetNum++;

        }

      }

      //std::cout << std::endl;
      memcpy(out,&out[0]+d_num_delay_samps,(noutput_items-d_num_delay_samps)*sizeof(gr_complex)); // push buffer to output
      memset(out+(noutput_items-d_num_delay_samps),0,d_num_delay_samps*sizeof(gr_complex)); // set zeros



      //---------------END TX/RX threads---------------------



      prev_written = nitems_written(0);

      d_workNum++;
      //std::cout << "End work packet stretch: " <<noutput_items << std::endl << std::endl;

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace radar */
} /* namespace gr */
