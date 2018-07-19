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
#include "gpio_source_s_impl.h"

namespace gr {
  namespace radar {

    gpio_source_s::sptr
    gpio_source_s::make(std::string device_args, int pins, int sample_rate)
    {
      return gnuradio::get_initial_sptr
        (new gpio_source_s_impl(device_args,pins,sample_rate));
    }

    /*
     * The private constructor
     */
    gpio_source_s_impl::gpio_source_s_impl(std::string device_args, int pins, int sample_rate)
      : gr::sync_block("gpio_source_s",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, pins, sizeof(int16_t))),
              //Parameters
              d_device_args(device_args),
              d_pins(pins),
              d_samp_rate(sample_rate)
    {
    //Setup Soapy Device
    d_kw = SoapySDR::KwargsFromString(d_device_args);
    d_soapysdr = SoapySDR::Device::make(d_kw);


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
    d_value = 0xFFFF;// GPIO(0-5) overload bits = 0

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
    d_value = 0x80;//bit 6 input
    d_soapysdr->writeGPIODir(d_GPIOBanks[0],d_value);

    t1 = std::chrono::steady_clock::now();
    t2 = t1;

    d_deltaTime_Average = 0;
    d_deltaTime_Average_Running = 0;
    d_num_works = 1;
    firstPacket = 1;

    //std::cout << "is steady: "<< std::chrono::high_resolution_clock.is_steady() << std::endl;
    }

    /*
     * Our virtual destructor.
     */
    gpio_source_s_impl::~gpio_source_s_impl()
    {
      //Turn off GPIO Overload
      d_soapysdr->writeRegister(d_listRegInterfaces[1], 6<<5, 0xFFFF);
    }

    void gpio_source_s_impl::readGPIO()
    {
      //Check Affinity and Priority
      //std::cout << "Thread Affinity: " << processor_affinity() << std::endl;
      //std::cout << "Thread priority active: " << active_thread_priority() << std::endl;
      //std::cout << "Thread priority: " << thread_priority() << std::endl;


      set_thread_priority(1);//Sets the current thread priority.

      if(firstPacket)
      {
        firstPacket = 0;
        t1 = std::chrono::steady_clock::now();
      }

      t2 = std::chrono::steady_clock::now();
      auto timePeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    //  std::cout << "New Work timePeriod: " << timePeriod << std::endl;

      d_deltaTime_Total = 0;

      for(int i = 0; i < d_noutput_items; i++)
      {

        t2 = std::chrono::steady_clock::now();
        auto timePeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
        //std::cout << "timePeriod: " << timePeriod;

        while(timePeriod < (1e9 / (float) d_samp_rate))
        {
          t2 = std::chrono::steady_clock::now();
          timePeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

        }
        //timePeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

        t1 = std::chrono::steady_clock::now();

        d_readValue = d_soapysdr->readGPIO(d_GPIOBanks[0]);

        d_deltaTime_Total += timePeriod - (1e9/ (float) d_samp_rate);

      //  std::cout << "while loop timeperiod: " << timePeriod  << std::endl;

        //Chirp_Sync[i] = (short) d_readValue;
        Chirp_Sync[i] = 1 && (d_readValue >> 7);
        //std::cout << "GPIO: " << d_readValue << std::endl;
        //boost::this_thread::sleep(boost::posix_time::microseconds(1e6/ (float) d_samp_rate));
      }
      d_deltaTime_Average = (float) d_deltaTime_Total / (float) d_noutput_items;
      //std::cout << "   delta timePeriod Average: " << d_deltaTime_Average << std::endl;
    }

    int
    gpio_source_s_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      Chirp_Sync.assign(noutput_items,0);
      int16_t *out = (int16_t *) output_items[0];



      //std::cout << "   Work items: " << noutput_items << std::endl;
      d_noutput_items = noutput_items;

      d_thread_readGPIO = gr::thread::thread(boost::bind(&gpio_source_s_impl::readGPIO, this));

      d_thread_readGPIO.join();

      d_num_works++;
      d_deltaTime_Average_Running = (d_deltaTime_Average*1/d_num_works)  + (d_deltaTime_Average_Running*(d_num_works-1)/d_num_works);
      //std::cout << "   delta timePeriod Average Running: " << d_deltaTime_Average_Running << std::endl;

      memcpy(out,&Chirp_Sync[0],noutput_items*sizeof(int16_t));
      //memset(out,0,noutput_items*sizeof(short));

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace radar */
} /* namespace gr */
