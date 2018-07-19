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

#ifndef INCLUDED_RADAR_GPIO_SOURCE_S_IMPL_H
#define INCLUDED_RADAR_GPIO_SOURCE_S_IMPL_H

#include <radar/gpio_source_s.h>

//soapy
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Time.hpp>//
#include <SoapySDR/Version.hpp>


//boost
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/thread/thread.hpp>

#include <chrono>

namespace gr {
  namespace radar {

    class gpio_source_s_impl : public gpio_source_s
    {
     private:
       std::string d_device_args;
       int d_pins;
       int d_samp_rate;

       SoapySDR::Kwargs d_kw;
       SoapySDR::Device *d_soapysdr;


       std::vector<std::string> d_listRegInterfaces;
       std::vector<std::string> d_GPIOBanks;

       unsigned d_register;
       int16_t  d_value;
       std::vector<int16_t> Chirp_Sync;

       volatile int16_t d_readValue;
       long long d_deltaTime_Total;
       float d_deltaTime_Average = 0;
       float d_deltaTime_Average_Running = 0;

       int d_num_works = 1;
       bool firstPacket;

       std::chrono::steady_clock::time_point t1;
       std::chrono::steady_clock::time_point t2;

       int d_noutput_items;
       gr::thread::thread d_thread_readGPIO;


     public:
      gpio_source_s_impl(std::string device_args, int pins, int sample_rate);
      ~gpio_source_s_impl();

      void readGPIO();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_GPIO_SOURCE_S_IMPL_H */
