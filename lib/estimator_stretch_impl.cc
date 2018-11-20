/* -*- c++ -*- */
/*
 * Copyright 2018 Erik Moore DU2SRI.
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
#include "estimator_stretch_impl.h"

namespace gr {
  namespace radar {

    estimator_stretch::sptr
    estimator_stretch::make(int samp_rate, float center_freq, float bandwidth, int chirp_len, int samp_up, int samp_down, float DC_freq_offset)
    {
      return gnuradio::get_initial_sptr
        (new estimator_stretch_impl(samp_rate, center_freq, bandwidth, chirp_len, samp_up, samp_down, DC_freq_offset));
    }

    /*
     * The private constructor
     */
    estimator_stretch_impl::estimator_stretch_impl(int samp_rate, float center_freq, float bandwidth, int chirp_len, int samp_up, int samp_down,float DC_freq_offset)
      : gr::block("estimator_stretch",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
              d_samp_rate(samp_rate),
              d_center_freq(center_freq),
              d_bandwidth(bandwidth),
              d_chirp_len(chirp_len),
              d_samp_up(samp_up),
              d_samp_down(samp_down),
              d_freq_offset(DC_freq_offset)
    {

      d_msg_stretch_in = false;

  		// Setup estimation constants
  		d_const_stretch = d_bandwidth / d_chirp_len * d_samp_rate;
  		// Register input message ports
  		d_port_id_in_stretch = pmt::mp("Msg in Stretch");
  		message_port_register_in(d_port_id_in_stretch);
  		set_msg_handler(d_port_id_in_stretch, boost::bind(&estimator_stretch_impl::handle_msg_stretch, this, _1));

  		// Register output message port
  		d_port_id_out = pmt::mp("Msg out");
  		message_port_register_out(d_port_id_out);
    }

    void
      estimator_stretch_impl::handle_msg_stretch(pmt::pmt_t msg)
      {
  		// Handle CW msg and call estimate if all msgs are available
  		d_msg_stretch = msg;
  		d_msg_stretch_in = true;
  		if(d_msg_stretch_in){
  			d_msg_stretch_in = false;
  			estimate();
  		}
  	}

    void
      estimator_stretch_impl::estimate()
      {
  		// Get timestamp and frequencies (cw, up-chirp, down-chirp)
  		std::vector<float> freq_stretch;
  		pmt::pmt_t timestamp;
  		pmt::pmt_t msg_part;
  		pmt::pmt_t power;

  		for(int k=0; k<pmt::length(d_msg_stretch); k++){ // search freq stretch
  			msg_part = pmt::nth(k,d_msg_stretch);
  			if(pmt::symbol_to_string(pmt::nth(0,msg_part))=="frequency"){
  				freq_stretch = pmt::f32vector_elements(pmt::nth(1,msg_part));
  			}
  			else if(pmt::symbol_to_string(pmt::nth(0,msg_part))=="rx_time"){
  				timestamp = pmt::nth(1,msg_part);
  			}
  			else if(pmt::symbol_to_string(pmt::nth(0,msg_part))=="power"){
  				power = msg_part;
  			}
  		}

  		/*for(int k=0; k<pmt::length(d_msg_up); k++){ // search freq UP
  			msg_part = pmt::nth(k,d_msg_up);
  			if(pmt::symbol_to_string(pmt::nth(0,msg_part))=="frequency"){
  				freq_up = pmt::f32vector_elements(pmt::nth(1,msg_part));
  			}
  		}*/


  		// Get Range out of Stretch frequencies
  		std::vector<float> range_stretch;
      float range_estimate=0;
      float last_range_estimate=0;
  		for(int k=0; k<freq_stretch.size(); k++){

        range_estimate = (freq_stretch[k]-d_freq_offset)/d_const_stretch/2*c_light;
        if(range_estimate>last_range_estimate)
        {
  			     range_stretch.push_back(range_estimate);
             last_range_estimate = range_estimate;
        }else{
          range_stretch.push_back(last_range_estimate);
        }

  		}

      std::vector<float> velocity, range;

      velocity.push_back(0);
  		range.push_back(range_stretch.back());


  		// Pack output msg and push to output
  		pmt::pmt_t time_pack;
  		time_pack = pmt::list2(pmt::string_to_symbol("rx_time"), timestamp); // make list for timestamp information

  		pmt::pmt_t vel_value, vel_pack;
  		vel_value = pmt::init_f32vector(velocity.size(), velocity); // vector to pmt
  		vel_pack = pmt::list2(pmt::string_to_symbol("velocity"), vel_value); // make list for velocity information

  		pmt::pmt_t range_value, range_pack;
  		range_value = pmt::init_f32vector(range.size(), range); // vector to pmt
  		range_pack = pmt::list2(pmt::string_to_symbol("range"), range_value); // make list for range information

  		pmt::pmt_t value;

  		value = pmt::list3(time_pack, vel_pack, range_pack); // all information to one pmt list

  		message_port_pub(d_port_id_out,value);
  	}

    void estimator_stretch_impl::set_dc_freq_offset(float freq_offset) {
      d_freq_offset = freq_offset;
    }

    /*
     * Our virtual destructor.
     */
    estimator_stretch_impl::~estimator_stretch_impl()
    {
    }



  } /* namespace radar */
} /* namespace gr */
