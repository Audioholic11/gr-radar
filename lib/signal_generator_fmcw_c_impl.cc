/* -*- c++ -*- */
/*
 * Copyright 2014 Communications Engineering Lab, KIT.
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
#include "signal_generator_fmcw_c_impl.h"

namespace gr {
  namespace radar {

    signal_generator_fmcw_c::sptr
    signal_generator_fmcw_c::make(
            const int samp_rate,
            const int packet_len,
            int samp_up,
            int samp_up_hold,
            int samp_down,
            int samp_down_hold,
            int samp_cw,
            int samp_dead,
            const float freq_cw,
            const float freq_sweep,
            const float amplitude,
            const std::string& len_key,
            const std::string& chirp_len_key,
            const std::string& total_len_key
    ) {
      return gnuradio::get_initial_sptr(new signal_generator_fmcw_c_impl(
          samp_rate,
          packet_len,
          samp_up,
          samp_up_hold,
          samp_down,
          samp_down_hold,
          samp_cw,
          samp_dead,
          freq_cw,
          freq_sweep,
          amplitude,
          len_key,
          chirp_len_key,
          total_len_key
      ));
    }

    signal_generator_fmcw_c_impl::signal_generator_fmcw_c_impl(
            const int samp_rate,
            const int packet_len,
            int samp_up,
            int samp_up_hold,
            int samp_down,
            int samp_down_hold,
            int samp_cw,
            int samp_dead,
            float freq_cw,
            const float freq_sweep,
            const float amplitude,
            const std::string& len_key,
            const std::string& chirp_len_key,
            const std::string& total_len_key)
      : gr::sync_block("signal_generator_fmcw_c",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
      , d_samp_rate(samp_rate)
      , d_samp_up(samp_up)
      , d_samp_up_hold(samp_up_hold)
      , d_samp_down(samp_down)
      , d_samp_down_hold(samp_down_hold)
      , d_samp_cw(samp_cw)
      , d_samp_dead(samp_dead)
      , d_freq_cw(freq_cw)
      , d_freq_sweep(freq_sweep)
      , d_amplitude(amplitude)

      , d_packet_len(packet_len)
      , d_key_len(pmt::string_to_symbol(len_key))
      , d_value_len(pmt::from_long(d_packet_len))

      , d_chirp_len(samp_cw+samp_up+samp_up_hold+samp_down+samp_down_hold)
      , d_total_len(d_chirp_len + samp_dead)

      , d_key_chirp_len(pmt::string_to_symbol(chirp_len_key))
      , d_value_chirp_len(pmt::from_long(d_chirp_len))

      , d_key_total_len(pmt::string_to_symbol(total_len_key))
      , d_value_total_len(pmt::from_long(d_total_len))

      , d_key_deadtime(pmt::string_to_symbol("Deadtime"))
      , d_value_deadtime(pmt::from_long(samp_dead))

      , d_srcid(pmt::string_to_symbol("sig_gen_fmcw"))

      , d_wv_counter(0)
      , d_waveform(d_total_len, freq_cw)
      , d_waveform_amp(d_total_len,d_amplitude)
    {
      set_d_samp_dead(d_samp_dead);//reset dead samples to allign total samples
      //set_max_noutput_items(100*d_packet_len);
      set_min_noutput_items(2*d_packet_len);
      //process_aff_set.push_back(3);
      //set_processor_affinity(process_aff_set);
    }

    signal_generator_fmcw_c_impl::~signal_generator_fmcw_c_impl()
    {
    }

    //Auxiliary method: Set Members (perform calculations if necessary)
    void
    signal_generator_fmcw_c_impl::set_d_chirp_len(int chirp_length)
    {d_chirp_len = chirp_length;}

    void
    signal_generator_fmcw_c_impl::set_d_total_len(int total_length)
    {
      d_total_len = total_length;
      d_value_total_len = pmt::from_long(total_length);
    }

    void
    signal_generator_fmcw_c_impl::set_waveform()
    {

      d_waveform.assign(d_total_len,d_freq_cw);
      d_waveform_amp.assign(d_total_len,d_amplitude);
      // Setup waveform vector: Contains cw, up-chirp (hold), down-chirp (hold).
      // CW is already set above. Note this vector contains the phase increment of
      // the waveform, not the IQ samples itself.
      for(int k=0; k<d_samp_up; k++) {
        d_waveform[k+d_samp_cw] =
          d_freq_cw + (d_freq_sweep/(float)d_samp_up) * k;
      }
      for(int k=0; k<d_samp_up_hold; k++) {
        d_waveform[k+d_samp_cw+d_samp_up] =
          d_freq_cw + d_freq_sweep;
      }
      for(int k=0; k<d_samp_down; k++) {
        d_waveform[k+d_samp_cw+d_samp_up+d_samp_up_hold] =
          d_freq_cw + d_freq_sweep - (d_freq_sweep/(float)d_samp_down) * k;
      }
      for(int k=0; k<d_samp_down_hold; k++) {
      d_waveform[k+d_samp_cw+d_samp_up+d_samp_up_hold+d_samp_down] =
          d_freq_cw;
      }
      for(int k=0; k<d_samp_dead; k++) {
      d_waveform_amp[k+d_samp_cw+d_samp_up+d_samp_up_hold+d_samp_down+d_samp_down_hold] =
          0;
      }

      d_wv_counter = 0;
      d_counter_deadtime = 0;
    }

    void  signal_generator_fmcw_c_impl::set_d_samp_dead(int dead_samples)
    {
      //d_samp_dead = dead_samples;
      set_d_samp_dead_round(dead_samples);
      set_waveform();
    }

    void
    signal_generator_fmcw_c_impl::set_d_samp_dead_round(int dead_samples)
    {
      //round up to nearest packet len: update d_samp_dead and d_total_len
      int packet_modulo = (d_chirp_len + dead_samples) % d_packet_len;
      d_samp_dead = (dead_samples + d_packet_len) - packet_modulo;
      d_total_len = d_chirp_len + d_samp_dead;

      d_value_deadtime = pmt::from_long(d_samp_dead);
      d_value_total_len = pmt::from_long(d_total_len);
    }

    void
    signal_generator_fmcw_c_impl::set_chirp(int up_samples, int up_hold_samples,
                                            int down_samples, int down_hold_samples,
                                            int cw_samples)
    {
      //set members: all chirp related
      d_samp_up = up_samples;
      d_samp_up_hold = up_hold_samples;
      d_samp_down = down_samples;
      d_samp_down_hold = down_hold_samples;
      d_samp_cw = cw_samples;
      d_chirp_len = up_samples+up_hold_samples+down_samples+down_hold_samples+cw_samples;

      d_value_chirp_len = pmt::from_long(d_chirp_len);

      set_d_samp_dead(d_samp_dead);//must update deadsamples
    }

    // Work Function
    int
    signal_generator_fmcw_c_impl::work(
        int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items
    )
    {


      gr_complex *out = (gr_complex *) output_items[0];
      // Integrate phase for iq signal
      for (int i=0; i<noutput_items; i++) {

        if (((nitems_written(0)+i) % d_total_len) == (d_chirp_len)) {
          d_counter_deadtime = 1;
        }

        // Set tag on every packet_len-th item

        // Set tag on every chirp_len-th item
        if ((nitems_written(0)+i) % d_total_len == 0) {
          add_item_tag(0, nitems_written(0)+i, d_key_chirp_len, d_value_chirp_len, d_srcid);
          add_item_tag(0, nitems_written(0)+i, d_key_total_len, d_value_total_len, d_srcid);
          d_counter_deadtime = 0;
          d_wv_counter = 0;
        }


        if ((nitems_written(0)+i) % d_packet_len == 0) {
          add_item_tag(0, nitems_written(0)+i, d_key_len, d_value_len, d_srcid);
          if(d_counter_deadtime)
            add_item_tag(0, nitems_written(0)+i, d_key_deadtime, d_value_deadtime, d_srcid);
          //d_wv_counter = 0;
        }


         //process_aff = processor_affinity();
         //std::cout << FMAG("Affinity size: ") << process_aff.size() <<std::endl;
         //for(int i = 0;i<process_aff.size();i++)
         //{
          // std::cout << FMAG("Affinity: ") << process_aff[i]<< std::endl;
         //}

        // Write sample
        *out++ = d_amp*exp(d_phase);

        //if((nitems_written(0)) % d_samp_up)
        d_phase = 1j*std::fmod(
            imag(d_phase)+2*M_PI*d_waveform[d_wv_counter]/(float)d_samp_rate,
            2*M_PI
        );
        d_amp = d_waveform_amp[d_wv_counter];
        d_wv_counter++;
      }

      //std::cout << "End work packet FMCW Signal: " <<noutput_items << std::endl;

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace radar */
} /* namespace gr */
