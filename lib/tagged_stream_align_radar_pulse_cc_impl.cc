/* -*- c++ -*- */
/*
 * Copyright 2018 Erik Moore.
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
#include "tagged_stream_align_radar_pulse_cc_impl.h"

namespace gr {
  namespace radar {

    tagged_stream_align_radar_pulse_cc::sptr
    tagged_stream_align_radar_pulse_cc::make(int samp_rate, const std::string& len_key, int chirp_len)
    {
      return gnuradio::get_initial_sptr
        (new tagged_stream_align_radar_pulse_cc_impl(samp_rate, len_key, chirp_len));
    }

    /*
     * The private constructor
     */
    tagged_stream_align_radar_pulse_cc_impl::tagged_stream_align_radar_pulse_cc_impl(int samp_rate, const std::string& len_key, int chirp_len)
      : gr::block("tagged_stream_align_radar_pulse_cc",
              gr::io_signature::make(1, 1,  sizeof(gr_complex)),
              gr::io_signature::make(1, 1,  sizeof(gr_complex))),
              d_chirp_len(chirp_len),
              d_samp_rate(samp_rate),
              d_key_len(pmt::string_to_symbol(len_key)),
              d_value_len(pmt::from_long(d_chirp_len))
    {
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    tagged_stream_align_radar_pulse_cc_impl::~tagged_stream_align_radar_pulse_cc_impl()
    {
    }

    void
    tagged_stream_align_radar_pulse_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    tagged_stream_align_radar_pulse_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {

      /*std::vector<tag_t> tags;
      int ncp = std::min(noutput_items, ninput_items[0]);
      get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + noutput_items);
      for(size_t i=0; i<tags.size(); i++){
          gr::tag_t t = tags[i];
          int offset = (nitems_read(0) - nitems_written(0));
          t.offset -= offset;
          add_item_tag(0,t);
          }
      memcpy(output_items[0], input_items[0], ncp*d_itemsize);
      consume_each(ncp);
      return ncp;*/


        // tags
        std::vector<tag_t> tags;
        int num_chirps;

        const uint64_t nread = nitems_read(0); //number of items read on port 0
        get_tags_in_range(tags, 0, nread, nread+noutput_items);

        std::sort(tags.begin(), tags.end(), tag_t::offset_compare);

        std::vector<tag_t>::iterator vitr = tags.begin();

        //std::cout << "packet offset: " << nread << std::endl;

        while(vitr != tags.end())
        {

          std::cout << "tags offset: " << (*vitr).offset << " key: " << (*vitr).key << " value: " << (*vitr).value << std::endl;
          if((pmt::eqv((*vitr).key, d_key_len)))
          {
            num_chirps++;
          }

          vitr++;
        }


      /*  get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + ninput_items[0], d_lengthtag);
        if(tags.size() > 0){
            d_have_sync = true;
            consume_each( tags[0].offset - nitems_read(0) );
        } else {
            consume_each(ninput_items[0]);
        }
        return 0;


      const <+ITYPE+> *in = (const <+ITYPE+> *) input_items[0];
      <+OTYPE+> *out = (<+OTYPE+> *) output_items[0];

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;*/
    }

  } /* namespace radar */
} /* namespace gr */
