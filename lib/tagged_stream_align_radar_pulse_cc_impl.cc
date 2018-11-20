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
    tagged_stream_align_radar_pulse_cc::make(int samp_rate, const std::string& len_key, int chirp_len, int debug_print)
    {
      return gnuradio::get_initial_sptr
        (new tagged_stream_align_radar_pulse_cc_impl(samp_rate, len_key, chirp_len,debug_print));
    }

    /*
     * The private constructor
     */
    tagged_stream_align_radar_pulse_cc_impl::tagged_stream_align_radar_pulse_cc_impl(int samp_rate, const std::string& len_key, int chirp_len, int debug_print)
      : gr::block("tagged_stream_align_radar_pulse_cc",
              gr::io_signature::make(1, 1,  sizeof(gr_complex)),
              gr::io_signature::make(1, 1,  sizeof(gr_complex))),
              d_chirp_len(chirp_len),
              d_samp_rate(samp_rate),
              d_key_len(pmt::string_to_symbol(len_key)),
              d_value_len(pmt::from_long(d_chirp_len)),
              d_debug_print(debug_print)
    {
      set_tag_propagation_policy(TPP_DONT);
      d_srcid = pmt::string_to_symbol("tagged_stream_align_radar_pulse_cc");
      d_last_chirp_len_offset=0;
      d_input_tag_shift=0;

      //Chirp Forcast
      d_in_out_chirps_ratio=1;
      d_chirp_len_tag_value = d_chirp_len;
      d_output_chirps = 1;
      d_input_chirps = 1;

      d_overflow=0;

      //debug
      workNum=0;
      forcastNum=0;

      //set_history(d_chirp_len_tag_value);
      set_history(1);
      set_output_multiple(d_chirp_len_tag_value);
    }

    /*
     * Our virtual destructor.
     */
    tagged_stream_align_radar_pulse_cc_impl::~tagged_stream_align_radar_pulse_cc_impl()
    {
    }


    void
    tagged_stream_align_radar_pulse_cc_impl::set_debug_print( int debug_print)
    {
      d_debug_print = debug_print;
    }

    void
    tagged_stream_align_radar_pulse_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      forcastNum++;

      d_output_chirps = (float) noutput_items / (float) d_chirp_len_tag_value;
      //ninput_items_required[0] = ((d_output_chirps/d_in_out_chirps_ratio)) * d_chirp_len_tag_value;

      //if(ninput_items_required[0] >= noutput_items)
      //  {ninput_items_required[0] = noutput_items;}

        ninput_items_required[0] = d_chirp_len_tag_value+history()-1;
       //ninput_items_required[0] = noutput_items+history()-1;

      return;
    }

    int
    tagged_stream_align_radar_pulse_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      workNum++;

      gr_complex *in = (gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      /*std::vector<tag_t> tags;

      get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + noutput_items);
      for(size_t i=0; i<tags.size(); i++){
          gr::tag_t t = tags[i];
          int offset = (nitems_read(0) - nitems_written(0));
          t.offset -= offset;
          add_item_tag(0,t);
        }*/

        //Work - to - work variables
        const uint64_t nread = nitems_read(0); //number of items read on port 0
        const uint64_t nwritten = nitems_written(0);
        int num_chirps = 0;
        d_output_tag_shift = 0;


        if(d_overflow>0)
        {
          int noverflow_write = 0;
          if(d_overflow>noutput_items){
            noverflow_write = noutput_items;
          }else{
            noverflow_write = d_overflow;
          }

          memcpy(out+d_output_tag_shift, in, noverflow_write*sizeof(gr_complex));
          d_overflow -= noverflow_write;
          d_output_tag_shift += noverflow_write;
          *in = *(in+noverflow_write-1);
        }



        if(ninput_items[0] > 0 && (d_debug_print == 1))
        {
          std::cout << std::endl << BOLD(FGRN("Work Number: ")) << workNum << std::endl;

          std::cout << FRED(" Forcast Number: ") << forcastNum << std::endl;
          std::cout << FRED(" max noutput items: ") << max_noutput_items() << std::endl;
          std::cout << FRED(" min noutput items: ") << min_noutput_items() << std::endl;

          std::cout << FRED(" OVERFLOW begin: ") << d_overflow << std::endl;
          std::cout << FRED(" history: ") << history() << std::endl;

          std::cout << FWHT(" ninput_items: ") << ninput_items[0] << std::endl;
          std::cout << FWHT(" noutput_items: ") << noutput_items << std::endl;

          std::cout << FWHT(" number read: ") << nread << std::endl;
          std::cout << FWHT(" number written: ") << nwritten << std::endl;

        }
        //Get and iterate Tags
        std::vector<tag_t> tags;
        std::vector<tag_t> chirp_tags;

        get_tags_in_range(tags, 0, nread, nread+ninput_items[0]);

        std::sort(tags.begin(), tags.end(), tag_t::offset_compare);

        std::vector<tag_t>::iterator vitr = tags.begin();

        while(vitr != tags.end())
        {

          //std::cout << "tags offset: " << (*vitr).offset << " key: " << (*vitr).key << " value: " << (*vitr).value << std::endl;
          if((pmt::eqv((*vitr).key, d_key_len)))//find chirp_len
          {
            //if((*vitr).offset >= (d_last_chirp_len_offset + pmt::to_long((*vitr).value)))
            if((*vitr).offset > d_last_chirp_len_offset)//avoid duplicates
            {
              //std::cout << "tags offset: " << (*vitr).offset << " key: " << (*vitr).key << " value: " << (*vitr).value << std::endl;


              chirp_tags.push_back(*vitr);

              d_chirp_len_tag_value = pmt::to_long((*vitr).value);



              d_input_tag_shift = (*vitr).offset - nread;
              d_last_input_sample_used = d_input_tag_shift + pmt::to_long((*vitr).value);
              d_chirp_len_offset_period = (*vitr).offset - d_last_chirp_len_offset;

              add_item_tag(0, nwritten+d_output_tag_shift, (*vitr).key, (*vitr).value, d_srcid);
              //add_item_tag(0, (*vitr).offset, (*vitr).key, (*vitr).value, d_srcid);

              memcpy(out+d_output_tag_shift, in+d_input_tag_shift, pmt::to_long((*vitr).value)*sizeof(gr_complex));

              if(d_debug_print == 1)
              {
              std::cout << UNDL(FCYN(" tags offset: ")) << chirp_tags[num_chirps].offset << " key: " << chirp_tags[num_chirps].key << " value: " << chirp_tags[num_chirps].value << std::endl;
              std::cout << FYEL(" d_input_tag_shift: ") << d_input_tag_shift << std::endl;
              std::cout << FYEL(" d_output_tag_shift: ") << d_output_tag_shift << std::endl;
              std::cout << FYEL(" d_last_chirp_len_offset: ") << d_last_chirp_len_offset << std::endl;
              std::cout << FYEL(" d_chirp_len_offset_period: ") << d_chirp_len_offset_period << std::endl;
              }

              d_output_tag_shift += pmt::to_long((*vitr).value);
              d_last_chirp_len_offset = (*vitr).offset;
              num_chirps++;

              d_input_chirps = num_chirps;

            }
          }

          vitr++;
        }


        //d_in_out_chirps_ratio = d_input_chirps / (d_output_chirps);

        //std::cout << "noutput_items: " << noutput_items << std::endl;
        //std::cout << "ninput_items: " << ninput_items[0] << std::endl;

        //std::cout << FRED(" num_chirps: ") << num_chirps << std::endl;
        /*int ncp = 0;
        if(num_chirps < 1)
        {
        //int  ncp = std::min(noutput_items, ninput_items[0]);
          ncp = 0;
        }
        else {
          ncp = noutput_items;
        }*/

        int ncp = d_output_tag_shift;

        if(ncp>noutput_items)
        {
          d_overflow = ncp-noutput_items;
          ncp = noutput_items;
        }


        int consume_ncp = 0;
        if(d_last_input_sample_used > ninput_items[0])
        {
          consume_ncp = ninput_items[0];
        }
        else{
          consume_ncp = d_last_input_sample_used;
        }

        if(ncp >0 && (d_debug_print==1))
        {
          std::cout << FBLU(" Number of Chirps Input: ") << d_input_chirps << std::endl;
          std::cout << FBLU(" Number of Chirps Output: ") << d_output_chirps << std::endl;
          //std::cout << FBLU(" In / Out Ratio: ") << d_in_out_chirps_ratio << std::endl;
          std::cout << FBLU(" Consume Number: ") << consume_ncp << std::endl;
          std::cout << FBLU(" Return  Number: ") << ncp << std::endl;

          //std::cout << "ninput_items: " << ninput_items[0] << std::endl;
          //std::cout << "noutput_items: " << noutput_items << std::endl;
          //std::cout << FCYN("Work Number: ")<< workNum << std::endl;
          std::cout << FRED(" OVERFLOW END: ") << d_overflow << std::endl;
        }
        //memcpy(out, in, ncp*sizeof(gr_complex));

        //set_history(d_overflow+1);

        set_output_multiple(d_chirp_len_tag_value+d_overflow);
        set_min_noutput_items(d_chirp_len_tag_value+d_overflow);
        set_alignment(d_chirp_len_tag_value);
        consume_each(consume_ncp);
        return ncp;

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
