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


#ifndef INCLUDED_RADAR_SOAPYSDR_ECHOTIMER_H
#define INCLUDED_RADAR_SOAPYSDR_ECHOTIMER_H

#include <radar/api.h>
#include <gnuradio/tagged_stream_block.h>


namespace gr {
  namespace radar {

    /*!
     * \brief <+description of block+>
     * \ingroup radar
     *
     */
    class RADAR_API soapysdr_echotimer : virtual public gr::tagged_stream_block
    {
     public:
      typedef boost::shared_ptr<soapysdr_echotimer> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar::soapysdr_echotimer.
       *
       * To avoid accidental use of raw pointers, radar::soapysdr_echotimer's
       * constructor is in a private implementation
       * class. radar::soapysdr_echotimer::make is the public interface for
       * creating new instances.
       */
      static sptr make(int samp_rate, float center_freq, int num_delay_samps,
        std::string args="device args", std::string antenna_tx="BAND2", float gain_tx=0,
        float timeout_tx=.1, float wait_tx=.001, float lo_offset_tx=0,
        std::string antenna_rx="LNAH", float gain_rx=0,
        float timeout_rx=.1, float wait_rx=.001, float lo_offset_rx=0,
        const std::string& len_key="packet_len");

      virtual void set_num_delay_samps(int num_samps) = 0;
  	  virtual void set_rx_gain(size_t chan, float gain) = 0;
      virtual void set_tx_gain(size_t chan, float gain) = 0;
    };

  } // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_SOAPYSDR_ECHOTIMER_H */
