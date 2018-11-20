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


#ifndef INCLUDED_RADAR_ESTIMATOR_STRETCH_H
#define INCLUDED_RADAR_ESTIMATOR_STRETCH_H

#include <radar/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace radar {

    /*!
     * \brief <+description of block+>
     * \ingroup radar
     *
     */
    class RADAR_API estimator_stretch : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<estimator_stretch> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of radar::estimator_stretch.
       *
       * To avoid accidental use of raw pointers, radar::estimator_stretch's
       * constructor is in a private implementation
       * class. radar::estimator_stretch::make is the public interface for
       * creating new instances.
       */
      static sptr make(int samp_rate, float center_freq, float bandwidth, int chirp_len, int samp_up, int samp_down ,float DC_freq_offset);

      //Variable Sets and gets
      virtual void set_dc_freq_offset(float freq_offset)=0;
    };

  } // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_ESTIMATOR_STRETCH_H */
