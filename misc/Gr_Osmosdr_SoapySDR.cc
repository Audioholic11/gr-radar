//from source_impl.h
#include <osmosdr/source.h>
#ifdef HAVE_IQBALANCE
#include <gnuradio/iqbalance/optimize_c.h>
#include <gnuradio/iqbalance/fix_cc.h>
#endif
#include <source_iface.h>
//sourc_iface.h
  #include <osmosdr/ranges.h>
  #include <osmosdr/time_spec.h>
  #include <gnuradio/basic_block.h>
#include <map>

//from sourc_impl.cc
#include <gnuradio/io_signature.h>
#include <gnuradio/blocks/null_source.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/constants.h>
#include <soapy_source_c.h>

//from soapy_source_c.h
#include <gnuradio/block.h>
#include <gnuradio/sync_block.h>
#include "osmosdr/ranges.h"
#include "source_iface.h"

//from soapy_source_c.cc
#include <iostream>
#include <algorithm> //find

#include <boost/assign.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <gnuradio/io_signature.h>

#include "arg_helpers.h"
#include "soapy_source_c.h"
#include "soapy_common.h"
#include "osmosdr/source.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Version.hpp>

//from soapy_common.h
#include <osmosdr/ranges.h>
#include <SoapySDR/Types.hpp>
#include <boost/thread/mutex.hpp>

//from soapy_common.cc
#include <SoapySDR/Version.hpp>




//    std::cerr << std::endl;
//    BOOST_FOREACH( std::string dev, dev_list )
//      std::cerr << "'" << dev << "'" << std::endl;

    if ( dev_list.size() )
      arg_list.push_back( dev_list.front() );
    else
      throw std::runtime_error("No supported devices found to pick from.");
  }

  BOOST_FOREACH(std::string arg, arg_list) {

    dict_t dict = params_to_dict(arg);

//    std::cerr << std::endl;
//    BOOST_FOREACH( dict_t::value_type &entry, dict )
//      std::cerr << "'" << entry.first << "' = '" << entry.second << "'" << std::endl;

    source_iface *iface = NULL;
    gr::basic_block_sptr block;

BOOST_FOREACH( std::string dev, soapy_source_c::get_devices() )
dev_list.push_back( dev );

if ( dict.count("soapy") ) {
      soapy_source_c_sptr src = make_soapy_source_c( arg );
      block = src; iface = src.get();
    }
