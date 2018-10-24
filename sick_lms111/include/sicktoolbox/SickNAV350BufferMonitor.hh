/*!
 * \file SickNav350BufferMonitor.hh
 * \brief Defines a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LMS LIDAR.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_NAV350_BUFFER_MONITOR_HH
#define SICK_NAV350_BUFFER_MONITOR_HH

#define DEFAULT_SICK_BYTE_TIMEOUT         (35000)  ///< Max allowable time between consecutive bytes

/* Definition dependencies */
#include "SickNAV350Message.hh"
#include "SickBufferMonitor.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief A class for monitoring the receive buffer when interfacing with a Sick LD LIDAR
   */
  class SickNav350BufferMonitor : public SickBufferMonitor< SickNav350BufferMonitor, SickNav350Message > {

  public:

    /** A standard constructor */
    SickNav350BufferMonitor( );

    /** A method for extracting a single message from the stream */
    void GetNextMessageFromDataStream( SickNav350Message &sick_message ) throw( SickIOException );

    /** A standard destructor */
    ~SickNav350BufferMonitor( );

  };
    
} /* namespace SickToolbox */

#endif /* SICK_NAV350_BUFFER_MONITOR_HH */
