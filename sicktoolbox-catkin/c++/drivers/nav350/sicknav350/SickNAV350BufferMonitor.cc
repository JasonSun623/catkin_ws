/*!
 * \file SickNav350BufferMonitor.cc
 * \brief Implements a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LD LIDAR.
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

/* Auto-generated header */
#include "sicktoolbox/SickConfig.hh"

/* Implementation dependencies */
#include <iostream>
#include <sys/ioctl.h>

#include "sicktoolbox/SickNAV350BufferMonitor.hh"
#include "sicktoolbox/SickNAV350Message.hh"
#include "sicktoolbox/SickException.hh"
#include "sicktoolbox/SickNAV350Utility.hh"

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A standard constructor
   */
  SickNav350BufferMonitor::SickNav350BufferMonitor( ) : SickBufferMonitor< SickNav350BufferMonitor, SickNav350Message >(this) { }

  /**
   * \brief Acquires the next message from the SickNav350 byte stream
   * \param &sick_message The returned message object
   */
  void SickNav350BufferMonitor::GetNextMessageFromDataStream( SickNav350Message &sick_message ) throw( SickIOException ) {

    /* Flush the input buffer */
    uint8_t byte_buffer;  

    /* A buffer to hold the current byte out of the stream */
    const uint8_t sick_response_header[1] = {0x02};
    const uint8_t sick_response_trailer[1] = {0x03};
    
    uint8_t checksum = 0;  
    uint8_t message_buffer[SICK_NAV350_MSG_PAYLOAD_MAX_LEN /*SickNav350Message::MESSAGE_MAX_LENGTH*/] = {0};
    uint32_t payload_length = 0;
	int8_t succ=0;

    try {

      /* Search for the header in the byte stream */
      for (unsigned int i = 0; i < sizeof(sick_response_header);) {
    	  /* Acquire the next byte from the stream */
    	  _readBytes(&byte_buffer,1,DEFAULT_SICK_BYTE_TIMEOUT);
//    	  std::cout<<byte_buffer<<" test"<<std::endl;
    	  /* Check if the current byte matches the expected header byte */


    	  if (byte_buffer == sick_response_header[i]) {
    		  i++;
    	  }
    	  else {
    		  i = 0;
    	  }
	
      }  
      /* Populate message buffer w/ response header */
      memcpy(message_buffer,sick_response_header,1);


      /* Search for the header in the byte stream */
      for (unsigned int i = 0; i <SICK_NAV350_MSG_PAYLOAD_MAX_LEN+1;i++) {
//		std::cout<<i<<std::endl;
    	  /* Acquire the next byte from the stream */
    	  _readBytes(&byte_buffer,1,DEFAULT_SICK_BYTE_TIMEOUT*10);
 //   	  if (i%100==0)
//    	  std::cout<<byte_buffer<<" "<<i<<" ";
	
    	  /* Check if the current byte matches the expected header byte */
    	  message_buffer[i+1]=byte_buffer;
    	  if (byte_buffer == sick_response_trailer[0]) {
    		  succ=1;
    		  break;
    	  }
    	  payload_length++;

      }
      if (succ==0)
      {
    	  std::cout<<"Incorrect message"<<std::endl;
    	  return;
      }
      
      /* Build the return message object based upon the received payload
       * and compute the associated checksum.
       *
       * NOTE: In constructing this message we ignore the header bytes
       *       buffered since the BuildMessage routine will insert the
       *       correct header automatically and compute the payload's
       *       checksum for us. We could probably get away with using
       *       just ParseMessage here and not computing the checksum as
       *       we are using TCP.  However, its safer this way.
       */
      sick_message.BuildMessage(&message_buffer[SickNav350Message::MESSAGE_HEADER_LENGTH],payload_length);
      
//      std::cout<<"success"<<std::endl;
      /* Success */

    }

    catch(SickTimeoutException &sick_timeout) { /* This is ok! */ }
    
    /* Catch a bad checksum! */
    catch(SickBadChecksumException &sick_checksum_exception) {
      sick_message.Clear(); // Clear the message container
    }
    
    /* Catch any serious IO buffer exceptions */
    catch(SickIOException &sick_io_exception) {
      throw;
    }

    /* A sanity check */
    catch (...) {
      throw;
    }
    
  }
  
  /**
   * \brief A standard destructor
   */
  SickNav350BufferMonitor::~SickNav350BufferMonitor( ) { }
    
} /* namespace SickToolbox */
