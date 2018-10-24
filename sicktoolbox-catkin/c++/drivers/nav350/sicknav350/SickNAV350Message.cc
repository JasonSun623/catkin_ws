/*!
 * \file SickNav350Message.cc
 * \brief Implements the class SickNav350Message.
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
#include <iomanip>
#include <iostream>
#include <arpa/inet.h> 

#include "sicktoolbox/SickNAV350Message.hh"
#include "sicktoolbox/SickNAV350Utility.hh" // for byye-order conversions where necessary

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A default constructor
   */
  SickNav350Message::SickNav350Message( ) :
    SickMessage< SICK_NAV350_MSG_HEADER_LEN, SICK_NAV350_MSG_PAYLOAD_MAX_LEN, SICK_NAV350_MSG_TRAILER_LEN >()  {

    /* Initialize the object */
    Clear(); 
  }
  
  /**
   * \brief Another constructor.
   * \param *payload_buffer The payload for the packet as an array of bytes (including the header)
   * \param payload_length The length of the payload array in bytes
   */
  SickNav350Message::SickNav350Message( const uint8_t * const payload_buffer, const unsigned int payload_length ) :
    SickMessage< SICK_NAV350_MSG_HEADER_LEN, SICK_NAV350_MSG_PAYLOAD_MAX_LEN, SICK_NAV350_MSG_TRAILER_LEN >()  {

    /* Build the message object (implicit initialization) */
    BuildMessage(payload_buffer,payload_length); 
  }
  
  /**
   * \brief Constructs a Sick Nav350 message given parameter values
   * \param *payload_buffer An address of the first byte to be copied into the message's payload
   * \param payload_length The number of bytes to be copied into the message buffer
   */
  void SickNav350Message::BuildMessage( const uint8_t * const payload_buffer, const unsigned int payload_length ) {

    /* Call the parent method
     * NOTE: The parent method resets the object and assigns _message_length, _payload_length,
     *       _populated and copies the payload into the message buffer at the correct position
     */
    SickMessage< SICK_NAV350_MSG_HEADER_LEN, SICK_NAV350_MSG_PAYLOAD_MAX_LEN, SICK_NAV350_MSG_TRAILER_LEN >
      ::BuildMessage(payload_buffer,payload_length);
    
    /*
     * Set the message header!
     */
    _message_buffer[0] = 0x02; // STX
    

    /*
     * Set the message trailer (just a checksum)!
     */
    _message_buffer[_message_length-1] = 0x03;
//    std::cout<<"message length"<<_message_length<<std::endl;
/*    for (int i=0;i<_message_length;i++)
    {
    	std::cout<<(char) _message_buffer[i]<<"  "<<(int) _message_buffer[i]<<std::endl;
    }*/


  }
  
  /**
   * \brief Print the message contents.
   */
  void SickNav350Message::Print( ) const {

    std::cout.setf(std::ios::hex,std::ios::basefield);
//    std::cout << "Service code: " << (unsigned int) GetServiceCode() << std::endl;
//    std::cout << "Service subcode: " << (unsigned int) GetServiceSubcode() << std::endl;
    std::cout << std::flush;

    /* Call parent's print function */
    SickMessage< SICK_NAV350_MSG_HEADER_LEN, SICK_NAV350_MSG_PAYLOAD_MAX_LEN, SICK_NAV350_MSG_TRAILER_LEN >::Print();    
  }
  
  /**
   * \brief Compute the message checksum (single-byte XOR).
   * \param data The address of the first data element in a sequence of bytes to be included in the sum
   * \param length The number of byte in the data sequence
   */
  uint8_t SickNav350Message::_computeXOR( const uint8_t * const data, const uint32_t length ) {
    
    /* Compute the XOR by summing all of the bytes */
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < length; i++) {
      checksum ^= data[i]; // NOTE: this is equivalent to simply summing all of the bytes
    }
    
    /* done */
    return checksum;
  }
  void SickNav350Message::ParseMessage( const uint8_t * const message_buffer )
  {
  
  }

  SickNav350Message::~SickNav350Message( ) { }
  
} /* namespace SickToolbox */
