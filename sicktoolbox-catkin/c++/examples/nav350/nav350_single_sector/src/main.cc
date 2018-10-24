/*!
 * \file main.cc
 * \brief A simple application illustrating the use of
 *        the Sick LD C++ driver using a single sector.
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

#include <stdlib.h>
#include <string>
#include <iostream>
#include <sicktoolbox/SickNAV350.hh>

/* Use the namespace */
using namespace std;
using namespace SickToolbox;

int main (int argc, char *argv[]) {
 
  /* A string for the IP address */
  string sick_ip_addr(/*DEFAULT_SICK_IP_ADDRESS*/"120.0.100.11");

  /* Check the num of args */
  if(argc > 2 || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cerr << "Usage: nav350_single_sector [SICK IP ADDRESS]" << endl
	      << "Ex. nav350_single_sector 192.168.1.11" << endl;
    return -1;
  }
  
  /* Assign the IP address */
  if(argc == 2) {
    sick_ip_addr = argv[1];
  }

  /* Define the data buffers */
  double values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int num_values = 0;

  /* Define the bounds for a single sector */
  double sector_start_ang = 90;
  double sector_stop_ang = 270;

  /* Define the object */
  SickNav350 sick_nav350(sick_ip_addr);

  /*
   * Initialize the Sick LD
   */
  try {
    sick_nav350.Initialize();
  }
  
  catch(...) {
    cerr << "Initialize failed! Are you using the correct IP address?" << endl;
    return -1;
  }
  
  try {
	sick_nav350.GetSickIdentity();
	   std::cout<<"Identitet"<<std::endl;	
        sick_nav350.SetOperatingMode(4);
	std::cout<<"Mode"<<std::endl;
	sick_nav350.GetDataNavigation(1,1);
  }
  /* Catch any exceptions */ 
  catch(...) {
    cerr << "An error occurred!" << endl;
  }
	while(1)
	{
		std::cout<<"Podatak"<<std::endl;
		sick_nav350.GetDataNavigation(1,1);
	}

  /*
   * Uninitialize the device
   */
  try {
    //sick_nav350.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
  /* Success !*/
  return 0;
  
}
