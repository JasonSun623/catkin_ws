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
/*  
#include <stdlib.h>
#include <string>
#include <iostream>
#include <pthread.h>
#include <sicktoolbox/SickNav350.hh>
*/
#include <math.h>

#include "server.h"
/* Use the namespace */
using namespace std;
using namespace SickToolbox;
/*void *Server(void *arg)
{
	sem_t sem=(sem_t) arg;

}*/
void GetMeasurements(ServerPacket *sp,SickNav350 *sn)
{
    double range_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
    unsigned int num_measurements = {0};
    unsigned int sector_start_timestamp = {0};
	unsigned int sector_stop_timestamp = {0};
    double sector_step_angle = {0};
    double sector_start_angle = {0};
    double sector_stop_angle = {0};
	int i;
	sn->GetSickMeasurements(range_values,
                                        &num_measurements,
                                        &sector_step_angle,
                                        &sector_start_angle,
                                        &sector_stop_angle,
                                        &sector_start_timestamp,
                                        &sector_stop_timestamp);
	sp->m_.meas_num=num_measurements;
	sp->m_.step_angle=sector_step_angle;
	sp->m_.start_angle=sector_start_angle;
	sp->m_.stop_angle=sector_stop_angle;
	sp->m_.timestamp=sector_start_timestamp;
	for (i=0;i<num_measurements;i++)
	{
		sp->m_.distance[i]=range_values[i];
	}	
	sp->m_.ReflectorData_=sn->ReflectorData_;
	sp->m_.PoseData_=sn->PoseData_;
}
void OpenPort(ServerPacket *sp/*,ServerPacket *sp1*/)
{	
	struct sockaddr_in self;

	int sockfd;
	/*---Create streaming socket---*/
    if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
	{
		perror("Socket");
		exit(errno);
	}

	/*---Initialize address/port structure---*/
	bzero(&self, sizeof(self));
	self.sin_family = AF_INET;
	self.sin_port = htons(MY_PORT);
	self.sin_addr.s_addr = INADDR_ANY;

	int optval;
	int optlen;
	char *optval2;
	optval = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);



	/*---Assign a port number to the socket---*/
    if ( bind(sockfd, (struct sockaddr*)&self, sizeof(self)) != 0 )
	{
		perror("socket--bind");
		exit(errno);
	}

	/*---Make it a "listening socket"---*/
	if ( listen(sockfd, 20) != 0 )
	{
		perror("socket--listen");
		exit(errno);
	}
	sp->sockfd=sockfd;
//	sp1->sockfd=sockfd;
}
void ProcessCustomRequest(ServerPacket *sp,SickNav350 *sick_nav350)
{
	static uint8_t res[6000];
	int resp_size;
	static uint8_t req[1000];
	int req_size;

	int i;
		sem_wait(&(sp->sem2_));
		if (sp->request==1)
 		{
			for (i=0;i<sp->req_size;i++)
			{
				req[i]=sp->req[i];
			}
			req_size=sp->req_size;
			sem_post(&(sp->sem2_));	
//printf("ja\n");		
			sick_nav350->GetResponseFromCustomMessage(req,req_size,res,&resp_size);
			sem_wait(&(sp->sem3_));
			for (i=0;i<resp_size;i++)
			{
//				printf("%c",res[i]);
				sp->resp[i]=res[i];
			}
//			printf("\n");
			sp->resp_size=resp_size;
			sp->request=0;
			sem_post(&(sp->sem3_));
		}
		else
		{
			sem_post(&(sp->sem2_));
		}

}
int main (int argc, char *argv[]) {


	sem_t sem;
 	
	ServerPacket sp;  
//	ServerPacket sp1;
	std::vector<ServerPacket *> sp_vector;

	OpenPort(&sp/*,&sp1*/);
	
	sp_vector.push_back(&sp);

	sp.m_.ReflectorData_.num_reflector=0;
	sem_t len;
	pthread_t thread;
	pthread_create(&thread,NULL,Server,(void *)(&sp));
/*	pthread_t thread1;
	pthread_create(&thread1,NULL,Server,(void *)(&sp1));
*/
//	sem_post(&sp.sem1_);
	uint8_t res[6000];
	int resp_size;
	uint8_t req[6000];
	int req_size;

/*	while(1)
	{
		sem_wait(&(sp.sem1_));
		printf("ja\n");
		sem_post(&(sp.sem1_));
	
	}
	return 0;*/
  /* A string for the IP address */
  //string sick_ip_addr(DEFAULT_SICK_IP_ADDRESS);
	string sick_ip_addr("192.168.1.100");

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
sick_nav350.PoseData_.x=1;
sick_nav350.PoseData_.y=2;
sick_nav350.PoseData_.phi=3;

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
	 std::cout<<"Identitet"<<std::endl   ;	
       sick_nav350.SetOperatingMode(4);
  }
  /* Catch any exceptions */
  catch(...) {
    cerr << "An error occurred!" << endl;
  }  
int i;
		sem_post(&(sp.sem1_));
		sem_post(&(sp.sem2_));
		 sem_post(&(sp.sem3_));
		sem_post(&(sp.sem4_));

/*		sem_post(&(sp1.sem1_));
		sem_post(&(sp1.sem2_));
		sem_post(&(sp1.sem3_));
*/
	ServerPacket* spp;
	pthread_t* threadp;
	FILE *f;
int count=0;
struct timeval  tv;

	while(1)
	{
		count++;
		sick_nav350.GetDataNavigation(1,1);
/*		sem_wait(&(sp.sem1_));
		GetMeasurements(&sp,&sick_nav350);
		sem_post(&(sp.sem1_));*/
//printf("size = %d\n",sp_vector.size());
		for (i=0;i<sp_vector.size();i++)
		{
			sem_wait(&(sp_vector[i]->sem1_));
			GetMeasurements(sp_vector[i],&sick_nav350);
			sem_post(&(sp_vector[i]->sem1_));
		 ProcessCustomRequest(sp_vector[i],&sick_nav350);
	
			if (i==sp_vector.size()-1)
			{
				sem_wait(&(sp_vector[i]->sem4_));
				if (sp_vector[i]->accepted==1)
				{
					spp=new ServerPacket();
					sem_post(&(spp->sem1_));
					sem_post(&(spp->sem2_));
					sem_post(&(spp->sem3_));
					sem_post(&(spp->sem4_));
					spp->sockfd=sp_vector[i]->sockfd;
					sp_vector.push_back(spp);
					threadp=new pthread_t();
					pthread_create(threadp,NULL,Server,(void *)(spp));
				}
				sem_post(&(sp_vector[i]->sem4_));
				break;
			}		
		}
/*		sem_wait(&(sp1.sem1_));
		GetMeasurements(&sp1,&sick_nav350);
		sem_post(&(sp1.sem1_));*/
//		 ProcessCustomRequest(&sp,&sick_nav350);
//		 ProcessCustomRequest(&sp1,&sick_nav350);
//		if (count==1)
//		{
//			count=0;
gettimeofday(&tv, NULL);

double time_in_mill = 
         (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
			f=fopen("data.txt","a");
	int br=sick_nav350.PoseData_.phi-180000-1250-300;
	if (br<0) br+=360;
				printf("%d %d %d %d %d\n",sick_nav350.PoseData_.x-(int) (529*cos(br/1000.*3.14159/180)),sick_nav350.PoseData_.y-(int) (529*sin(br/1000.*3.14159/180)),br,tv.tv_sec,tv.tv_usec/1000);
				fprintf(f,"%d %d %d %d %d\n",sick_nav350.PoseData_.x,sick_nav350.PoseData_.y,sick_nav350.PoseData_.phi,tv.tv_sec,tv.tv_usec/1000);
			fclose(f);
		//}
		//printf("count = %d\n",count);
		usleep(20000);	
/*		for (i=0;i<100000;i++)
		{
		}*/	
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
