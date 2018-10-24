#ifndef SERVER_H
#define SERVER_H
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <resolv.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <pthread.h>
#include <semaphore.h>
#include <sicktoolbox/SickNAV350.hh>

#define MY_PORT		2111
#define MAXBUF		6024
using namespace SickToolbox;
class Measurements{
public:
    sick_nav350_reflector_tag ReflectorData_;
    sick_nav350_pose_tag PoseData_;

	int meas_num;
	double distance[3600];
	double step_angle,start_angle,stop_angle,timestamp;

};
class ServerPacket
{
public:
	int sockfd;
	sem_t sem1_;
	sem_t sem2_;
	sem_t sem3_;
	sem_t sem4_;
	int accepted;
	Measurements m_;
	char req[6000];
	int req_size;
	int request;
	char resp[6000];
	int resp_size;
	ServerPacket()
	{
		accepted=0;
	}
	~ServerPacket()
	{
		if (sockfd>-1)
		{
			close(sockfd);
			sockfd=-1;			
		}
	}
};
int ConvertNumberToString(int num,char *str);
int GetPoseData(ServerPacket *sp,char *res);
int GetPose(ServerPacket *sp,char *res);
int DoMapping(ServerPacket *sp,char *res);
int ChangeState(char *res);
int CheckRequest(char* buffer,int count,ServerPacket *sp,char *res,int &last);
void *Server(void *arg);
#endif
