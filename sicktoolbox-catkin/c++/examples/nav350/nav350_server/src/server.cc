#include "server.h"
int ConvertNumberToString(int num,char *str)
{
	if (num==0)
	{
		str[0]='0';
//		str[1]='0';
		return 1;
	}
	if (num<0)	
	{
		str[0]='-';
		num=-num;
	}
	else
	{
		str[0]='+';
	}
	int broj_zn=0,num1=num;
	while (num1>0)
	{
		num1=num1/16;
		broj_zn++;
	}
	int i;
	for (i=broj_zn-1;i>=0;i--)
	{
		if (num%16<10)
		{
			str[i]=48+num%16;
		}
		else
		{
			str[i]=65+num%16-10;
		}
		num=num/16;
	}
	return broj_zn;
}
int GetPoseData(ServerPacket *sp,char *res,char mask)
{
	int count=0,i;
	res[count++]=2;
	res[count++]='s';
	res[count++]='M';
	res[count++]='A';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='P';
	res[count++]='O';
	res[count++]='S';
	res[count++]='G';
	res[count++]='e';
	res[count++]='t';
	res[count++]='D';
	res[count++]='a';
	res[count++]='t';
	res[count++]='a';
	res[count++]=3;

	res[count++]=2;
	res[count++]='s';
	res[count++]='A';
	res[count++]='N';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='P';
	res[count++]='O';
	res[count++]='S';
	res[count++]='G';
	res[count++]='e';
	res[count++]='t';
	res[count++]='D';
	res[count++]='a';
	res[count++]='t';
	res[count++]='a';
	res[count++]=' ';
	res[count++]='0'; //version
	res[count++]=' ';
	res[count++]='0'; //error
	res[count++]=' ';
	res[count++]='0'; //wait
	res[count++]=' ';
	res[count++]=mask; //mask
	res[count++]=' ';
	res[count++]='1'; //pose data follow
	res[count++]=' ';
	char str[100];
	int size;
	size=ConvertNumberToString(sp->m_.PoseData_.x,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	res[count++]=' ';


	size=ConvertNumberToString(sp->m_.PoseData_.y,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.PoseData_.phi,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.PoseData_.optionalPoseData,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	if (sp->m_.PoseData_.optionalPoseData==1)
	{
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.outputMode,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.timeStamp,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.meanDeviation,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.positionMode,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.infoState,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.numUsedReflectors,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}

	}
//	printf("gotov pose data (slijedi reflector data)\n");
	res[count++]=' ';
	int j;

	if (mask=='0' || mask=='2')
	{
		res[count++]='1'; //landmark data follow

		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.filter,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}

		 res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.num_reflector,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
//		printf("ref count= %d\n",sp->m_.ReflectorData_.num_reflector);
		for (j=0;j<sp->m_.ReflectorData_.num_reflector;j++)
		{
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.cart[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.x[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.y[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.polar[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.dist[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.phi[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.optional[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			if (sp->m_.ReflectorData_.optional[j]==1)
			{
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.LocalID[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.GlobalID[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.type[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.subtype[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.quality[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.timestamp[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.size[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.hitCount[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.meanEchoAmplitude[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.meanEchoAmplitude[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.indexStart[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
				res[count++]=' ';
				size=ConvertNumberToString(sp->m_.ReflectorData_.indexEnd[j],str);
				for (i=0;i<size;i++)
				{
					res[count++]=str[i];		
				}
	
			}
	
		}
	}
	else
	{
		res[count++]='0';
	}
//	printf("gotov landmark data (slijedi distance data)\n");
	
	res[count++]=' ';
	if (mask=='0')
	{
		res[count++]='0';
	}
	else
	{
		res[count++]='1';
		res[count++]=' ';
		res[count++]='D';
		res[count++]='I';
		res[count++]='S';
		res[count++]='T';
		res[count++]='1';

		res[count++]=' '; //multiplier
		res[count++]='1';
	
		res[count++]=' ';
		res[count++]='0'; //offset
	
		res[count++]=' ';
		size=ConvertNumberToString((int) (sp->m_.start_angle*1000),str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
//		 printf("gotov start %d\n",count);
		res[count++]=' ';
//		printf("%d\n",(int)(sp->m_.step_angle*1000));
		size=ConvertNumberToString((int)(sp->m_.step_angle*1000),str);
//		printf("%d\n",size);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
//		printf("gotov step\n");
		res[count++]=' ';
		size=ConvertNumberToString((int) (sp->m_.timestamp),str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
//		printf("gotov timestamp\n");
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.meas_num,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
//		printf("dist count=%d\n",sp->m_.meas_num);
		for (j=0;j<sp->m_.meas_num;j++)
		{
			res[count++]=' ';
			size=ConvertNumberToString((int) sp->m_.distance[j],str);
			for (i=0;i<size;i++)
			{
			 	res[count++]=str[i];		
			}
		}
	
		res[count++]=' ';
		res[count++]='0'; //remission
	
	
		for (i=0;i<count;i++)
		{
//			printf("%c",res[i]);
		}
//		printf("\n");
	}
	res[count++]=3;

	return count;

}
int DoMapping(ServerPacket *sp,char *res)
{
	char str[100];
	int size;

	int count=0,i;
	res[count++]=2;
	res[count++]='s';
	res[count++]='M';
	res[count++]='A';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='M';
	res[count++]='A';
	res[count++]='P';
	res[count++]='D';
	res[count++]='o';
	res[count++]='M';
	res[count++]='a';
	res[count++]='p';
	res[count++]='p';
	res[count++]='i';
	res[count++]='n';
	res[count++]='g';
	res[count++]=3;

	res[count++]=2;
	res[count++]='s';
	res[count++]='A';
	res[count++]='N';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='M';
	res[count++]='A';
	res[count++]='P';
	res[count++]='D';
	res[count++]='o';
	res[count++]='M';
	res[count++]='a';
	res[count++]='p';
	res[count++]='p';
	res[count++]='i';
	res[count++]='n';
	res[count++]='g';

	res[count++]=' ';
	res[count++]='0'; //error
	res[count++]=' ';
	res[count++]='1'; //landmark data follow

	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.ReflectorData_.filter,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}

	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.ReflectorData_.num_reflector,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	int j;
//	printf("ref count= %d\n",sp->m_.ReflectorData_.num_reflector);
	for (j=0;j<sp->m_.ReflectorData_.num_reflector;j++)
	{
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.cart[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.x[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.y[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		res[count++]='0'; //no polar data follow
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.optional[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		if (sp->m_.ReflectorData_.optional[j]==1)
		{
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.LocalID[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.GlobalID[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.type[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.subtype[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.quality[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.timestamp[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.size[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.hitCount[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.meanEchoAmplitude[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.meanEchoAmplitude[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.indexStart[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.indexEnd[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}

		}

	}
//	printf("gotov landmark data (slijedi distance data)\n");
	
	res[count++]=' ';
	res[count++]='1';

	res[count++]=' ';
	res[count++]='D';
	res[count++]='I';
	res[count++]='S';
	res[count++]='T';
	res[count++]='1';

	res[count++]=' '; //multiplier
	res[count++]='1';

	res[count++]=' ';
	res[count++]='0'; //offset

	res[count++]=' ';
	size=ConvertNumberToString((int) (sp->m_.start_angle*1000),str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
//	 printf("gotov start %d\n",count);
	res[count++]=' ';
//	printf("%d\n",(int)(sp->m_.step_angle*1000));
	size=ConvertNumberToString((int)(sp->m_.step_angle*1000),str);
//	printf("%d\n",size);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
//	printf("gotov step\n");
	res[count++]=' ';
	size=ConvertNumberToString((int) (sp->m_.timestamp),str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
//	printf("gotov timestamp\n");
	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.meas_num,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
//	printf("dist count=%d\n",sp->m_.meas_num);
	for (j=0;j<sp->m_.meas_num;j++)
	{
		res[count++]=' ';
		size=ConvertNumberToString((int) sp->m_.distance[j],str);
		for (i=0;i<size;i++)
		{
		 	res[count++]=str[i];		
		}
	}

	res[count++]=' ';
	res[count++]='0'; //remission


	res[count++]=3;
	for (i=0;i<count;i++)
	{
//		printf("%c",res[i]);
	}
//	printf("\n");

	return count;

}

int GetPose(ServerPacket *sp,char *res)
{
	int count=0,i;
	res[count++]=2;
	res[count++]='s';
	res[count++]='M';
	res[count++]='A';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='P';
	res[count++]='O';
	res[count++]='S';
	res[count++]='G';
	res[count++]='e';
	res[count++]='t';
	res[count++]='P';
	res[count++]='o';
	res[count++]='s';
	res[count++]='e';
	res[count++]=3;

	res[count++]=2;
	res[count++]='s';
	res[count++]='A';
	res[count++]='N';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='P';
	res[count++]='O';
	res[count++]='S';
	res[count++]='G';
	res[count++]='e';
	res[count++]='t';
	res[count++]='P';
	res[count++]='o';
	res[count++]='s';
	res[count++]='e';
	res[count++]=' ';
	res[count++]='0'; //version
	res[count++]=' ';
	res[count++]='0'; //error
	res[count++]=' ';
	res[count++]='0'; //wait
	res[count++]=' ';
	res[count++]='1'; //pose data follow
	res[count++]=' ';
	char str[100];
	int size;
	size=ConvertNumberToString(sp->m_.PoseData_.x,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	res[count++]=' ';


	size=ConvertNumberToString(sp->m_.PoseData_.y,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.PoseData_.phi,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.PoseData_.optionalPoseData,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	if (sp->m_.PoseData_.optionalPoseData==1)
	{
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.outputMode,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.timeStamp,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.meanDeviation,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.positionMode,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.infoState,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.PoseData_.numUsedReflectors,str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}

	}
	res[count++]=3;
//	printf("gotov pose data (slijedi reflector data)\n");

	return count;

}

int GetLandmarkData(ServerPacket *sp,char *res)
{
	int count=0,i;
	res[count++]=2;
	res[count++]='s';
	res[count++]='M';
	res[count++]='A';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='L';
	res[count++]='M';
	res[count++]='D';
	res[count++]='G';
	res[count++]='e';
	res[count++]='t';
	res[count++]='D';
	res[count++]='a';
	res[count++]='t';
	res[count++]='a';
	res[count++]=3;

	res[count++]=2;
	res[count++]='s';
	res[count++]='A';
	res[count++]='N';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='L';
	res[count++]='M';
	res[count++]='D';
	res[count++]='G';
	res[count++]='e';
	res[count++]='t';
	res[count++]='D';
	res[count++]='a';
	res[count++]='t';
	res[count++]='a';
	res[count++]=' ';
	res[count++]='0'; //version
	res[count++]=' ';
	res[count++]='0'; //error
	res[count++]=' ';
	res[count++]='0'; //wait
	res[count++]=' ';
	res[count++]='1'; //mask
	res[count++]=' ';
	res[count++]='1'; //landmark data follow

	res[count++]=' ';
	char str[100];
	int size;
	size=ConvertNumberToString(sp->m_.ReflectorData_.filter,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}

	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.ReflectorData_.num_reflector,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
	int j;
//	printf("ref count= %d\n",sp->m_.ReflectorData_.num_reflector);
	for (j=0;j<sp->m_.ReflectorData_.num_reflector;j++)
	{
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.cart[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.x[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.y[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.polar[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.dist[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.phi[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		res[count++]=' ';
		size=ConvertNumberToString(sp->m_.ReflectorData_.optional[j],str);
		for (i=0;i<size;i++)
		{
			res[count++]=str[i];		
		}
		if (sp->m_.ReflectorData_.optional[j]==1)
		{
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.LocalID[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.GlobalID[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.type[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.subtype[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.quality[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.timestamp[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.size[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.hitCount[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.meanEchoAmplitude[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.meanEchoAmplitude[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.indexStart[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}
			res[count++]=' ';
			size=ConvertNumberToString(sp->m_.ReflectorData_.indexEnd[j],str);
			for (i=0;i<size;i++)
			{
				res[count++]=str[i];		
			}

		}

	}
//	printf("gotov landmark data (slijedi distance data)\n");
	
	res[count++]=' ';
	res[count++]='1';

	res[count++]=' ';
	res[count++]='D';
	res[count++]='I';
	res[count++]='S';
	res[count++]='T';
	res[count++]='1';

	res[count++]=' '; //multiplier
	res[count++]='1';

	res[count++]=' ';
	res[count++]='0'; //offset

	res[count++]=' ';
	size=ConvertNumberToString((int) (sp->m_.start_angle*1000),str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
//	 printf("gotov start %d\n",count);
	res[count++]=' ';
//	printf("%d\n",(int)(sp->m_.step_angle*1000));
	size=ConvertNumberToString((int)(sp->m_.step_angle*1000),str);
//	printf("%d\n",size);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
//	printf("gotov step\n");
	res[count++]=' ';
	size=ConvertNumberToString((int) (sp->m_.timestamp),str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
//	printf("gotov timestamp\n");
	res[count++]=' ';
	size=ConvertNumberToString(sp->m_.meas_num,str);
	for (i=0;i<size;i++)
	{
		res[count++]=str[i];		
	}
//	printf("dist count=%d\n",sp->m_.meas_num);
	for (j=0;j<sp->m_.meas_num;j++)
	{
		res[count++]=' ';
		size=ConvertNumberToString((int) sp->m_.distance[j],str);
		for (i=0;i<size;i++)
		{
		 	res[count++]=str[i];		
		}
	}

	res[count++]=' ';
	res[count++]='0'; //remission


	res[count++]=3;
	for (i=0;i<count;i++)
	{
//		printf("%c",res[i]);
	}
//	printf("\n");

	return count;

}

int ChangeState(char *res,char c)
{
	int count=0;
	res[count++]=2;
	res[count++]='s';
	res[count++]='M';
	res[count++]='A';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='E';
	res[count++]='V';
	res[count++]='A';
	res[count++]='C';
	res[count++]='h';
	res[count++]='a';
	res[count++]='n';
	res[count++]='g';
	res[count++]='e';
	res[count++]='S';
	res[count++]='t';
	res[count++]='a';
	res[count++]='t';
	res[count++]='e';
	res[count++]=3;

	res[count++]=2;
	res[count++]='s';
	res[count++]='A';
	res[count++]='N';
	res[count++]=' ';
	res[count++]='m';
	res[count++]='N';
	res[count++]='E';
	res[count++]='V';
	res[count++]='A';
	res[count++]='C';
	res[count++]='h';
	res[count++]='a';
	res[count++]='n';
	res[count++]='g';
	res[count++]='e';
	res[count++]='S';
	res[count++]='t';
	res[count++]='a';
	res[count++]='t';
	res[count++]='e';
	res[count++]=' ';
	res[count++]='0';
	res[count++]=' ';
	res[count++]=c;
	res[count++]=3;
	return count;
	
}
int CheckRequest(char* buffer,int count,ServerPacket *sp,char *res,int &last)
{
	int i,t=0;
	int poc=0;
	int kr=-1;
	char *mes;
	int mescount=0;
	int sval;
	int resp_size=0;
	for (i=0;i<count;i++)
	{
		if (buffer[i]==2 && t==0) 
		{
			poc=i;
			t=1;
		}
		if (buffer[i]==3 && t==1)
		{
			kr=i;
			t=2;			
			break;
		}
	}
//	printf("t=%d\n",t);

	if (t==2)
	{
		last=kr+1;
		mes=buffer+poc;
		mescount=kr-poc+1;
//		if (mescount>17)
//		{
			if  (strncmp(&mes[1],"sMN mNLMDGetData",16)==0)
			{
				printf("get landmark data\n");
				sem_wait(&(sp->sem1_));		
				resp_size=GetLandmarkData(sp,res);				
		 		sem_post(&(sp->sem1_));		
				return resp_size;
			} 
			else if (strncmp(&mes[1],"sMN mNPOSGetData",16)==0)
			{
				printf("get pose data\n");
				sem_wait(&(sp->sem1_));		
				resp_size=GetPoseData(sp,res,mes[mescount-2]);				
				sem_post(&(sp->sem1_));		

				return resp_size;

			} 
			else if (strncmp(&mes[1],"SMN MNPOSGetPose",16)==0)
			{
				printf("get pose\n");
				sem_wait(&(sp->sem1_));		
				resp_size=GetPose(sp,res);				
		 		sem_post(&(sp->sem1_));		
				return resp_size;
			} 
			else if (strncmp(&mes[1],"SMN MNMAPDoMapping",17)==0)
			{
			}
			else if (strncmp(&mes[1],"sMN mNEVAChangeState",17)==0)
			{
printf("change state %c\n",mes[mescount-2]);
				resp_size=ChangeState(res,mes[mescount-2]);
				return resp_size;
			}
			else
			{
		
			sem_wait(&(sp->sem2_));			
				for (i=0;i<mescount;i++)
				{
					sp->req[i]=mes[i];
				}
				sp->req_size=mescount;
				sp->request=1;
				sem_post(&(sp->sem2_));
//				printf("test\n");
				while (1)
				{					
//					printf("request=%d\n",sp->request);
					sem_wait(&(sp->sem3_));
					if (sp->request==0)
					{
//						printf("request=%d\n",sp->request);
//						printf("%d\n",sp->resp_size);
//						sem_wait(&(sp->sem3_));
						for (i=0;i<sp->resp_size;i++)
						{
							res[i]=sp->resp[i];
//							printf ("%c %d\n",res[i],i);
						}							
//						printf("\n");
						resp_size=sp->resp_size;	
						sem_post(&(sp->sem3_));
						return resp_size;
					}
					sem_post(&(sp->sem3_));

					for (i=0;i<10000;i++)
					{
					}
				}
		//	}

		}
	}
	else
	{
		last=0;
		return 0;
	}
	
}
void *Server(void *arg)
{
	ServerPacket *sp=(ServerPacket *) arg;
/*	while (1)
	{
		sem_wait(&(sp->sem1_));
		printf("ja1\n");
		sem_post(&(sp->sem1_));
	}	
*/



int sockfd=sp->sockfd;
	struct sockaddr_in self;
	char buffer[MAXBUF];
	char buffer1[1000];
	char res[20000];

	/*---Create streaming socket---*/
 /*  if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
	{
		perror("Socket");
		exit(errno);
	}

	/*---Initialize address/port structure---*/
/*	bzero(&self, sizeof(self));
	self.sin_family = AF_INET;
	self.sin_port = htons(MY_PORT);
	self.sin_addr.s_addr = INADDR_ANY;

	 /*---Assign a port number to the socket---*/
 /*   if ( bind(sockfd, (struct sockaddr*)&self, sizeof(self)) != 0 )
	{
		perror("socket--bind");
		exit(errno);
	}

	/*---Make it a "listening socket"---*/
/*	if ( listen(sockfd, 20) != 0 )
	{
		perror("socket--listen");
		exit(errno);
	}

	/*---Forever... ---*/
int count=0,count1=0,count2=0,countres=0;
	int i,last;
	int clientfd;
		 struct sockaddr_in client_addr;
		int addrlen=sizeof(client_addr);
	while (1)
	{	

		/*---accept a connection (creating a data pipe)---*/
		clientfd = accept(sockfd, (struct sockaddr*)&client_addr, (socklen_t*) &addrlen);
		sem_wait(&(sp->sem4_));
		 sp->accepted=1;
		sem_post(&(sp->sem4_));
		printf("%s:%d connected\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
		while (1)
		{
			/*---Echo back anything sent---*/
			/*send(clientfd, buffer, */count=recv(clientfd, buffer, MAXBUF, MSG_DONTWAIT);//, 0);
			for (i=0;i<count;i++)
			{
//				printf("%d %c\n",clientfd,buffer[i]);
			}
			if (count>0)
			{
//				printf("duljina klijentske poruke = %d\n",count);
					
				for (i=0;i<count;i++)
				{
					buffer1[count1]= buffer[i];
					count1++;
				}
				countres=CheckRequest(buffer1,count1,sp,res,last);
				if (last>0){
					for (int i=last;i<count1;i++)
					{
						buffer1[i-last]= buffer1[i];				
					}
					count1=count1-last;
				}
//				printf("countres=%d\n",countres);
			}	
			if ( countres>0)
			{
//				printf("clientfd=%d\n",clientfd);
				send(clientfd,res,countres,0);
//				printf("poslana poruka duljine %d\n",countres);
				countres=0;

			}
			usleep(5000);
/*			for (i=0;i<10000;i++)
			{
			}*/
		}
		/*---Close data connection---*/
		close(clientfd);
	}

	/*---Clean up (should never get here!)---*/
	close(sockfd);




}
