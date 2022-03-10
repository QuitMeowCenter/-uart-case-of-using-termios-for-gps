#include<stdio.h>
#include<termios.h>
#include<sys/unistd.h>
#include <string.h>
#include <sys/types.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
int set_opt(int fd,int nspeed,int nbits,char nEvent,int nstop)
{
	struct termios newtio,oldtio;
	if(tcgetattr(fd,&oldtio)!=0)
	{
		perror("set up serial 1");
		return -1;
	}
	memset(&newtio,0,sizeof(newtio));
	newtio.c_cflag|=CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	newtio.c_lflag &=~(ICANON | ECHO | ECHOE | ISIG);
	newtio.c_oflag&=~OPOST;
	switch(nbits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	switch(nEvent)
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':
		newtio.c_iflag |= (INPCK｜ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= ~PARODD;
	break;
	case 'N':
		newtio.c_cflag &= ~PARENB;
	break;
	}
	switch(nspeed)
	{
		case 2400:
			cfsetispeed(&newtio, B2400);
			cfsetospeed(&newtio, B2400);
			break;
		case 4800:
			cfsetispeed(&newtio, B4800);
			cfsetospeed(&newtio, B4800);
		break;
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
		break;
		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
		break;
		default:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
		break;
	}
	if(nstop==1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if(nstop==2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	newtio.c_cc[VTIME] = 1;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	if( tcsetattr(fd, TCSANOW, &newtio)!=0)
	{
		perror("com set error");
		return -1;
	}
	return 0;
}
int open_opt(char *com)
{
	int fd;
	fd=open(com, O_RDWR|O_NOCTTY);
	char c;
	if(fd<0)
	{
		return -1;
	}
	if(fcntl(fd, F_SETFL, 0)<0)
	{
		printf("fcntl failed");
		return -1;
	}
	return fd;
}
int read_gps_raw_data(int fd,char *buf)
{
	int start=0;
	int i;
	char c;
	int iRet;
	while(1)
	{
		iRet=read(fd,&c,1);
		if(iRet==1)
		{
			if(c=='$')
			{
				start=1;
			}
			if(start)
			{
				buf[i++]=c;
			}
			if(c=='\n'||c=='\r')
			{
				return 0;
			}
			
		}
		else
		{
			return -1;
		}
	}
}

int parse_gps_raw_data(char *buf,char *time, char *lat, char *ns, char *lng, char *ew)
{
	char tmp[10];
	if(buf[0]!='$')
	{
		return -1;
	}
	else if(strncmp(buf+3,'GGA',3)!=0)
	{
		return -1;
	}
	else if(strstr(buf,',,,,,')!=NULL)
	{
		printf("please place the gps to open area");
		return -1;
	}
	else{
	sscanf(buf,"%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]",tmp,time,lat,ns,lng,ew);
	return 0;
	}
}
int main(int argc,char **argv)
{
	int fd;
	char buf[1000];
	char time[100]; 
	char lat[100];
	char ns[100];
	char lng[100]; 
	char ew[100];
	float flat,flng;
	if(argc!=2)
	{
		printf("Usage:\n");
		printf("%s</dev/ttySAC1 or other>\n",argv[0]);
		return -1;
	}
	fd=open_opt(argv[1]);
	if(fd<0)
	{
		printf("open %s failed\n",argv[1]);
		return -1;
	}
	int iRet=set_opt(fd,115200,8,N,1);
	if(iRet)
	{
		printf("set port err");
		return-1;
	}
	printf("Enter char:");
	while(1)
	{
		iRet=read_gps_raw_data(fd,buf);
		if(iRet==0)
		{
			iRet=parse_gps_raw_data(buf,time,lat,ns,lng,ew);
		}
		if(iRet==0)
		{
			printf("time:%s\n",time);
			printf("lat:%s\n",lat);
			printf("ns:%s\n",ns);
			printf("lng:%s\n",lng);
			printf("ew:%s\n",ew);
			sscanf(lat+2,"%f",&flat);
			flat=flat/60;
			flat+=(lat[0]-'0')*10+(lat[1]-'0');
			sscanf(lng+3,"%f",&flng);
			
			flng=flng/60;
			flng+=(lng[0]-'0')*100+(lng[1]-'0')*10+(lng[2]-'0');
			printf("flat:%.06f\n",flat);
			printf("flng:%.06f\n",flng);
		}
		
		
	}
	return 0;
}