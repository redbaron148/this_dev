#include <stdio.h>
#include <strings.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

int fd1;
char* buff;
int rd,nbytes,tries;

int main()
{
	struct termios port_settings;

	cfsetispeed(&port_settings, B115200);
	cfsetospeed(&port_settings, B115200);
	
	port_settings.c_cflag &= ~PARENB;
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
	
	buff = new char[100];
	fd1=open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
	sleep(1);
	tcsetattr(fd1, TCSANOW,&port_settings);
	sleep(1);
	fcntl(fd1, F_SETFL,0);
	sleep(1);
	write(fd1,"s 0 d -10;",10);
	sleep(1.);
	rd=read(fd1,buff,100);
	printf("bytes sent are %s\n",buff);
	close(fd1);
	delete [] buff;
	return 0;
}
