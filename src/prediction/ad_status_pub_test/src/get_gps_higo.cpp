#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

int set_opt(int fd, int bSpeed, int dBits, char parity, int stopBit)
{
  struct termios newtio, oldtio;
  if (tcgetattr(fd, &oldtio) != 0)
  {
    perror("tcgetattr");
    exit(1);
  }

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD; //将本地模式(CLOCAL)和串行数据接收(CREAD)设置为有效
  /*这里有两个选项应当一直打开，一个是CLOCAL，另一个是CREAD。这两个选项可以保证你的程序不会
  变成端口的所有者，而端口所有者必须去处理发散性作业控制和挂断信号，同时还保证了串行接口驱动会读取过来的数据字节。*/

  newtio.c_cflag &= ~CSIZE; //屏蔽数据位

  switch (dBits)
  {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8; // 8 data bits
    break;
  }

  //设置奇偶位
  switch (parity)
  {
  case 'O':
    newtio.c_cflag |= PARENB;           //使能奇偶校验
    newtio.c_cflag |= PARODD;           //奇
    newtio.c_iflag |= (INPCK | ISTRIP); //将奇偶校验设置为有效同时从接收字串中脱去奇偶校验位
    break;
  case 'E':
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'N':
    newtio.c_cflag &= ~PARENB;
    break;
  }

  //设置波特率
  switch (bSpeed)
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
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }

  //设置停止位
  if (stopBit == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;

  newtio.c_cc[VTIME] = 0;   //设置等待数据时间，单位：0.1秒
  newtio.c_cc[VMIN]  = 100; // Minimum number of characters to read

  tcflush(fd, TCIFLUSH); //刷新缓冲区，让输入输出数据有效：Flush input and output buffers and make the change
  if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) // TCSANOW标志所有改变必须立刻生效而不用等到数据传输结束
  {
    perror("com set error");
    return -1;
  }

  return 0;
}

int main(void)
{
  int atFd, nmeaFd, nset1, nwrite, nread;
  char at[20], nmea[1024];

  atFd = open("/dev/ttyUSB2", O_RDWR); //打开at串口
  if (atFd == -1)
  {
    perror("atFd:");
    exit(1);
  }

  set_opt(atFd, 4800, 8, 'N', 1); //设置at串口属性
  memset(at, 0, 20);
  memcpy(at, "AT+CGPS=1\r", sizeof("AT+CGPS=1\r"));
  nwrite = write(atFd, at, strlen(at));
  if (-1 == nwrite)
  {
    perror("at com write");
    exit(1);
  }

  nmeaFd = open("/dev/ttyUSB1", O_RDWR); //打开nmea串口

  if (nmeaFd == -1)
  {
    perror("nmeaFd:");
    exit(1);
  }

  set_opt(nmeaFd, 4800, 8, 'N', 1); //设置nmea串口属性
  while (1)
  {
    memset(nmea, 0, 1024);
    nread = read(nmeaFd, nmea, 1024); //读串口
    if (nread > 0)
    {
      printf("\n\tGPS dataLen=%d, data:\n", nread);
      nmea[nread] = '\0';
      printf("%s\n", nmea); //输出所读取的数据
    }
    sleep(2);
  }
  close(atFd);
  close(nmeaFd);
  return 0;
}