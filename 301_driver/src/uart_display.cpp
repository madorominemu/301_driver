
#include <ros/ros.h>
#include <bulldog_driver/uart_display.h>

UARTDisplay::UARTDisplay(string dev)
{
  handle = -1;
  port.assign(dev);
}

UARTDisplay::~UARTDisplay()
{
  if(handle != -1)
    close(handle);

  handle = -1;
}

int UARTDisplay::connect()
{
  if(handle != -1)
    return 0;

  handle = open(port.c_str(), O_RDWR |O_NOCTTY | O_NDELAY);
  if(handle == -1)
  {
    ROS_ERROR("Error opening display port");
    return -1;
  }

  fcntl (handle, F_SETFL, O_APPEND | O_NONBLOCK & ~FNDELAY);
  initPort();

  cout<<"Opening display: '"<<port<<"'..."<<"succeeded."<<endl;
  return 0;
}


void UARTDisplay::initPort()
{
  if(handle == -1)
    return;

  int BAUDRATE = B115200;
  struct termios newtio;
  tcgetattr (handle, &newtio);

  cfsetospeed (&newtio, (speed_t)BAUDRATE);
  cfsetispeed (&newtio, (speed_t)BAUDRATE);

  newtio.c_iflag = IGNBRK;		
  newtio.c_lflag = 0;			
  newtio.c_oflag = 0;			
  newtio.c_cflag |= (CLOCAL | CREAD);	
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
  newtio.c_cflag &= ~CSIZE;		
  newtio.c_cflag |= CS8;			
  newtio.c_cflag &= ~PARENB;		
  newtio.c_cflag &= ~PARODD;		
  newtio.c_cflag &= ~CSTOPB;		
  newtio.c_cc[VMIN] = 0;
  newtio.c_cc[VTIME] = 100;

  tcflush (handle, TCIFLUSH);
  tcsetattr (handle, TCSANOW, &newtio);	
}

int UARTDisplay::writeDisplay(string str)
{
  if(handle == -1)
    return -1;

  int countSent = write(handle, str.c_str(), str.length());
  if(countSent < 0)
    return -1;

  return 0;
}


