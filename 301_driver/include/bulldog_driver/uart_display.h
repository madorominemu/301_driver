#ifndef __UARTDISPLAY_H_
#define __UARTDISPLAY_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sstream>
#include <unistd.h>

using namespace std;

class UARTDisplay {
  private:
    int handle;
    string port;

  protected:
    void initPort();

  public:
    int connect();
    int writeDisplay(string str);

    UARTDisplay(string dev);
    ~UARTDisplay();
};

#endif
