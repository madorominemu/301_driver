/*
 * @Author: wzy
 * @Date: 2022-12-30 11:13:09
 * @LastEditTime: 2023-04-04 14:52:33
 * @FilePath: /bulldog_driver/include/bulldog_driver/motor_controller.h
 * @Description: 头部注释
 */
#ifndef __MOTORCONTROLLER_H_
#define __MOTORCONTROLLER_H_

#include <string>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sstream>
#include <unistd.h>
#include <bulldog_driver/Constants.h>
#include <bulldog_driver/ErrorCodes.h>
#include <unistd.h>


class MotorController {

  private:
    int handle;
    std::string port;

  public:
    MotorController(std::string dev);
    ~MotorController();
    bool device_connect_;
    bool initDevice();
    int OpenDevice(std::string port, unsigned int baudrate_);
    int getBatteryVoltage();
    int getControllerTemp();
    int * readCountAndSpeed();
    int * getMotorCurrent();
    bool setMotorSpeed(int left_speed, int right_speed);
    unsigned short getRtuCrc(unsigned char* data , unsigned char length);
    int hexToDec(char* hex);
    void initPort();
    int connect();
    int enableDevice(int type);
    //not use now
    int getDigitalInput(int number);
};

#endif
