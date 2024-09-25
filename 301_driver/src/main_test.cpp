
#include<bulldog_driver/motor_controller.h>
#include <unistd.h>


int main(int argc, char* argv[])
{
  MotorController test;
	// test.OpenDevice("/dev/ttyUSB0", 9600);

  // while(1){
  // test.setMotorSpeed(100, -100);
  // test.readCountAndSpeed();
  // }

  uint8_t a[4] = {0x3A ,0xC9 ,0xE1 ,0xFB};
  int32_t *b;
  b = (int32_t*)a;
  std::cout<<"b: "<< (int)*b<<std::endl;


  return 0;

}
