#include <ros/ros.h>
#include <bulldog_driver/motor_controller.h>
MotorController::MotorController(std::string dev)
{
	device_connect_ = false;
	handle = -1;
	port.assign(dev);
}

MotorController::~MotorController()
{
	if (handle != -1)
		close(handle);
	handle = -1;
}

int MotorController::connect()
{
	if (handle != -1)
		return 0;
	handle = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (handle == -1)
	{
		ROS_ERROR("Error opening controller port");
		return -1;
	}
	fcntl(handle, F_SETFL, O_APPEND | O_NONBLOCK & ~FNDELAY);
	initPort();
	std::cout << "Opening controller: '" << port << "'..."
			  << "succeeded." << std::endl;
	device_connect_ = true;
	initDevice();
	return 0;
}

void MotorController::initPort()
{
	if (handle == -1)
		return;
	int BAUDRATE = B460800;
	struct termios newtio;
	tcgetattr(handle, &newtio);
	cfsetospeed(&newtio, (speed_t)BAUDRATE);
	cfsetispeed(&newtio, (speed_t)BAUDRATE);
	newtio.c_iflag = IGNBRK;
	newtio.c_lflag = 0;
	newtio.c_oflag = 0;
	newtio.c_cflag |= (CLOCAL | CREAD);
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8;
	newtio.c_cflag |= (PARENB | PARODD);
	newtio.c_cflag |= INPCK;
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cc[VMIN] = 0;
	newtio.c_cc[VTIME] = 0.1;

	tcflush(handle, TCIFLUSH);
	tcsetattr(handle, TCSANOW, &newtio);
}

bool MotorController::initDevice()
{
	if (device_connect_)
	{
		/**
		//加速度
		std::cout<<"Test Start....."<<std::endl;
		uint8_t test_data1[8] = {0x01, 0x06, 0x10, 0x84, 0x03, 0xE8, 0x00, 0x00};
		unsigned int crc16_result = 0;
		crc16_result = getRtuCrc(test_data1, 6);
		test_data1[7] = ceil(crc16_result / 256);
		test_data1[6] = crc16_result - test_data1[7] * 256;
		for(int i=0; i<8;i++){
			printf("%02X ",test_data1[i]);
		}
		printf("\n");
		write(handle, test_data1, 8);
		usleep(20000);
		//减速度
		uint8_t test_data2[8] = {0x01, 0x06, 0x11, 0xC8, 0x0B, 0xB8, 0x00, 0x00};
		crc16_result = getRtuCrc(test_data2, 6);
		test_data2[7] = ceil(crc16_result / 256);
		test_data2[6] = crc16_result - test_data2[7] * 256;
		for(int i=0; i<8;i++){
			printf("%02X ",test_data2[i]);
		}
		printf("\n");
		write(handle, test_data2, 8);
		usleep(20000);
		//波特率
		uint8_t test_data3[8] = {0x01, 0x06, 0x10, 0xBB, 0x00, 0x08, 0x00, 0x00};
		crc16_result = getRtuCrc(test_data3, 6);
		test_data3[7] = ceil(crc16_result / 256);
		test_data3[6] = crc16_result - test_data3[7] * 256;
		for(int i=0; i<8;i++){
			printf("%02X ",test_data3[i]);
		}
		printf("\n");
		write(handle, test_data3, 8);
		sleep(1000);
		**/
		//速度置零
		setMotorSpeed(0, 0);
		uint8_t tmp_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		uint8_t ret_buffer[128] = {0x00, 0x00, 0x00, 0x00, 0x00, 0X00, 0x00, 0x00};
		//左编码器计数清零
		uint8_t clearl_enc_data[13] = {0x01, 0x10, 0x10, 0x08, 0X00, 0X02, 0X04, 0X00, 0X00, 0X00, 0X00, 0X3F, 0XC9};
		int send_count = write(handle, clearl_enc_data, 13);
		if (send_count < 0)
		{
			ROS_ERROR("Write data failed!");
			return false;
		}
		usleep(25000);
		//读取数据
		size_t read_size = 0;
		size_t size = 0;
		while ((read_size = read(handle, ret_buffer, 128)) > 0)
		{
			if (read_size < 128)
				break;
		}
		//右编码器计数清零
		uint8_t clearr_enc_data[13] = {0x01, 0x10, 0x10, 0xE0, 0X00, 0X02, 0X04, 0X00, 0X00, 0X00, 0X00, 0X30, 0X27};
		usleep(25000);
		int send_count1 = write(handle, clearr_enc_data, 13);
		if (send_count1 < 0)
		{
			ROS_ERROR("Write data failed!");
			return false;
		}
		usleep(25000);
		//读取数据
		read_size = 0;
		size = 0;
		while ((read_size = read(handle, ret_buffer, 128)) > 0)
		{
			if (read_size < 128)
				break;
		}
		//保存EEP
		uint8_t save_data[8] = {0x01, 0x06, 0x10, 0xC0, 0XAA, 0X60, 0XF3, 0XBE};
		usleep(20000);
		int send_count2 = write(handle, save_data, 8);
		if (send_count2 < 0)
		{
			ROS_ERROR("Write data failed!");
			return false;
		}
		sleep(2);
		//读取数据
		read_size = 0;
		size = 0;
		while ((read_size = read(handle, ret_buffer, 128)) > 0)
		{
			if (read_size < 128)
				break;
		}
		uint8_t data[8] = {0x01, 0x06, 0x10, 0x5A, 0X00, 0X0A, 0X2D, 0X1E};
		//写入数据
		usleep(20000);
		int send_count3 = write(handle, data, 8);
		if (send_count3 < 0)
		{
			ROS_ERROR("Write data failed!");
			return false;
		}
		sleep(2);
		//读取数据
		read_size = 0;
		size = 0;
		while ((read_size = read(handle, ret_buffer, 128)) > 0)
		{
			if (read_size < 128)
				break;
		}

		ROS_INFO("Init controller device success!");
		return true;
	}
	else
	{
		ROS_ERROR("Can Not find controller device port!");
		return false;
	}
}

int *MotorController::readCountAndSpeed()
{
	if (device_connect_)
	{
		uint8_t data[8] = {0x01, 0x03, 0x11, 0xEA, 0X00, 0X0E, 0XE0, 0XC6};
		uint8_t ret_buffer[128] = {0x00, 0x00, 0x00, 0x00, 0x00, 0X00, 0x00, 0x00};
		uint8_t tmp_buffer[17] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		unsigned int crc16_result = 0;
		;
		//写入数据
		usleep(15000);
		int send_count = write(handle, data, 8);
		if (send_count < 0)
		{
			ROS_ERROR("Can Not find controller device port!");
			return NULL;
		}
		usleep(20000);
		//读取数据
		size_t read_size = 0;
		size_t size = 0;

		while ((read_size = read(handle, ret_buffer, 128)) > 0)
		{
			if (read_size < 128)
				break;
		}

		printf("recice: ");
		for (int i = 0; i < 33; i++)
		{
			printf("%02X ", ret_buffer[i]);
		}
		printf("\n");

		// usleep(1500);
		//计算反馈的值
		uint8_t left_enc_hex[4] = {0x00, 0x00, 0x00, 0x00};
		uint8_t right_enc_hex[4] = {0x00, 0x00, 0x00, 0x00};
		uint8_t left_speed_hex[2] = {0x00, 0x00};
		uint8_t right_speed_hex[2] = {0x00, 0x00};
		uint8_t left_current_hex[2] = {0x00, 0x00};
		uint8_t right_current_hex[2] = {0x00, 0x00};
		uint8_t voltage_hex[2] = {0x00, 0x00};
		uint8_t temperature_hex[2] = {0x00, 0x00};
		left_enc_hex[0] = ret_buffer[4];
		left_enc_hex[1] = ret_buffer[3];
		left_enc_hex[2] = ret_buffer[6];
		left_enc_hex[3] = ret_buffer[5];
		right_enc_hex[0] = ret_buffer[8];
		right_enc_hex[1] = ret_buffer[7];
		right_enc_hex[2] = ret_buffer[10];
		right_enc_hex[3] = ret_buffer[9];
		left_speed_hex[0] = ret_buffer[12];
		left_speed_hex[1] = ret_buffer[11];
		right_speed_hex[0] = ret_buffer[14];
		right_speed_hex[1] = ret_buffer[13];
		right_current_hex[0] = ret_buffer[16];
		right_current_hex[1] = ret_buffer[15];
		left_current_hex[0] = ret_buffer[18];
		left_current_hex[1] = ret_buffer[17];
		voltage_hex[0] = ret_buffer[28];
		voltage_hex[1] = ret_buffer[27];
		temperature_hex[0] = ret_buffer[30];
		temperature_hex[1] = ret_buffer[29];
		int32_t *left_enc;
		int32_t *right_enc;
		int16_t *left_speed;
		int16_t *right_speed;
		int16_t *left_current;
		int16_t *right_current;
		int16_t *voltage;
		int16_t *temperature;
		left_enc = (int32_t *)left_enc_hex;
		right_enc = (int32_t *)right_enc_hex;
		left_speed = (int16_t *)left_speed_hex;
		right_speed = (int16_t *)right_speed_hex;
		left_current = (int16_t *)left_current_hex;
		right_current = (int16_t *)right_current_hex;
		voltage = (int16_t *)voltage_hex;
		temperature = (int16_t *)temperature_hex;
		ROS_DEBUG_STREAM_NAMED("info_read_size", "info_read_size: " << read_size);
		ROS_DEBUG_STREAM_NAMED("Encoder", "left_enc: " << (int)*left_enc << "   "
													   << "right_enc: " << (int)*right_enc);
		ROS_DEBUG_STREAM_NAMED("RPMspeed", "left_speed: " << (int)*left_speed << "   "
														  << "right_speed: " << (int)*right_speed);
		ROS_DEBUG_STREAM_NAMED("Current", "left_current: " << (int)*left_current << "   "
														   << "right_current: " << (int)*right_current);
		ROS_DEBUG_STREAM_NAMED("Voltage", "voltage: " << (int)*voltage);
		ROS_DEBUG_STREAM_NAMED("Temperature", "temperature: " << (int)*temperature);
		static int return_data[8];
		return_data[0] = (int)*left_enc;
		return_data[1] = (int)*right_enc;
		return_data[2] = (int)*left_speed;
		return_data[3] = (int)*right_speed;
		return_data[4] = (int)*left_current;
		return_data[5] = (int)*right_current;
		return_data[6] = (int)*voltage;
		return_data[7] = (int)*temperature;
		return return_data;
	}
	else
	{
		ROS_ERROR("Can Not find controller device port!");
	}
}

bool MotorController::setMotorSpeed(int left_speed, int right_speed)
{
	if (device_connect_)
	{
		uint8_t data[13] = {0x01, 0x10, 0x11, 0x5C, 0X00, 0X02, 0X04, 0X00, 0X00, 0X00, 0X00, 0x00, 0x00};
		uint8_t ret_buffer[128] = {0x00, 0x00, 0x00, 0x00, 0x00, 0X00, 0x00, 0x00};
		uint8_t tmp_buffer[13] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0X00, 0x00, 0x00};
		int16_t left_vel = (int16_t)left_speed;
		int16_t right_vel = (int16_t)right_speed;
		std::cout << "left_vel: " << left_vel << "right_vel: " << right_vel << std::endl;
		data[7] = (uint8_t)(left_vel >> 8 & 0x00FF);
		data[8] = (uint8_t)(left_vel >> 0 & 0x00FF);
		data[9] = (uint8_t)(right_vel >> 8 & 0x00FF);
		data[10] = (uint8_t)(right_vel >> 0 & 0x00FF);
		unsigned int crc16_result = 0;
		crc16_result = getRtuCrc(data, 11);
		data[12] = ceil(crc16_result / 256);
		data[11] = crc16_result - data[12] * 256;
		/**
		for(int i=0; i<13;i++){
			printf("%02X ",data[i]);
		}
		printf("\n");
		**/
		//写入数据
		usleep(15000);
		int send_count = write(handle, data, 13);
		if (send_count < 0)
		{
			ROS_ERROR("Can Not find controller device port!");
			return false;
		}

		//读取数据

		size_t read_size = 0;
		size_t size = 0;
		usleep(20000);
		while ((read_size = read(handle, tmp_buffer, 13)) > 0)
		{
			std::cout << "read_size: " << read_size << std::endl;
		}
		ROS_DEBUG_STREAM_NAMED("speed_read_size", "speed_read_size: " << read_size);
	}
	else
	{
		ROS_ERROR("Can Not find controller device port!");
	}
}

unsigned short MotorController::getRtuCrc(unsigned char *data, unsigned char length)
{
	unsigned short reg_crc = 0xffff;
	while (length--)
	{
		reg_crc ^= *data++;
		for (int j = 0; j < 8; j++)
		{
			if (reg_crc & 0x01)
			{
				reg_crc = (reg_crc >> 1) ^ 0xa001;
			}
			else
			{
				reg_crc = reg_crc >> 1;
			}
		}
	}
	return reg_crc;
}

int MotorController::getDigitalInput(int number)
{
}

int MotorController::enableDevice(int type)
{
	uint8_t data[8] = {0x01, 0x06, 0x10, 0x5A, 0X00, 0X01, 0X6C, 0XD9};  //去使能
	uint8_t ret_buffer[128] = {0x00, 0x00, 0x00, 0x00, 0x00, 0X00, 0x00, 0x00};
	if(type == 1)
	{
		data[5] = 0x0A;  //上使能
		data[6] = 0X2D;
		data[7] = 0X1E;
	}
	//写入数据
	usleep(15000);
	int send_count3 = write(handle, data, 8);
	if (send_count3 < 0)
	{
		ROS_ERROR("Write data failed!");
		return -1;
	}
	usleep(15000);
	//读取数据
	int read_size = 0;
	int size = 0;
	while ((read_size = read(handle, ret_buffer, 128)) > 0)
	{
		if (read_size < 128)
			break;
	}
	if(type != 1)
	{
		ROS_INFO("Disable motor controller success!");
		return -1;
	}
	else
	{
		ROS_INFO("Enable motor controller success!");
		return 1;
	}
}