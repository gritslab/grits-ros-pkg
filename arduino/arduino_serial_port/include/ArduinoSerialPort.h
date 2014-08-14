/*
 * ArduinoSerialPort.h
 *
 *  Created on: Jun 4, 2011
 *      Author: Jean-Pierre de la Croix
 */

#ifndef ARDUINOSERIALPORT_H_
#define ARDUINOSERIALPORT_H_

#include <ros/ros.h>
#include <ros/advertise_service_options.h>

#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h>
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>

#include <arduino_serial_port/SerialPacketWrite.h>
#include <arduino_serial_port/SerialPacketRead.h>
#include <arduino_serial_port/SerialPacketReadEOL.h>

using namespace arduino_serial_port;

class ArduinoSerialPort {

private:
	ros::NodeHandle mNodeHandle;

	ros::ServiceServer mSerialPortWriter;
	ros::ServiceServer mSerialPortReader;
	ros::ServiceServer mSerialPortReaderEOL;

	int mSerialPort;
	std::string mSerialPortName;
	int mSerialPortBaudRate;

	bool serial_port_write(arduino_serial_port::SerialPacketWrite::Request &req,
		 				   arduino_serial_port::SerialPacketWrite::Response &res);

	bool serial_port_read(arduino_serial_port::SerialPacketRead::Request &req,
						  arduino_serial_port::SerialPacketRead::Response &res);

	bool serial_port_read_eol(arduino_serial_port::SerialPacketReadEOL::Request &req,
		                      arduino_serial_port::SerialPacketReadEOL::Response &res);

	int serial_port_init(const char* device_name, int baud);

public:
	ArduinoSerialPort();
	virtual ~ArduinoSerialPort();
	void run();
	void setup();
};

#endif /* ARDUINOSERIALPORT_H_ */
