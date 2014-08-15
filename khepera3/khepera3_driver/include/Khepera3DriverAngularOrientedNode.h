/*
 * K3Driver.h
 *
 *  Created on: Jun 8, 2011
 *      Author: jdelacroix
 */

#ifndef K3DRIVER_H_
#define K3DRIVER_H_

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket() and bind() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <netdb.h>

#include <errno.h>
#include <signal.h>

#include <math.h>

#include <ros/ros.h>

#include "khepera3_driver/UnicycleControlMsg.h"

class Khepera3DriverNode {

private:

	ros::NodeHandle m_node_handle;

	ros::Subscriber mControlSubscriber;

	std::string m_ip_address;
	int m_port;
	int myID;

	struct sigaction m_signal_callback;	/* Signal for timeouts */

	int m_socket;                        /* Socket */
	struct sockaddr_in m_server_address; 	/* Local address */
//	struct sockaddr_in m_client_address; 	/* Client address */

	void send_control(khepera3_driver::UnicycleControlMsg req);
	bool send_control_udp(int right_wheel_speed, int left_wheel_speed);


public:

	Khepera3DriverNode();
	virtual ~Khepera3DriverNode();
	void run();
	void connect();
	void disconnect();
	bool initialize();
	

};

#endif /* K3DRIVER_H_ */
