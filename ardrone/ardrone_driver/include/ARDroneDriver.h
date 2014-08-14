/*
 * ARDroneDriver.h
 *
 *  Created on: Aug 17, 2012
 *      Author: jdelacroix
 */

#ifndef ARDRONEDRIVER_H_
#define ARDRONEDRIVER_H_

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket() and bind() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <netdb.h>

#include <errno.h>
#include <signal.h>

#include <stdint.h>

#include <math.h>

#include <ros/ros.h>

#include <ardrone_driver/ARDroneCommand.h>
#include <ardrone_driver/ARDroneControl.h>

class ARDroneDriver {

private:

	ros::NodeHandle m_node_handle;

	ros::ServiceServer m_command_sender;
	ros::ServiceServer m_control_sender;

	std::string m_ip_address;
	int m_port;

//	struct sigaction m_signal_callback;	/* Signal for timeouts */

	int m_socket;                        /* Socket */
	struct sockaddr_in m_server_address; 	/* Local address */
//	struct sockaddr_in m_client_address; 	/* Client address */

//	int m_timeout;

//	void m_alarm_callback(int arg);

	bool send_control(ardrone_driver::ARDroneControl::Request &req,
					ardrone_driver::ARDroneControl::Response &res);

	bool send_command(ardrone_driver::ARDroneCommand::Request &req,
					ardrone_driver::ARDroneCommand::Response &res);

	bool send_control_udp(float roll, float pitch, float yaw, float gaz);

	enum ARDroneSignal { SIG_INIT, SIG_NORMAL, SIG_EMERGENCY, SIG_LAND, SIG_TAKEOFF };
	bool send_command_udp(ARDroneSignal s);

	bool m_is_flying;
	bool m_is_ready;

	int32_t m_sequence_number;

public:

	ARDroneDriver();
	virtual ~ARDroneDriver();
	void run();
	void connect();
	void disconnect();
	bool initialize();

};

#endif /* ARDRONEDRIVER_H_ */
