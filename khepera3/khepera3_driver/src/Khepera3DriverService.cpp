// Copyright (C) 2014 Georgia Tech Research Corporation
// see the LICENSE file included with this software

#include "Khepera3DriverService.h"

void alarm_callback(int arg) { }

Khepera3DriverService::Khepera3DriverService() {

	//Check for port parameter
	if(!(ros::param::get("khepera3/port", m_port)))
		m_port = 4555; // default port

	//Check for IP address parameter
	if(!ros::param::get("khepera3/ip_addr", m_ip_address))
		m_ip_address = "192.168.1.201"; // default IP address

	m_timeout = 1;

	ROS_INFO("Connecting to K3 at %s:%i.", m_ip_address.c_str(), m_port);

	m_data_receiver = m_node_handle.advertiseService("khepera3_receive_data", &Khepera3DriverService::receive_data, this);
    m_control_sender = m_node_handle.advertiseService("khepera3_send_control", &Khepera3DriverService::send_control, this);
}

Khepera3DriverService::~Khepera3DriverService() {

}

bool Khepera3DriverService::send_control(khepera3_driver::UnicycleControl::Request &req,
								      khepera3_driver::UnicycleControl::Response &res) {

	// convert unicycle to differential driver
	// limit v \in [-0.3148,0.3148] and w \in [-2.2763,2.2763]

	const float v = fmaxf(fminf(req.linear_velocity,0.3148),-0.3148);
	const float w = fmaxf(fminf(req.angular_velocity,2.2763),-2.2763);

	const float R = 0.021; // wheel radius
	const float L = 0.0885; // wheel base length

	// hardware conversion factor
	const float SF = 6.2953e-6;

	const float vel_r = v/R + (w*L)/(2*R);
	const float vel_l = v/R - (w*L)/(2*R);

	const int right_wheel_speed = floor(vel_r*R/SF);
	const int left_wheel_speed = floor(vel_l*R/SF);

	res.status = send_control_udp(right_wheel_speed, left_wheel_speed);

	return res.status;

}

bool Khepera3DriverService::send_control_udp(int right_wheel_speed, int left_wheel_speed) {

	char message[256];
	sprintf(message, "$K3DRV,REQ,CTRL,%d,%d", right_wheel_speed, left_wheel_speed);

	  /* Send received datagram back to the client */
	printf("Sending request: %s\n", message);
	if (sendto(m_socket, message, strlen(message), 0,
	  (struct sockaddr *) &m_server_address, sizeof(m_server_address)) != strlen(message)) {
	  ROS_ERROR("sendto() sent a different number of bytes than expected");
	  return false;
	}

	printf("Waiting to receive a message on port %d (timeout = %ds).\n", m_port, m_timeout);

	char reply[256];
	int reply_length;
	socklen_t client_address_length;
	struct sockaddr_in client_address;

	alarm(m_timeout);
	/* Block until receive message from a client */
	if ((reply_length = recvfrom(m_socket, reply, 256, 0,
		(struct sockaddr *) &client_address, &client_address_length)) < 0) {
		if(errno == EINTR) {
			alarm(0);
			ROS_ERROR("timeout() received.");
			return false;
		} else {
			ROS_ERROR("recvfrom() failed");
			return false;
		}
	}

	return true;

}

bool Khepera3DriverService::receive_data(khepera3_driver::SensorData::Request &req,
								  khepera3_driver::SensorData::Response &res) {
	char reply[256];
	res.status = receive_data_udp(reply);

	if(res.status) {

		char *token, *saveptr, *delim = ",";

		token = strtok_r(reply, delim, &saveptr);
		if (token == NULL || strcmp(token, "$K3DRV") != 0) {
			ROS_ERROR("Parsing failed: expected $K3DRV token");
			res.status = false;
			return res.status;
		}

		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL || strcmp(token, "RES") != 0) {
			ROS_ERROR("Parsing failed: expected RES token");
			res.status = false;
			return res.status;
		}

		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL || strcmp(token, "DATA") != 0) {
			ROS_ERROR("Parsing failed: expected DATA token");
			res.status = false;
			return res.status;
		}

		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL || strcmp(token, "IR") != 0) {
			ROS_ERROR("Parsing failed: expected IR token");
			res.status = false;
			return res.status;
		}

		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL) {
			ROS_ERROR("Parsing failed: expected IR_COUNT token");
			res.status = false;
			return false;
		}

		res.infrared_sensor_count = atoi(token);

		for (int i = 0; i < res.infrared_sensor_count; i++) {
			token = strtok_r(NULL, delim, &saveptr);
			if (token == NULL) {
				ROS_ERROR("Parsing failed: expected IR_%d token", i);
				res.status = false;
				return false;
			}
			res.infrared_sensors[i] = atoi(token);
		}

		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL || strcmp(token, "ENC") != 0) {
			ROS_ERROR("Parsing failed: expected ENC token");
			res.status = false;
			return false;
		}

		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL) {
			ROS_ERROR("Parsing failed: expected ENC_COUNT token");
			res.status = false;
			return false;
		}

		res.wheel_encoder_count = atoi(token);

		for (int i = 0; i < res.wheel_encoder_count; i++) {
			token = strtok_r(NULL, delim, &saveptr);
			if (token == NULL) {
				ROS_ERROR("Parsing failed: expected ENC_%d token", i);
				res.status = false;
				return false;
			}
			res.wheel_encoders[i] = atoi(token);
		}

	}

	return res.status;

}

bool Khepera3DriverService::receive_data_udp(char *reply) {

	char message[256];
	socklen_t client_address_length;
	struct sockaddr_in client_address;
	int reply_length;
	sprintf(message, "$K3DRV,REQ,DATA");

	  /* Send received datagram back to the client */
	printf("Sending request: %s\n", message);
	if (sendto(m_socket, message, strlen(message), 0,
	  (struct sockaddr *) &m_server_address, sizeof(m_server_address)) != strlen(message)) {
	  ROS_ERROR("sendto() sent a different number of bytes than expected");
	  return false;
	}

	printf("Waiting to receive a message on port %d (timeout = %ds).\n", m_port, m_timeout);

	alarm(m_timeout);
	/* Block until receive message from a client */
	if ((reply_length = recvfrom(m_socket, reply, 256, 0,
		(struct sockaddr *) &client_address, &client_address_length)) < 0) {
		if(errno == EINTR) {
			alarm(0);
			ROS_ERROR("timeout() received.");
			return false;
		} else {
			ROS_ERROR("recvfrom() failed");
			return false;
		}
	}

	reply[reply_length] = '\0';

	return true;
}

bool Khepera3DriverService::initialize() {
	char message[256];
	sprintf(message, "$K3DRV,REQ,INIT");

	  /* Send received datagram back to the client */
	printf("Sending request: %s\n", message);
	if (sendto(m_socket, message, strlen(message), 0,
	  (struct sockaddr *) &m_server_address, sizeof(m_server_address)) != strlen(message)) {
	  ROS_ERROR("sendto() sent a different number of bytes than expected");
	  return false;
	}

	printf("Waiting to receive a message on port %d (timeout = %ds).\n", m_port, m_timeout);

	char reply[256];
	int reply_length;
	socklen_t client_address_length;
	struct sockaddr_in client_address;

	alarm(m_timeout);
	/* Block until receive message from a client */
	if ((reply_length = recvfrom(m_socket, reply, 256, 0,
		(struct sockaddr *) &client_address, &client_address_length)) < 0) {
		if(errno == EINTR) {
			alarm(0);
			ROS_ERROR("timeout() received.");
			return false;
		} else {
			ROS_ERROR("recvfrom() failed");
			return false;
		}
	}

	return true;
}

void Khepera3DriverService::connect() {

	/* Create socket for sending/receiving datagrams */
	if ((m_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		ROS_FATAL("socket() failed");
		exit(-1);
	}

	/* Construct local address structure */
	memset(&m_server_address, 0, sizeof(m_server_address));   /* Zero out structure */
	m_server_address.sin_family = AF_INET;                /* Internet address family */
	m_server_address.sin_addr.s_addr = inet_addr(m_ip_address.c_str()); /* Any incoming interface */
	m_server_address.sin_port = htons(m_port);      /* Local port */

	/* Bind to the local address */
//	if (bind(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
//		DieWithError("bind() failed");

	/* Set reading timeout */

	m_signal_callback.sa_handler = alarm_callback;
	if (sigfillset(&m_signal_callback.sa_mask) < 0) {
		ROS_FATAL("sigfillset() failed");
		exit(-1);
	}

	m_signal_callback.sa_flags = 0;

	if (sigaction(SIGALRM, &m_signal_callback, 0) < 0) {
		ROS_FATAL("sigaction() failed");
		exit(-1);
	}

	// connect
}

void Khepera3DriverService::disconnect() {
	close(m_socket);
}

void Khepera3DriverService::run() {

	while(m_node_handle.ok()) {
		ros::spin();
	}
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "khepera3_driver");

  Khepera3DriverService driver;

  driver.connect();
  while(driver.initialize() != true) {}
  driver.run();
  driver.disconnect();

  return 1;
}
