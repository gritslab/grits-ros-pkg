#include "Khepera3Driver.h"

void alarm_callback(int arg) { }

Khepera3Driver::Khepera3Driver() {

	//Check for port parameter
	if(!(ros::param::get("khepera3/port", m_port)))
		m_port = 4555; // default port

	//Check for IP address parameter
	if(!ros::param::get("khepera3/ip_addr", m_ip_address))
		m_ip_address = "192.168.1.201"; // default IP address

	ROS_INFO("Connecting to Khepera3 at %s:%i.", m_ip_address.c_str(), m_port);

	mControlSubscriber = m_node_handle.subscribe("khepera3_send_control", 1, &Khepera3Driver::send_control, this); 
}

Khepera3Driver::~Khepera3Driver() {
	disconnect();
}

void Khepera3Driver::send_control(khepera3_driver::UnicycleControlInput msg) {

	// convert unicycle to differential driver
	// limit v \in [-0.3148,0.3148] and w \in [-2.2763,2.2763]

	const float v = fmaxf(fminf(msg.linear_velocity,0.3148),-0.3148);
	const float w = fmaxf(fminf(msg.angular_velocity,2.2763),-2.2763);

	const float R = 0.021; // wheel radius
	const float L = 0.0885; // wheel base length

	// hardware conversion factor
	const float SF = 6.2953e-6;

	const float vel_r = v/R + (w*L)/(2*R);
	const float vel_l = v/R - (w*L)/(2*R);

	int right_wheel_speed = floor(vel_r*R/SF);
	int left_wheel_speed = floor(vel_l*R/SF);

	const float correctDiff = -2*(w*L)/(2*R);
	const int quantizedCorrectDiff = floor(correctDiff*R/SF);

	if (correctDiff < 0){ //turning left
		if (right_wheel_speed >= MAXWHEELSPEED){ //if right wheel is spinning at its max
			right_wheel_speed = MAXWHEELSPEED;
			left_wheel_speed = MAXWHEELSPEED+quantizedCorrectDiff;
		}
		if (left_wheel_speed <= MINWHEELSPEED){ //if left wheel spinning at its max going backwards
			left_wheel_speed = MINWHEELSPEED;
			right_wheel_speed = MINWHEELSPEED-quantizedCorrectDiff;
		}
	}
	if (correctDiff > 0){ //turning right{
		if (left_wheel_speed >= MAXWHEELSPEED){ //if left wheel is spinning at its max
			left_wheel_speed = MAXWHEELSPEED;
			right_wheel_speed = MAXWHEELSPEED-quantizedCorrectDiff;
		}
		if (right_wheel_speed <= MINWHEELSPEED){ //if right wheel spinning at its max going backwards
			right_wheel_speed = MINWHEELSPEED;
			left_wheel_speed = MINWHEELSPEED+quantizedCorrectDiff;
		}
	}

	send_control_udp(right_wheel_speed, left_wheel_speed);

}

bool Khepera3Driver::send_control_udp(int right_wheel_speed, int left_wheel_speed) {

	char message[256];
	sprintf(message, "$K3DRV,REQ,CTRL,%d,%d", right_wheel_speed, left_wheel_speed);

	  /* Send received datagram back to the client */
	printf("Sending request: %s\n", message);
	if (sendto(m_socket, message, strlen(message), 0,
	  (struct sockaddr *) &m_server_address, sizeof(m_server_address)) != strlen(message)) {
	  ROS_ERROR("sendto() sent a different number of bytes than expected");
	  return false;
	}

	return true;

}

bool Khepera3Driver::initialize() {
	char message[256];
	sprintf(message, "$K3DRV,REQ,INIT");

	  /* Send received datagram back to the client */
	printf("Sending request: %s\n", message);
	if (sendto(m_socket, message, strlen(message), 0,
	  (struct sockaddr *) &m_server_address, sizeof(m_server_address)) != strlen(message)) {
	  ROS_ERROR("sendto() sent a different number of bytes than expected");
	  return false;
	}

	return true;
}

void Khepera3Driver::connect() {

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

void Khepera3Driver::disconnect() {
	close(m_socket);
}

void Khepera3Driver::run() {

	while(m_node_handle.ok()) {
		ros::spin();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "khepera3_driver");

  Khepera3Driver driver;

  driver.connect();
  driver.initialize();
  driver.run();
  driver.disconnect();

  return 1;
}
