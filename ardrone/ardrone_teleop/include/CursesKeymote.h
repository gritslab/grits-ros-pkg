// Copyright (C) 2014 Georgia Tech Research Corporation
// see the LICENSE file included with this software

#ifndef CURSESKEYMOTE_H_
#define CURSESKEYMOTE_H_

#include <ncurses.h>

#include <ros/ros.h>

#include <ardrone_driver/ARDroneControl.h>
#include <ardrone_driver/ARDroneCommand.h>

class CursesKeymote {

private:
	ros::NodeHandle m_node_handle;

	ros::ServiceClient m_control_srv_client;
	ros::ServiceClient m_command_srv_client;

	WINDOW *m_main_window;

	ardrone_driver::ARDroneCommand m_command_call;
	ardrone_driver::ARDroneControl m_control_call;

	enum CommandSignal { CMD_EMERGENCY, CMD_TAKEOFF, CMD_LAND, CMD_NOOP };
	enum ControlSignal { CTRL_ROLL, CTRL_PITCH, CTRL_YAW, CTRL_GAZ, CTRL_TRIM, CTRL_NOOP };

	void signalCommand(CommandSignal sig);
	void signalControl(ControlSignal sig, float control);

	float m_control_effort;

	uint8_t m_timeout;

public:
	CursesKeymote();
	virtual ~CursesKeymote();

	void run(void );
	void stop(void );
	bool initialize(void );
};

#endif /* CURSESKEYMOTE_H_ */
