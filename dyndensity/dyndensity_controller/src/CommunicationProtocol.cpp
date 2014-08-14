//
//  communicationProtocol.h
//  GLParticles1
//
//  Created by GRITS on 4/10/14.
//  Copyright (c) 2014 GRITS. All rights reserved.
//

#include "CommunicationProtocol.h"
#include <iostream>

// stuff for UDP communication to ROS server

CommunicationProtocol::CommunicationProtocol(){
	//ROS server address/port
	m_ip_address = "192.168.1.122";
	m_port = 4556;
}
CommunicationProtocol::CommunicationProtocol(char* ip_address, int port){
	m_ip_address = ip_address;
	m_port = port;
}

CommunicationProtocol::~CommunicationProtocol(){
	close(m_socket);
}

void alarm_callback(int arg) { }

bool CommunicationProtocol::receiveDensity(struct Density *density){

	socklen_t client_address_length;
	struct sockaddr_in client_address;
	int reply_length;

	struct DensitySerialized undecodedDensity[PACKET_LENGTH];

	// Block until receive message from a client 
	if ((reply_length = recvfrom(m_socket, &undecodedDensity, PACKET_LENGTH*sizeof(struct DensitySerialized), 0,
		(struct sockaddr *) &client_address, &client_address_length)) < 0) {
		if(errno == EINTR) {
			return false;
		} else {
			return false;
		}
	}

    for (int i = 0; i < PACKET_LENGTH; i++) {
        density[i].type = undecodedDensity[i].type; //WARNING: it appears that ints are transferred normally...for the machine used for now
        if (undecodedDensity[i].type == DensityTypeGaussian){
            
            //std::cout << undecodedDensity.serializedParameters.gaussianParameters.meanX << std::endl;
            //std::cout << undecodedDensity.serializedParameters.gaussianParameters.meanY << std::endl;
            //std::cout << undecodedDensity.serializedParameters.gaussianParameters.correlation << std::endl;
            //std::cout << undecodedDensity.serializedParameters.gaussianParameters.stdX << std::endl;
            //std::cout << undecodedDensity.serializedParameters.gaussianParameters.stdY << std::endl;
            
            decode(undecodedDensity[i].serializedParameters.gaussianParameters.meanX, density[i].parameters.gaussianParameters.meanX);
            decode(undecodedDensity[i].serializedParameters.gaussianParameters.meanY, density[i].parameters.gaussianParameters.meanY);
            decode(undecodedDensity[i].serializedParameters.gaussianParameters.correlation, density[i].parameters.gaussianParameters.correlation);
            decode(undecodedDensity[i].serializedParameters.gaussianParameters.stdX, density[i].parameters.gaussianParameters.stdX);
            decode(undecodedDensity[i].serializedParameters.gaussianParameters.stdY, density[i].parameters.gaussianParameters.stdY);
        }
    }


	return true;
}

bool CommunicationProtocol::sendDensity(struct Density *density) {
    
    //WARNING: really should be using htons() for ints and chars

    struct DensitySerialized serializedDensity[PACKET_LENGTH];
    
    for (int i = 0; i < PACKET_LENGTH; i++) { //packing the densities into serialized density structure
        serializedDensity[i].type = density[i].type; //
        
        if (density[i].type == DensityTypeGaussian){
            serialize(density[i].parameters.gaussianParameters.meanX, serializedDensity[i].serializedParameters.gaussianParameters.meanX);
            serialize(density[i].parameters.gaussianParameters.meanY, serializedDensity[i].serializedParameters.gaussianParameters.meanY);
            serialize(density[i].parameters.gaussianParameters.correlation, serializedDensity[i].serializedParameters.gaussianParameters.correlation);
            serialize(density[i].parameters.gaussianParameters.stdX, serializedDensity[i].serializedParameters.gaussianParameters.stdX);
            serialize(density[i].parameters.gaussianParameters.stdY, serializedDensity[i].serializedParameters.gaussianParameters.stdY);
        }
    }
    
    //shipping it out
    if (sendto(m_socket, &serializedDensity, PACKET_LENGTH*sizeof(struct DensitySerialized), 0,
               (struct sockaddr *) &m_server_address, sizeof(m_server_address)) != sizeof(struct Density)) {
        return false;
    }
    
	return true;
}

void CommunicationProtocol::serialize(float input, char* serialization){
	sprintf(serialization, "%f", input);
}
void CommunicationProtocol::decode(char* serialization, float &decoded){
	decoded = atof(serialization);
}

void CommunicationProtocol::emptyBuffer(){
    
    //read() might actually be more efficient
    //might make race condition with the remote sender's sending speed? but this should be faster...
    
    
	socklen_t client_address_length;
	struct sockaddr_in client_address;
	int reply_length;
    
	struct DensitySerialized undecodedDensity;
    
	// Block until receive message from a client
	while ((reply_length = recvfrom(m_socket, &undecodedDensity, sizeof(struct DensitySerialized), 0,
                                    (struct sockaddr *) &client_address, &client_address_length)) >= 0);
    
 
}

void CommunicationProtocol::UDPSetupServer(){
    // Establish UDP communication
    
	/* Create socket for sending/receiving datagrams */
	if ((m_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		exit(-1);
	}
    
	// Construct local address structure 
	memset(&m_server_address, 0, sizeof(m_server_address));   // Zero out structure 
	m_server_address.sin_family = AF_INET;                // Internet address family 
	m_server_address.sin_addr.s_addr = inet_addr(m_ip_address.c_str()); // Any incoming interface 
	m_server_address.sin_port = htons(m_port);      // Local port 
   
	// Bind to the local address 
    	if (bind(m_socket,  (struct sockaddr *) &m_server_address , sizeof(m_server_address) ) < 0){
    		printf("bind() error\n");
		exit(-1);
	}
    
	// Set reading timeout 
    
	m_signal_callback.sa_handler = alarm_callback;

	if (sigfillset(&m_signal_callback.sa_mask) < 0) {
		exit(-1);
	}
    
	m_signal_callback.sa_flags = 0;
    
	if (sigaction(SIGALRM, &m_signal_callback, 0) < 0) {
		exit(-1);
	}

	int bufferSize = (PACKET_LENGTH+1)*sizeof(DensitySerialized); //will probably not be able to take 11 densities since UDP and IP preambles exist.
	if (setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, &bufferSize, sizeof(bufferSize)) == -1){
    		printf("setsockopt() error\n");
		exit(-1);		
	}
    
	// connect
}

void CommunicationProtocol::UDPConnect() {
    // Establish UDP communication
    
	/* Create socket for sending/receiving datagrams */
	if ((m_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		exit(-1);
	}
    
	// Construct local address structure 
	memset(&m_server_address, 0, sizeof(m_server_address));   // Zero out structure 
	m_server_address.sin_family = AF_INET;                // Internet address family 
	m_server_address.sin_addr.s_addr = inet_addr(m_ip_address.c_str()); // Any incoming interface 
	m_server_address.sin_port = htons(m_port);      // Local port 
   
	// Bind to the local address 
    //	if (bind(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
    //		DieWithError("bind() failed");
    
	// Set reading timeout 
    
	m_signal_callback.sa_handler = alarm_callback;

	if (sigfillset(&m_signal_callback.sa_mask) < 0) {
		exit(-1);
	}
    
	m_signal_callback.sa_flags = 0;
    
	if (sigaction(SIGALRM, &m_signal_callback, 0) < 0) {
		exit(-1);
	}
    
	// connect
}

