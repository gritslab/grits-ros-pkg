//
//  communicationProtocol.h
//  GLParticles1
//
//  Created by GRITS on 4/10/14.
//  Copyright (c) 2014 GRITS. All rights reserved.
//

#ifndef GLParticles1_communicationProtocol_h
#define GLParticles1_communicationProtocol_h

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket() and bind() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <netdb.h>

#include <errno.h>
#include <signal.h>

#include <string>

#define SERIALIZATION_LENGTH 8
#define PACKET_LENGTH 11 //ish

enum DensityTypes {
	DensityTypeEnd = 0, // used to mark that all densities were sent
	DensityTypeStart,
	DensityTypeGaussian
};

struct GaussianParameters{
    float meanX, meanY, correlation, stdX, stdY;
};

union Parameters{
    struct GaussianParameters gaussianParameters;
};

struct Density{
    int type;
    union Parameters parameters;
};

// for each parametrization defined, (eg Gaussian) its serialized version must be defined also
// serialization and decoding logic must also be added in sendDensity, receiveDensity.

struct GaussianParametersSerialized{
	char meanX[SERIALIZATION_LENGTH];
	char meanY[SERIALIZATION_LENGTH];
	char correlation[SERIALIZATION_LENGTH];
	char stdX[SERIALIZATION_LENGTH];
	char stdY[SERIALIZATION_LENGTH];
};

union ParametersSerialized{
    struct GaussianParametersSerialized gaussianParameters;
};

struct DensitySerialized{
    int type;
    union ParametersSerialized serializedParameters;
};

class CommunicationProtocol{

public:
	bool sendDensity(struct Density *density);
	bool receiveDensity(struct Density *density);
	void UDPConnect();
	void UDPSetupServer();

    void emptyBuffer();

	CommunicationProtocol();
	CommunicationProtocol(char* ip_address, int port);
	virtual ~CommunicationProtocol();

private:
    
    void serialize(float input, char* serialization);
	void decode(char* serialization, float &decoded);

	std::string m_ip_address;
	int m_port;
	struct sockaddr_in m_server_address;
	struct sigaction m_signal_callback;
	int m_socket;

};

#endif
