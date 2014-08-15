#ifndef _DYNAMIC_DENSITY_CONTROL_H_
#define _DYNAMIC_DENSITY_CONTROL_H_

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <khepera3_driver/UnicycleControlMsg.h>
#include <optitrack_driver/OptiTrackData.h>

#include <dyndensity_controller/displayerData.h>

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <string>

#include <Eigen/Dense>

#include <voronoi/VoronoiDiagramGenerator.h>
#include <CommunicationProtocol.h>
#include <dunavant/dunavant.hpp>

#define EPSILON	1
#define numIntegrationSamples 1000
#define dimension 2
#define PI 3.141592654

using namespace Eigen;

	float rho[9] = {-0.2223,   -0.4047,    0.4403,   -0.2680,    0.7809,   -0.4418,   -0.6960,   -0.5254,   -0.0337};
	float sigmaX[9] = {0.0342,    0.0900,    0.0700,    0.0320,    0.0379,    0.0975,    0.0713,    0.0458,    0.1723};
	float sigmaY[9] = {0.1517,    0.0290,    0.0370,    0.1261,    0.0801,   0.0511,    0.0680,    0.0685,    0.0182};
	float meanX[9] = {0.1571,    0.4408,    0.8767,    0.7221,    0.5133,    0.3119,    0.2786,    0.4622,    0.6268};
	float meanY[9] = {0.5768,    0.4808,    0.4743,    0.2558,    0.2732,    0.8576,    0.2076,    0.7952,    0.5528};
	float prior[9] = {0.1832,    0.0805,    0.0710,    0.1527,    0.0740,    0.1100,    0.1252,    0.0701,    0.1332};

struct MnCM{
	double m;
	double cm_x;
	double cm_y;	
	double mt;
	double cmt_x; //CM of time derivative of the distribution
	double cmt_y;
};

class DynamicDensityControl{

	public: 
		
		DynamicDensityControl(int startingID, int endingID, float x1, float x2, float y1, float y2, int refresh_rate, double R, double G, double B, double opaqueness,std::string shapeStr, float size, double functionR, double functionG, double functionB, double functionOpaqueness, double functionMarkerSize, double functionMin, double functionMax, int functionNumHor, int functionNumVer, double k, double k_linear, double k_angular, int logBool, std::string logPath, int controlChoice, int distributionChoice, double timeConstanta, int verbose, int visualizeDistribution, int visualizeCM, int visualizeVoronoiCells, int UIControlled);

		~DynamicDensityControl();
		void loop();
	
	private:
	    
	    void viconCallback(const optitrack_driver::OptiTrackData &vdata); // Called when new vicon data is recieved

	    std::vector<MnCM> computeMassAndCenterOfMass(std::vector<std::vector <PointVDG> > orderedList, float* xValues, float* yValues);//called from viconCallback
	    std::vector<PointVDG> computeControl(std::vector<MnCM> massAndCM, float *xValues, float *yValues, float *thetaValues);

	    void generatePCL(float x1, float x2, float y1, float y2, int numHor, int numVer); // Sends RVIZ visualization data for density functions
	    void sendControl(std::vector<PointVDG> control, float *xValues, float *yValues);

	    void phiHardCoded(double x, double y, double &functionValue, double &derivativeValue);
	    void phiReceived(double x, double y, double &functionValue, double &derivativeValue);

	    void visualizeOthers(std::vector<MnCM> massAndCM);
	    void visualizeVoronoi();

    //	ros::NodeHandle n;
	    ros::NodeHandle mNodeHandle;

	    ros::Subscriber mViconSubscriber;
	    ros::Publisher marker_pub, marker_array_pub;
	    std::vector<ros::Publisher>  mControlPublisher;

	    uint32_t shape;

	    VoronoiDiagramGenerator vdg; // Inefficient --> Need to fix for large teams of robots if demos slows down.

	    int startingID,endingID,refresh_rate,numSeeds;
	    double x1,x2,y1,y2,R,G,B,opaqueness,size;

	    double functionR, functionG, functionB, functionOpaqueness, functionMarkerSize, functionMin, functionMax;
	    int functionNumHor, functionNumVer;

	    double k, k_linear, k_angular;

	    int logBool;
	    FILE* logFile;
	
	    int verbose, visualizeDistribution, visualizeCM, visualizeVoronoiCells;

	    float *xValues, *yValues, *thetaValues;
	    bool *seedsRefreshed;

	    ros::Time startingTime;
	    ros::Time delayObserver;

	    MatrixXd dCvdp;
	    MatrixXd dCvdt;
	    MatrixXd p;
	    MatrixXd pdot;

	    int controlChoice, distributionChoice;
	    double a; //density function time constant

	    double cost;
	    double currentTime;

	    CommunicationProtocol communicator;
	    int UIControlled;
	    std::vector<struct Density> density;
	    std::vector<struct Density> previousDensity;
    //	struct Density *density;
    //	struct Density *previousDensity;
	    void receiveAllDensities();
	    void (DynamicDensityControl::*phi)(double x, double y, double &functionValue, double &derivativeValue);
	    ros::Time previousTime;
	    float xScale, yScale;

};

#endif // _DYNAMIC_DENSITY_CONTROL_H_
