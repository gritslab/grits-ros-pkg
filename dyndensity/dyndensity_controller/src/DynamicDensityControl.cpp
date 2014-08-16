#include "DynamicDensityControl.h"

DynamicDensityControl :: ~DynamicDensityControl(){
	delete []xValues;
	delete []yValues;
	delete []thetaValues;
	delete []seedsRefreshed;
}

DynamicDensityControl :: DynamicDensityControl(int startingID, int endingID, float x1, float x2, float y1, float y2, int refresh_rate, double R, double G, double B, double opaqueness,std::string shapeStr, float size, double functionR, double functionG, double functionB, double functionOpaqueness, double functionMarkerSize, double functionMin, double functionMax, int functionNumHor, int functionNumVer, double k, double k_linear, double k_angular, int logBool, std::string logPath, int controlChoice, int distributionChoice, double timeConstanta, int verbose, int visualizeDistribution, int visualizeCM, int visualizeVoronoiCells, int UIControlled){

	this->startingID = startingID;
	this->endingID = endingID;

	this->numSeeds = endingID - startingID + 1;

	//Coordinates for the voronoi seeds. Define them and then initialize them.
	xValues = new float[numSeeds];
	yValues = new float[numSeeds];
	thetaValues = new float[numSeeds];
	seedsRefreshed = new bool[numSeeds];

	for(int i = 0; i < numSeeds; i++){
		xValues[i] = 0;
		yValues[i] = 0;
		seedsRefreshed[i] = false;
	}

	this->x1 = x1;
	this->x2 = x2;
	this->y1 = y1;
	this->y2 = y2;

	this->refresh_rate = refresh_rate;

	this->R = R;
	this->G = G;
	this->B = B;
	this->opaqueness = opaqueness;

	this->size = size;

	this->functionR = functionR;
	this->functionG = functionG;
	this->functionB = functionB;
	this->functionOpaqueness = functionOpaqueness;
	this->functionMarkerSize = functionMarkerSize;
	this->functionMin = functionMin;
	this->functionMax = functionMax;
	this->functionNumHor = functionNumHor;
	this->functionNumVer = functionNumVer;

	if( shapeStr.compare("arrow") == 0 ){ shape = visualization_msgs::Marker::ARROW;}
	if( shapeStr.compare("cylinder") == 0 ){ shape = visualization_msgs::Marker::CYLINDER; this->size = this->size/2;}

	this->k = k;
	this->k_linear = k_linear;
	this->k_angular = k_angular;

	this->logBool = logBool;
	if (logBool) { logFile = fopen(logPath.c_str(), "w"); }

	this->controlChoice = controlChoice;
	this->distributionChoice = distributionChoice;
	this->a = timeConstanta/(2*PI); //makes a the period
	
	this->verbose = verbose;
	this->visualizeDistribution = visualizeDistribution;
	this->visualizeCM = visualizeCM;
	this->visualizeVoronoiCells = visualizeVoronoiCells;


	marker_pub = mNodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker_array_pub = mNodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
	mViconSubscriber = mNodeHandle.subscribe("/optitrack/data", 1, &DynamicDensityControl::viconCallback, this); // subscribe to Optitrack data without noise

	mControlPublisher.resize(numSeeds);
	for(int i = 1; i < numSeeds + 1; i ++){
		char temp[100];
		sprintf(temp,"/K%d/khepera3_send_control",i);
		mControlPublisher[i-1] = mNodeHandle.advertise<khepera3_driver::UnicycleControlInput>(temp, 1);

	}

	this->UIControlled = UIControlled;
	if (UIControlled) {
		phi = &DynamicDensityControl::phiReceived;
		communicator.UDPSetupServer();
	}
	else phi = &DynamicDensityControl::phiHardCoded;

	previousDensity.clear();

	startingTime = ros::Time::now();
	previousTime = ros::Time::now();

	xScale = fmin(abs(x1), abs(x2));
	yScale = fmin(abs(y1), abs(y2));
}

void DynamicDensityControl :: visualizeVoronoi(){
/*
	for (unsigned int seed = 0; seed < vdg.voronoiOrderedList.size(); seed++){
		visualization_msgs::Marker line_strip;
		line_strip.header.frame_id = "/my_frame";
		line_strip.header.stamp = ros::Time::now();
		line_strip.ns = "points_and_lines";
		line_strip.action = visualization_msgs::Marker::ADD;
		line_strip.pose.orientation.w = 1.0;
		line_strip.id = seed;
		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
		line_strip.scale.x = this->size;
		line_strip.color.r = this->R;
		line_strip.color.g = this->G;
		line_strip.color.b = this->B;
		line_strip.color.a = this->opaqueness;
		line_strip.lifetime = ros::Duration();

		for (unsigned int vertex = 0; vertex < vdg.voronoiOrderedList[seed].size(); vertex++){
			geometry_msgs::Point p;
			p.x = vdg.voronoiOrderedList[seed][vertex].x;
			p.y = vdg.voronoiOrderedList[seed][vertex].y;
			p.z = 0;

			line_strip.points.push_back(p);
		}

		geometry_msgs::Point loopPoint;
		loopPoint.x = vdg.voronoiOrderedList[seed][0].x;
		loopPoint.y = vdg.voronoiOrderedList[seed][0].y;
		loopPoint.z = 0;	

		line_strip.points.push_back(loopPoint);	


		marker_pub.publish(line_strip);
	}
*/
	visualization_msgs::Marker lineList;
	lineList.header.frame_id = "/my_frame";
	lineList.header.stamp = ros::Time::now();
	lineList.ns = "points_and_lines";
	lineList.action = visualization_msgs::Marker::ADD;
	lineList.pose.orientation.w = 1.0;
	lineList.id = 0;
	lineList.type = visualization_msgs::Marker::LINE_LIST;
	lineList.scale.x = this->size;
	lineList.color.r = this->R;
	lineList.color.g = this->G;
	lineList.color.b = this->B;
	lineList.color.a = this->opaqueness;
	lineList.lifetime = ros::Duration();

	for (unsigned int seed = 0; seed < vdg.voronoiOrderedList.size(); seed++){
		for (unsigned int vertex = 0; vertex < vdg.voronoiOrderedList[seed].size()-1; vertex++){
			geometry_msgs::Point p1;
			p1.x = vdg.voronoiOrderedList[seed][vertex].x;
			p1.y = vdg.voronoiOrderedList[seed][vertex].y;
			p1.z = 0.05;
			lineList.points.push_back(p1);

			geometry_msgs::Point p2;
			p2.x = vdg.voronoiOrderedList[seed][vertex+1].x;
			p2.y = vdg.voronoiOrderedList[seed][vertex+1].y;
			p2.z = 0.05;
			lineList.points.push_back(p2);
		}
		geometry_msgs::Point lastPoint;
		lastPoint.x = vdg.voronoiOrderedList[seed][vdg.voronoiOrderedList[seed].size()-1].x;
		lastPoint.y = vdg.voronoiOrderedList[seed][vdg.voronoiOrderedList[seed].size()-1].y;
		lastPoint.z = 0.05;	
		lineList.points.push_back(lastPoint);	

		geometry_msgs::Point firstPoint;
		firstPoint.x = vdg.voronoiOrderedList[seed][0].x;
		firstPoint.y = vdg.voronoiOrderedList[seed][0].y;
		firstPoint.z = 0.05;	
		lineList.points.push_back(firstPoint);	

		
	}
	marker_pub.publish(lineList);	
}

void DynamicDensityControl :: visualizeOthers(std::vector<MnCM> massAndCM){

	visualization_msgs::MarkerArray markers;

	for (int seed = 0; seed < numSeeds; seed++){
		//visualization for centroid of each Voronoi cells
		visualization_msgs::Marker markerCM;
		markerCM.header.frame_id = "/my_frame";
		markerCM.header.stamp = ros::Time::now();
		markerCM.ns = "centroid_visualizer";
		markerCM.action = visualization_msgs::Marker::ADD;
		markerCM.id = seed;
		markerCM.type = visualization_msgs::Marker::CYLINDER;

		markerCM.pose.position.x = massAndCM[seed].cm_x;
		markerCM.pose.position.y = massAndCM[seed].cm_y;
		markerCM.pose.position.z = 0.01;
		markerCM.pose.orientation.w = 0.0;

		markerCM.scale.x = 0.12;
		markerCM.scale.y = 0.12;
		markerCM.scale.z = 0.01;

		markerCM.color.r = 0.4;
		markerCM.color.g = 0.4;
		markerCM.color.b = 1;
		markerCM.color.a = functionOpaqueness;

		markers.markers.push_back(markerCM);

		//visualization for the control direction for each agent
		visualization_msgs::Marker markerControl;
		markerControl.header.frame_id = "/my_frame";
		markerControl.header.stamp = ros::Time::now();
		markerControl.ns = "control_visualizer";
		markerControl.action = visualization_msgs::Marker::ADD;
		markerControl.id = seed;
		markerControl.type = visualization_msgs::Marker::ARROW;

		markerControl.pose.position.x = xValues[seed];
		markerControl.pose.position.y = yValues[seed];
		markerControl.pose.position.z = 0.00;

		float theta = atan2(pdot(dimension*seed+1,0), pdot(dimension*seed,0));

		markerControl.pose.orientation.x = cos(theta/2);
		markerControl.pose.orientation.y = sin(theta/2);
		markerControl.pose.orientation.z = 0.0;
		markerControl.pose.orientation.w = 0.0;

		markerControl.scale.x = 0.25;
		markerControl.scale.y = 0.25;
		markerControl.scale.z = 0.01;

		markerControl.color.r = 0.3;
		markerControl.color.g = 0.3;
		markerControl.color.b = 0.3;
		markerControl.color.a = functionOpaqueness;

		markers.markers.push_back(markerControl);
	}	

	marker_array_pub.publish(markers);

}

void DynamicDensityControl :: receiveAllDensities(){
/*
	do{
		struct Density newDensity;
		communicator.receiveDensity(newDensity);
		bool newDensityQualified = true;

		if (newDensity.type == DensityTypeGaussian){ 
			//check if this density is sane, i.e. not a duplicate reading of the same finger.

			for (int i = 0; i < (int)density.size(); i++){
				if (density[i].type == DensityTypeGaussian){
std::cout << "previous density is gaussian!\n";	
					if ( (density[i].parameters.gaussianParameters.meanX - newDensity.parameters.gaussianParameters.meanX)*(density[i].parameters.gaussianParameters.meanX - newDensity.parameters.gaussianParameters.meanX) + (density[i].parameters.gaussianParameters.meanY - newDensity.parameters.gaussianParameters.meanY)*(density[i].parameters.gaussianParameters.meanY - newDensity.parameters.gaussianParameters.meanY) < EPSILON){ //if the distance between the center of this gaussian and any other gaussian is less than EPSILON
std::cout << "violated!\n";
						newDensityQualified = false;
						break;
					}
				}
			}
			
			if (newDensityQualified){
				newDensity.parameters.gaussianParameters.meanX *= xScale;
				newDensity.parameters.gaussianParameters.meanY *= yScale;	
				newDensity.parameters.gaussianParameters.stdX *= xScale;
				newDensity.parameters.gaussianParameters.stdY *= yScale;
			}
		}

		if (newDensityQualified)	density.push_back(newDensity);

	}
	while(density.back().type != DensityTypeEnd);

	for (int i = 0; i < (int)density.size()-1; i++){
		printf("Type: %d, parameters: %f, %f, %f, %f, %f\n", density[i].type, density[i].parameters.gaussianParameters.meanX, density[i].parameters.gaussianParameters.meanY, density[i].parameters.gaussianParameters.correlation, density[i].parameters.gaussianParameters.stdX, density[i].parameters.gaussianParameters.stdY);
	}
*/

	struct Density newDensity[PACKET_LENGTH];
	communicator.receiveDensity(newDensity); //get all the densities, including the empty ones

	for (int i = 0; i < PACKET_LENGTH; i++){
		if (newDensity[i].type == DensityTypeEnd) break; //if that was the end of the densities, break out.

		if (newDensity[i].type == DensityTypeGaussian){
			newDensity[i].parameters.gaussianParameters.meanX *= xScale;
			newDensity[i].parameters.gaussianParameters.meanY *= yScale;	
			newDensity[i].parameters.gaussianParameters.stdX *= xScale;
			newDensity[i].parameters.gaussianParameters.stdY *= yScale;
		}

		density.push_back(newDensity[i]); //if this isnt an empty density structure, add it.
	}

	
}

void DynamicDensityControl :: phiReceived(double x, double y, double &functionValue, double &derivativeValue){

	//figure out the time to numerically compute the time derivative
	ros::Time time = ros::Time::now();
	double t = previousTime.toSec()-time.toSec();
	functionValue = 0;
	for (int i = 0; i < (int)density.size(); i++){ //size-1 to ignore the last one because it holds no info
		if (density[i].type == DensityTypeGaussian){
			functionValue += exp(-1./2./(1.-density[i].parameters.gaussianParameters.correlation*density[i].parameters.gaussianParameters.correlation) *( (x-density[i].parameters.gaussianParameters.meanX)*(x-density[i].parameters.gaussianParameters.meanX)/density[i].parameters.gaussianParameters.stdX/density[i].parameters.gaussianParameters.stdX + (y-density[i].parameters.gaussianParameters.meanY)*(y-density[i].parameters.gaussianParameters.meanY)/density[i].parameters.gaussianParameters.stdY/density[i].parameters.gaussianParameters.stdY) - 2.*density[i].parameters.gaussianParameters.correlation*(x-density[i].parameters.gaussianParameters.meanX)*(y-density[i].parameters.gaussianParameters.meanY)/density[i].parameters.gaussianParameters.stdX/density[i].parameters.gaussianParameters.stdY );
		}
//		else {
//			std::cout << "WARNING: Unrecognized density function.\n";
//		}
	}
	derivativeValue = functionValue;
	for (int i = 0; i < (int)previousDensity.size(); i++){
		if (previousDensity[i].type == DensityTypeGaussian){
			derivativeValue -= exp(-1./2./(1.-previousDensity[i].parameters.gaussianParameters.correlation*previousDensity[i].parameters.gaussianParameters.correlation) *( (x-previousDensity[i].parameters.gaussianParameters.meanX)*(x-previousDensity[i].parameters.gaussianParameters.meanX)/previousDensity[i].parameters.gaussianParameters.stdX/previousDensity[i].parameters.gaussianParameters.stdX + (y-previousDensity[i].parameters.gaussianParameters.meanY)*(y-previousDensity[i].parameters.gaussianParameters.meanY)/previousDensity[i].parameters.gaussianParameters.stdY/previousDensity[i].parameters.gaussianParameters.stdY) - 2.*previousDensity[i].parameters.gaussianParameters.correlation*(x-previousDensity[i].parameters.gaussianParameters.meanX)*(y-previousDensity[i].parameters.gaussianParameters.meanY)/previousDensity[i].parameters.gaussianParameters.stdX/previousDensity[i].parameters.gaussianParameters.stdY );
		}
	}
//	derivativeValue /= 5*t;

}

void DynamicDensityControl :: phiHardCoded(double x, double y, double &functionValue, double &derivativeValue){

	ros::Time time = ros::Time::now();
	double t = time.toSec()-startingTime.toSec();

	if (distributionChoice == 1){
		functionValue = exp( -( (x-2*sin(t/a))*(x-2*sin(t/a)) + (y/4)*(y/4) ) );
		derivativeValue = ( (4*cos(t/a)*(x - 2*sin(t/a)))/(a*exp((x - 2*sin(t/a))*(x - 2*sin(t/a)) + (y*y)/16)) );
	}
	else if (distributionChoice == 2){
		functionValue = exp( -( (x-2*sin(t/a))*(x-2*sin(t/a))+(y*y) ) );
		derivativeValue = (4*cos(t/a)*(x - 2*sin(t/a)))/(a*exp((x - 2*sin(t/a))*(x - 2*sin(t/a)) + (y*y) ));
	}
	else if (distributionChoice == 3){
		functionValue = 1/exp((y + sin((2*t)/a))*(y + sin((2*t)/a)) + (x - sin(t/a))*(x - sin(t/a)));

		derivativeValue = ((2*cos(t/a)*(x - sin(t/a)))/a - (4*cos((2*t)/a)*(y + sin((2*t)/a)))/a)/exp((y + sin((2*t)/a))*(y + sin((2*t)/a)) + (x - sin(t/a))*(x - sin(t/a)));

//		functionValue = 1/exp((x - 2*sin(t/a))*(x - 2*sin(t/a)) + (y + 2*sin((2*t)/a))*(y + 2*sin((2*t)/a)));
//		derivativeValue = ((4*cos(t/a)*(x - 2*sin(t/a)))/a - (8*cos((2*t)/a)*(y + 2*sin((2*t)/a)))/a)/exp((x - 2*sin(t/a))*(x - 2*sin(t/a)) + (y + 2*sin((2*t)/a))*(y + 2*sin((2*t)/a)));
	}

	else if (distributionChoice == 4){
		functionValue = 1/exp((x - cos(t/a))*(x - cos(t/a)) + (y - sin(t/a))*(y - sin(t/a)));
		derivativeValue = -((2*sin(t/a)*(x - cos(t/a)))/a - (2*cos(t/a)*(y - sin(t/a)))/a)/exp((x - cos(t/a))*(x - cos(t/a)) + (y - sin(t/a))*(y - sin(t/a)));
	}
	else if (distributionChoice == 5){
		functionValue = 1/exp((x - cos(t/a))*(x - cos(t/a)) + (y - sin(t/a))*(y - sin(t/a))) + 1/exp((x + cos(t/a))*(x + cos(t/a)) + (y + sin(t/a))*(y + sin(t/a)));
		double temp1 = ((2*sin(t/a)*(x + cos(t/a)))/a - (2*cos(t/a)*(y + sin(t/a)))/a);
                double temp2 = exp((x + cos(t/a))*(x + cos(t/a)) + (y + sin(t/a))*(y + sin(t/a)));
                double temp3 = ((2*sin(t/a)*(x - cos(t/a)))/a - (2*cos(t/a)*(y - sin(t/a)))/a);
                double temp4 = exp((x - cos(t/a))*(x - cos(t/a)) + (y - sin(t/a))*(y - sin(t/a)));
                derivativeValue = temp1/temp2 - temp3/temp4;		
	}
	else if (distributionChoice == 6){

		functionValue = 0;

		for (int i = 0; i < 9; i++){
			functionValue += prior[i]/(2*PI*sigmaX[i]*sigmaY[i]*sqrt(1-rho[i]*rho[i])) * exp( -1/(2*(1-rho[i]*rho[i])) * ( (x-meanX[i])*(x-meanX[i])/sigmaX[i]/sigmaX[i] + (y-meanY[i])*(y-meanY[i])/sigmaY[i]/sigmaY[i] - 2*rho[i]*(x-meanX[i])*(y-meanY[i])/sigmaX[i]/sigmaY[i] ) );
			//1/exp((x - cos(t/a))*(x - cos(t/a)) + (y - sin(t/a))*(y - sin(t/a))) + 1/exp((x + cos(t/a))*(x + cos(t/a)) + (y + sin(t/a))*(y + sin(t/a)));
		}


		double temp1 = ((2*sin(t/a)*(x + cos(t/a)))/a - (2*cos(t/a)*(y + sin(t/a)))/a);
                double temp2 = exp((x + cos(t/a))*(x + cos(t/a)) + (y + sin(t/a))*(y + sin(t/a)));
                double temp3 = ((2*sin(t/a)*(x - cos(t/a)))/a - (2*cos(t/a)*(y - sin(t/a)))/a);
                double temp4 = exp((x - cos(t/a))*(x - cos(t/a)) + (y - sin(t/a))*(y - sin(t/a)));
                derivativeValue = temp1/temp2 - temp3/temp4;		
	}
	else {
		std::cout << "Wrong density!";
		exit(0);
	}
}

void DynamicDensityControl :: sendControl(std::vector<PointVDG> control, float *xValues, float *yValues){

	for(unsigned int i = 0; i < control.size(); i++){


		khepera3_driver::UnicycleControlInput mControlMsg;
		mControlMsg.linear_velocity = control[i].x; //really bad names, but these are linear and angular velocities...
		mControlMsg.angular_velocity = control[i].y;

		mControlPublisher[i].publish(mControlMsg);

	}

}

std::vector<PointVDG> DynamicDensityControl :: computeControl(std::vector<MnCM> massAndCM, float *xValues, float *yValues, float *thetaValues){
	//in this function, the control is computed

	std::vector<PointVDG> gradient( massAndCM.size() );
	std::vector<PointVDG> control( massAndCM.size() );
	static double lastTime = ros::Time::now().toSec();
	ros::Time time = ros::Time::now();
	currentTime = time.toSec();

	if (controlChoice == 1){ //Lloyd
		pdot = MatrixXd::Constant(dimension*massAndCM.size(),1,0);
		
		for (unsigned int seed = 0; seed < massAndCM.size(); seed++){
			gradient[seed].x = k * (massAndCM[seed].cm_x - xValues[seed]);
			gradient[seed].y = k * (massAndCM[seed].cm_y - yValues[seed]);
			if (verbose) std::cout << "Gradient for seed " << seed << ": ( " << gradient[seed].x << " , " << gradient[seed].y << " )" << "\n";

			double norm = sqrt( (gradient[seed].x)*(gradient[seed].x) + (gradient[seed].y)*(gradient[seed].y) );		
			control[seed].x = norm;
			control[seed].y = (-sin(thetaValues[seed])*gradient[seed].x + cos(thetaValues[seed])*gradient[seed].y) / norm;

			control[seed].x *= k_linear;
			control[seed].y *= k_angular;

			pdot(dimension*seed  ,0) = gradient[seed].x;
			pdot(dimension*seed+1,0) = gradient[seed].y;

		}
	}
	else if (controlChoice == 2){ //Cortes
		pdot = MatrixXd::Constant(dimension*massAndCM.size(),1,0);

		for (unsigned int seed = 0; seed < massAndCM.size(); seed++){
			gradient[seed].x = (massAndCM[seed].cmt_x*massAndCM[seed].mt - massAndCM[seed].mt*massAndCM[seed].cm_x)/massAndCM[seed].m - (k + massAndCM[seed].mt/massAndCM[seed].m) * (xValues[seed] - massAndCM[seed].cm_x);
			gradient[seed].y = (massAndCM[seed].cmt_y*massAndCM[seed].mt - massAndCM[seed].mt*massAndCM[seed].cm_y)/massAndCM[seed].m - (k + massAndCM[seed].mt/massAndCM[seed].m) * (yValues[seed] - massAndCM[seed].cm_y);
			if (verbose) std::cout << "Gradient for seed " << seed << ": ( " << gradient[seed].x << " , " << gradient[seed].y << " )" << "\n";

			double norm = sqrt( (gradient[seed].x)*(gradient[seed].x) + (gradient[seed].y)*(gradient[seed].y) );		
			control[seed].x = norm;
			control[seed].y = (-sin(thetaValues[seed])*gradient[seed].x + cos(thetaValues[seed])*gradient[seed].y) / norm;

			control[seed].x *= k_linear;
			control[seed].y *= k_angular;

			pdot(dimension*seed  ,0) = gradient[seed].x;
			pdot(dimension*seed+1,0) = gradient[seed].y;

		}


	}
	else if (controlChoice == 3){//our algorithm
		//constructing necessary matrices and vectors
		p = MatrixXd::Constant(dimension*massAndCM.size(),1,0);
		pdot = MatrixXd::Constant(dimension*massAndCM.size(),1,0);

		MatrixXd c = MatrixXd::Constant(dimension*massAndCM.size(),1,0);
		for (unsigned int seed = 0; seed < massAndCM.size(); seed++){
			p(dimension*seed  ,0) = xValues[seed];
			p(dimension*seed+1,0) = yValues[seed];
			c(dimension*seed  ,0) = massAndCM[seed].cm_x;
			c(dimension*seed+1,0) = massAndCM[seed].cm_y;
		}

		MatrixXd firstTerm = MatrixXd::Identity(dimension*massAndCM.size(), dimension*massAndCM.size()) - dCvdp;
		firstTerm = firstTerm.inverse();

		pdot = firstTerm * ( -k*(p-c) + dCvdt );

	//	std::cout << "pdot =" << std::endl << pdot << std::endl;
	
	//	std::cout << "p =" << std::endl << p << std::endl;
	//	std::cout << "c =" << std::endl << c << std::endl;
	//	std::cout << "firstTerm =" << std::endl << firstTerm << std::endl;
	
	//	std::cout << "dCvdp =" << std::endl << dCvdp << std::endl;
	//	std::cout << "dCvdt =" << std::endl << dCvdt << std::endl;

		for (unsigned int seed = 0; seed < massAndCM.size(); seed++){

			double norm = sqrt( pdot(dimension*seed)*pdot(dimension*seed) + pdot(dimension*seed+1)*pdot(dimension*seed+1) );		
			control[seed].x = norm;
			control[seed].y = (-sin(thetaValues[seed])*pdot(dimension*seed) + cos(thetaValues[seed])*pdot(dimension*seed+1)) / norm;

			control[seed].x *= k_linear;
			control[seed].y *= k_angular;

			if (verbose) std::cout << "Control for agent " << seed << ": ( " << control[seed].x << " , " << control[seed].y << " )" << "\n";
	
		}
	}
	else if (controlChoice == 4){//first order Neumann approx to our algorithm
		//constructing necessary matrices and vectors
		p = MatrixXd::Constant(dimension*massAndCM.size(),1,0);
		pdot = MatrixXd::Constant(dimension*massAndCM.size(),1,0);

		MatrixXd c = MatrixXd::Constant(dimension*massAndCM.size(),1,0);
		for (unsigned int seed = 0; seed < massAndCM.size(); seed++){
			p(dimension*seed  ,0) = xValues[seed];
			p(dimension*seed+1,0) = yValues[seed];
			c(dimension*seed  ,0) = massAndCM[seed].cm_x;
			c(dimension*seed+1,0) = massAndCM[seed].cm_y;
		}

//		MatrixXd firstTerm = MatrixXd::Identity(dimension*massAndCM.size(), dimension*massAndCM.size()) - dCvdp;
//		firstTerm = firstTerm.inverse();

		MatrixXd firstTerm = MatrixXd::Identity(dimension*massAndCM.size(), dimension*massAndCM.size()) + dCvdp;
		
		pdot = firstTerm * ( -k*(p-c) + dCvdt );

	//	std::cout << "pdot =" << std::endl << pdot << std::endl;
	
	//	std::cout << "p =" << std::endl << p << std::endl;
	//	std::cout << "c =" << std::endl << c << std::endl;
	//	std::cout << "firstTerm =" << std::endl << firstTerm << std::endl;
	
	//	std::cout << "dCvdp =" << std::endl << dCvdp << std::endl;
	//	std::cout << "dCvdt =" << std::endl << dCvdt << std::endl;

		for (unsigned int seed = 0; seed < massAndCM.size(); seed++){

			double norm = sqrt( pdot(dimension*seed)*pdot(dimension*seed) + pdot(dimension*seed+1)*pdot(dimension*seed+1) );		
			control[seed].x = norm;
			control[seed].y = (-sin(thetaValues[seed])*pdot(dimension*seed) + cos(thetaValues[seed])*pdot(dimension*seed+1)) / norm;

			control[seed].x *= k_linear;
			control[seed].y *= k_angular;

			if (verbose) std::cout << "Control for agent " << seed << ": ( " << control[seed].x << " , " << control[seed].y << " )" << "\n";

	
		}
	}
	else{
		std::cout << "Which control?? \n";
		exit(0);
	}


	lastTime = currentTime;
	return control;
}

void DynamicDensityControl :: generatePCL(float x1, float x2, float y1, float y2, int numHor, int numVer){
	float x, y;
/*	
	visualization_msgs::MarkerArray markers;

//	marker.color.r = functionValue/functionMax;
//	marker.color.g = functionG;
//	marker.color.b = functionB;
//	marker.color.a = functionOpaqueness;

//			marker.lifetime = ros::Duration();

	int ID = 0;

	for(int hor = 0; hor < numHor; hor++){
		x = x1 + (x2-x1) * ( (float)hor / (float)numHor );
		for(int ver = 0; ver < numVer; ver++){
			y = y1 + (y2-y1) * ( (float)ver / (float)numVer );

			visualization_msgs::Marker marker;

			marker.header.frame_id = "/my_frame";
			marker.header.stamp = ros::Time::now();

			// Any marker sent with the same namespace and id will overwrite the old one
			marker.ns = "function_visualizer";
			marker.action = visualization_msgs::Marker::ADD;
			marker.id = ID;

			marker.type = visualization_msgs::Marker::POINTS;

			// Set the marker action.  Options are ADD and DELETE
	


			// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = -0.01;

			marker.pose.orientation.w = 0.0;

			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker.scale.x = functionMarkerSize;
			marker.scale.y = functionMarkerSize;
			marker.scale.z = 0.01;

			double functionValue, trash;
			phi(x, y, functionValue, trash);


			marker.color.r = functionValue/functionMax;
			marker.color.g = functionG;
			marker.color.b = functionB;
			marker.color.a = functionOpaqueness;
			
			markers.markers.push_back(marker);
			ID++;
		}
	}
	// Publish the marker
	marker_pub.publish(markers);
*/
	visualization_msgs::Marker points;

	points.header.frame_id = "/my_frame";
	points.header.stamp = ros::Time::now();
	points.ns = "function_visualizer";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.x = 0;
	points.pose.orientation.y = 0;
	points.pose.orientation.z = 0.0;
	points.pose.orientation.w = 0.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = functionMarkerSize;
	points.scale.y = functionMarkerSize;
	points.color.r = 1;
	points.color.g = functionG;
	points.color.b = functionB;
	points.color.a = functionOpaqueness;

	for(int hor = 0; hor < numHor; hor++){
		x = x1 + (x2-x1) * ( (float)hor / (float)numHor );
		for(int ver = 0; ver < numVer; ver++){
			y = y1 + (y2-y1) * ( (float)ver / (float)numVer );

			double functionValue, trash;
			(this->*phi)(x, y, functionValue, trash);

			geometry_msgs::Point p;
			p.x = x;
			p.y = y;
			p.z = -0.02;
			points.points.push_back(p);

			std_msgs::ColorRGBA color;
			color.r = functionValue/functionMax;
			color.g = functionG;
			color.b = functionB;
			color.a = functionOpaqueness;
			points.colors.push_back(color);
		}
	}
	// Publish the marker
	marker_pub.publish(points);
}



std::vector<MnCM> DynamicDensityControl :: computeMassAndCenterOfMass(std::vector<std::vector <PointVDG> > orderedList, float* xValues, float* yValues){

# define NODE_NUM 3

	dCvdp = MatrixXd::Constant(dimension*orderedList.size(),dimension*orderedList.size(),0); //initializing
	dCvdt = MatrixXd::Constant(dimension*orderedList.size(),1,0);
	
	cost = 0; //numerically integrate to find the cost

	//struct that stores computed control, this is going to be returned
	std::vector<MnCM> massAndCM( orderedList.size() );
	
	//looping over all seeds
	for (unsigned int seed = 0; seed < orderedList.size(); seed++){

		//initialize the mass and center of mass storage structure for this seed
		massAndCM[seed].m = 0;
		massAndCM[seed].cm_x = 0;
		massAndCM[seed].cm_y = 0;

		//if this seed cannot make triangles anymore, we quit and go to the next seed.
		while(orderedList[seed].size() >= 3){

			//triangles exist! Let's compute mass and center of mass

			//starting numerical integration
			double area, area2;
//			int i, node;
			double node_xy[2*NODE_NUM] = {
			0.0, 0.0,
			1.0, 0.0,
			0.0, 1.0 };
			double node_xy2[2*NODE_NUM];
			node_xy2[0] = orderedList[seed][0].x; node_xy2[1] = orderedList[seed][0].y;
			node_xy2[2] = orderedList[seed][1].x; node_xy2[3] = orderedList[seed][1].y;
			node_xy2[4] = orderedList[seed][2].x; node_xy2[5] = orderedList[seed][2].y;

			int order, order_num;
//			int point_show = 2;
			int rule;
			double *w;
			double *xy;
			double *xy2;

			rule = 10;

			order_num = dunavant_order_num ( rule );

			xy = new double[2*order_num];
			xy2 = new double[2*order_num];
			w = new double[order_num];

			dunavant_rule ( rule, order_num, xy, w );
			area = triangle_area ( node_xy );
			reference_to_physical_t3 ( node_xy2, order_num, xy, xy2 );
			area2 = triangle_area ( node_xy2 );

			double value,derivative, x, y, forM, forCM_x, forCM_y, forMt, forCMt_x, forCMt_y, forCost;

			forM = 0; forCM_x = 0; forCM_y = 0; forMt = 0; forCMt_x = 0; forCMt_y = 0, forCost = 0;
			for ( order = 0; order < order_num; order++ )
			{
				x = xy2[0+order*2];
				y = xy2[1+order*2];

				//function specification here
//				value = phi(x,y);

				(this->*phi)(x,y,value,derivative);
				forM =    forM +    w[order] * value;
				forCM_x = forCM_x + w[order] * value * x;
				forCM_y = forCM_y + w[order] * value * y;
				forMt =    forMt +    w[order] * derivative;
				forCMt_x = forCMt_x + w[order] * derivative * x;
				forCMt_y = forCMt_y + w[order] * derivative * y;
				forCost = forCost + w[order] * value * ( (x-xValues[seed])*(x-xValues[seed]) + (y-yValues[seed])*(y-yValues[seed]) );
			}
			massAndCM[seed].m += area2 * forM;
			massAndCM[seed].cm_x += area2 * forCM_x; 
			massAndCM[seed].cm_y += area2 * forCM_y;
			massAndCM[seed].mt += area2 * forMt; //mass of the time derivative of the distribution
			massAndCM[seed].cmt_x += area2 * forCMt_x; //CM the time derivative of the distribution
			massAndCM[seed].cmt_y += area2 * forCMt_y;
			cost += area2 * forCost;

//			quad = area2 * quad;
			//	cout << "Calculated : " << quad << "\n";

			//numerical are integration done. Cleaning up for this round.
			delete [] w;
			delete [] xy;
			delete [] xy2;

			//removing the 2nd element in the list of vertices for this seed.
			orderedList[seed].erase( orderedList[seed].begin()+1 );

		}

		//division by the mass of the voronoi region to make these variables represent actual CM.
		massAndCM[seed].cm_x /= massAndCM[seed].m;
		massAndCM[seed].cm_y /= massAndCM[seed].m;
		massAndCM[seed].cmt_x /= massAndCM[seed].mt;
		massAndCM[seed].cmt_y /= massAndCM[seed].mt;

		if (verbose){
			std::cout << "Mass for region " << seed << ": " << massAndCM[seed].m << "\n";
			std::cout << "Center of mass for region " << seed << ": ( " << massAndCM[seed].cm_x << " , " << massAndCM[seed].cm_y << " )" << "\n";
		}
	

		double M = massAndCM[seed].m;
		double Mt = massAndCM[seed].mt;
		Vector2d CM(massAndCM[seed].cm_x, massAndCM[seed].cm_y);
		Vector2d CMt(massAndCM[seed].cmt_x, massAndCM[seed].cmt_y);

		dCvdt.block<dimension,1>(dimension*seed,0) = ( M*CMt*Mt - Mt*CM*M ) / ( M*M );
//		dCvdt.block<dimension,1>(dimension*seed,1) = ( CMt - CM )* Mt / M ; //more efficient i guess

		//starting numerical path integration <><><><><><><><><><><><>
		
		for(unsigned int neighbor = 0; neighbor < orderedList.size(); neighbor++){
			std::vector<PointVDG> adjacentVertices = vdg.checkAdjacency(seed,neighbor);
			if( adjacentVertices.size() > 1){ //if the list is non-empty, meaning they are neighbors, we compute the stuff here
				
				Vector2d seedAgent(xValues[seed],yValues[seed]);
				Vector2d neighborAgent(xValues[neighbor],yValues[neighbor]);
				Vector2d vertex1(adjacentVertices[0].x,adjacentVertices[0].y);
				Vector2d vertex2(adjacentVertices[1].x,adjacentVertices[1].y);
				Vector2d slope = vertex2 - vertex1;
				Vector2d q(0,0); //q is the integration dummy variable

				float normDistancebetweenAgents = sqrt( (xValues[seed]-xValues[neighbor])*(xValues[seed]-xValues[neighbor]) + (yValues[seed]-yValues[neighbor])*(yValues[seed]-yValues[neighbor]) );
				float dPartition = normDistancebetweenAgents / numIntegrationSamples;


				//looping for the nasty nature of this integral...leave the cm vector as it is, take the p vector component by component and compute partials 
				for(int dim = 0; dim < dimension; dim++){

					//path integrands
					Vector2d integrand1(0,0);
					double integrand2 = 0;
					Vector2d integrand3(0,0);
					double integrand4 = 0;

					for(int t = 0; t <= numIntegrationSamples; t++){
						q = vertex1 + slope*t/numIntegrationSamples;

						double phiValue, trash;
						(this->*phi)(q(0), q(1), phiValue, trash); //phiValue has phi(x,y) now.

						// dci/dpi first
						integrand1 += phiValue * q * ( neighborAgent(dim)-q(dim) ); //integrand and q both vectors
						integrand2 += phiValue * ( neighborAgent(dim)-q(dim) ); //integrand scalar

						// dci/dpi
						integrand3 += phiValue * q * ( q(dim)-seedAgent(dim) ); //integrand and q both vectors
						integrand4 += phiValue * ( q(dim)-seedAgent(dim) ); //integral scalar
				
					}
					Vector2d integral1 = integrand1 / normDistancebetweenAgents * dPartition;
					double integral2 = integrand2 / normDistancebetweenAgents * dPartition;
					Vector2d integral3 = integrand3 / normDistancebetweenAgents * dPartition;
					double integral4 = integrand4 / normDistancebetweenAgents * dPartition;


					dCvdp.block<dimension,1>(dimension*seed,dimension*neighbor+dim) = (integral1 - integral2*CM)/M;
					dCvdp.block<dimension,1>(dimension*seed,dimension*seed+dim) += (integral3 - integral4*CM)/M;
				}
			}
		}
//		std::cout << "dCvdp =" << std::endl << dCvdp << std::endl;
//		std::cout << "dCvdt =" << std::endl << dCvdt << std::endl;
	}
	

	return massAndCM;
# undef NODE_NUM
}	


void DynamicDensityControl :: viconCallback(const optitrack_driver::OptiTrackData &vdata){
//	printf("Inside viconCallback\n");
	float X, Y, Z, theta;
	X = vdata.position.x;
	Y = vdata.position.y;
	Z = vdata.position.z;
	theta = vdata.orientation.z;

	if( ( vdata.id <= this->endingID ) && ( this->startingID <= vdata.id ) ){
		xValues[vdata.id-startingID] = X;
		yValues[vdata.id-startingID] = Y;
		thetaValues[vdata.id-startingID] = theta;
		seedsRefreshed[vdata.id-startingID] = true;
//		printf("%dth agent's position refreshed to (%f,%f)\n",xValues[vdata.id-startingID],yValues[vdata.id-startingID]);
	}	

	//if any of the coordinates for the seeds are not updated, then we dont re-draw the voronoi region.
	bool go = true;
/*	for(int i = 0; i < numSeeds; i++){
		if(!seedsRefreshed[i]){
			go = false;
			break;
		}
	}*/

	if(go){
std::cout << "Other delays: " << ros::Time::now().toSec() - delayObserver.toSec();

		if (UIControlled) {
			std::cout << "Ready to receive densities: " << std::endl;
			receiveAllDensities();
std::cout << "UDP delay: " << ros::Time::now().toSec() - delayObserver.toSec() << std::endl;
		}
//		std::cout << "UIControlled value: " << UIControlled << std::endl;

		ros::Time algorithmDuration = ros::Time::now();
		//compute the Voronoi region.
		vdg.findOrderedList(x1,x2,y1,y2, xValues,yValues,numSeeds,0);
std::cout << "Voronoi partition computation delay: " << ros::Time::now().toSec() - delayObserver.toSec() << std::endl;

		// Computing the Mass and Center of Mass for each Voronoi Cell
		std::vector<MnCM> massAndCM = computeMassAndCenterOfMass(vdg.voronoiOrderedList,xValues,yValues);
std::cout << "CM, M computation delay: " << ros::Time::now().toSec() - delayObserver.toSec() << std::endl;

		// Computing the Control for each Agent
		std::vector<PointVDG> control = computeControl(massAndCM, xValues, yValues, thetaValues);
std::cout << "Control computation delay: " << ros::Time::now().toSec() - delayObserver.toSec() << std::endl;

		// Send the control to each agent
		sendControl(control,xValues,yValues);
std::cout << "Control sending delay: " << ros::Time::now().toSec() - delayObserver.toSec() << std::endl;

		// Visualize Voronoi Cells
		if (visualizeVoronoiCells) visualizeVoronoi();
std::cout << "main visualization delay: " << ros::Time::now().toSec() - delayObserver.toSec() << std::endl;

		if(logBool){ 
			fprintf(logFile,"%f, %f", currentTime-startingTime.toSec(), cost); 
			if ( (controlChoice == 3) || (controlChoice == 4) ){//if we're using our algorithm
				EigenSolver<MatrixXd> eigenSolver(dCvdp, false); //invoke the solver to get eigenvalues/eigenvectors
//				std::cout << "Eigenvalues of dcdp:\n";
//				std::cout << eigenSolver.eigenvalues() << std::endl; //peek at the eigenvalues
				VectorXcd eVals;
				eVals = eigenSolver.eigenvalues().cast< std::complex<double> >(); //retrieve the eigenvalues: they must be cast before retriving, which is what is done here
//				std::cout << "Eigenvalues of dcdp:\n";
//				std::cout << eVals << std::endl;
				VectorXd norms = VectorXd::Constant(eVals.size(), 0);
				for(int i = 0; i < eVals.size(); i++){
					norms[i] = std::norm(eVals[i]);
					fprintf(logFile,", %f", norms[i]);
//					std::cout << "At "<< i << ", the norm is " << norms[i] << "\n";
				}
//				std::cout << norms << std::endl; 
			}
			fprintf(logFile,"\n");
		} //logs the cost data along with time
		if(visualizeDistribution) generatePCL(x1, x2, y1, y2, functionNumHor, functionNumVer);
		
		if (visualizeCM) visualizeOthers(massAndCM);
std::cout << "other visualization delay: " << ros::Time::now().toSec() - delayObserver.toSec() << std::endl;

		//reset the flags for seed update.
		for(int i = 0; i < numSeeds; i++) seedsRefreshed[i] = false;
//		printf("All flags reset\n");

	ros::Time algorithmDuration2 = ros::Time::now();
	printf("Time for the algorithm to run: %f\n\n\n",algorithmDuration2.toSec()-algorithmDuration.toSec());

	delayObserver = ros::Time::now();

	previousDensity.clear();
	previousDensity = density;
	density.clear();

std::cout << "Density sizes: " << previousDensity.size() << " " << density.size() << std::endl;	

	previousTime = ros::Time::now();

	}
}

void DynamicDensityControl :: loop()
{
	ros::Rate r(refresh_rate);

	//main loop
	while (mNodeHandle.ok())
	{
		ros :: spin(); //makes calls to the callback if new data is recieved in the subscribed channels
		r.sleep();
	}
}

int main( int argc, char** argv )
{

	int startingID,endingID,refresh_rate;
	double x1,x2,y1,y2,R,G,B,opaqueness,size;
	std::string shapeStr;

	ros::init(argc, argv, "CVTControl");
	
	ros :: param :: get("~startingID",startingID);
	ros :: param :: get("~endingID",endingID);

	ros :: param :: get("~x1",x1);
	ros :: param :: get("~x2",x2);

	ros :: param :: get("~y1",y1);
	ros :: param :: get("~y2",y2);

	ros :: param :: get("~refresh_rate",refresh_rate);	

	ros :: param :: get("~R",R);
	ros :: param :: get("~G",G);
	ros :: param :: get("~B",B);
	ros :: param :: get("~opaqueness",opaqueness);
	ros :: param :: get("~shape",shapeStr);
	ros :: param :: get("~size",size);

	double functionR, functionG, functionB, functionOpaqueness, functionMarkerSize, functionMin, functionMax;
	ros :: param :: get("~functionR",functionR);
	ros :: param :: get("~functionG",functionG);
	ros :: param :: get("~functionB",functionB);
	ros :: param :: get("~functionOpaqueness",functionOpaqueness);
	ros :: param :: get("~functionMarkerSize",functionMarkerSize);
	ros :: param :: get("~functionMin",functionMin);
	ros :: param :: get("~functionMax",functionMax);

	int functionNumHor, functionNumVer;
	ros :: param :: get("~functionNumHor",functionNumHor);
	ros :: param :: get("~functionNumVer",functionNumVer);

	double k, k_linear, k_angular;
	ros :: param :: get("~k",k);
	ros :: param :: get("~k_linear",k_linear);
	ros :: param :: get("~k_angular",k_angular);

	int logBool;
	std::string logPath;
	ros :: param :: get("~logBool",logBool);
//	ros :: param :: get("~logPath",logPath);

	int controlChoice, distributionChoice;
	double timeConstanta;
	ros :: param :: get("~controlChoice",controlChoice);
	ros :: param :: get("~distributionChoice",distributionChoice);
	ros :: param :: get("~timeConstanta",timeConstanta);
		
	int verbose, visualizeDistribution, visualizeCM, visualizeVoronoiCells;
	ros :: param :: get("~verbose",verbose);
	ros :: param :: get("~visualizeDistribution",visualizeDistribution);
	ros :: param :: get("~visualizeCM",visualizeCM);
	ros :: param :: get("~visualizeVoronoiCells",visualizeVoronoiCells);

	int UIControlled;
	ros :: param :: get("~UIControlled",UIControlled);

	if (logBool){
		logPath += "/home/labuser/Desktop/data/";
		std::stringstream ss1;
		ss1 << (int)(timeConstanta+0.01);
		logPath += ss1.str();
 		logPath += "_d";
		std::stringstream ss2;
		ss2 << distributionChoice;
		logPath += ss2.str();
		logPath += "_";
		if (controlChoice == 1)		logPath += "lloyd";
		else if (controlChoice == 2)	logPath += "cortes";
		else if (controlChoice == 3)	logPath += "centralized";
		else if (controlChoice == 4)	logPath += "decentralized";

		logPath += ".txt";
	}


/*
std::vector<MatrixXd> mTest(2);
MatrixXd vTest;
MatrixXd vResult;

mTest[0] = MatrixXd::Constant(2, 2,1);
mTest[1] = MatrixXd::Constant(2, 2,2);
vTest = MatrixXd::Constant(2, 1,1);

std::cout << "mTest[0] =" << std::endl << mTest[0] << std::endl;
std::cout << "mTest[1] =" << std::endl << mTest[1] << std::endl;
std::cout << "vTest =" << std::endl << vTest << std::endl;

std::cout << "mTest[1](1,1) =" << std::endl << mTest[1](1,1) << std::endl;

vResult = (mTest[0] + mTest[1]) * vTest;

std::cout << "vResult =" << std::endl << vResult << std::endl;
*/
/*
MatrixXd mRandom;
mRandom = MatrixXd::Random(2, 2);
std::cout << "mRandom =" << std::endl << mRandom << std::endl;
mTest[1] = mRandom;
std::cout << "mTest[0] =" << std::endl << mTest[0] << std::endl;
std::cout << "mTest[1] =" << std::endl << mTest[1] << std::endl;
*/
/*
MatrixXd mRandom;
mRandom = MatrixXd::Constant(2, 2, 1);
std::cout << mRandom;
EigenSolver<MatrixXd> solver(mRandom, false);
std::cout << "wtf?\n";
std::cout << solver.eigenvalues()[1];
std::cout << "ftw?\n";
VectorXcd eVals;
eVals = solver.eigenvalues().cast< std::complex<double> >();
std::cout << eVals << "\n\n";
VectorXd norms = VectorXd::Constant(eVals.size(), 0);

std::cout << "The freaking size is: " << eVals.size() << "\n";

for(int i = 0; i < eVals.size(); i++){
	norms[i] = std::norm(eVals[i]);
	std::cout << "At "<< i << ", the norm is " << norms[i] << "\n";
}
std::cout << norms;
*/
//VectorXd dd;
//dd = solver.eigenvalues().cast<double>();
//dd = MatrixXd::Constant(1, 2, 0);
//dd = solver.eigenvalues();
/*
std::vector<MatrixXd> mTest(2);
mTest[0] = MatrixXd::Constant(5, 5, 0);
mTest[1] = MatrixXd::Constant(5, 5, 0);
std::cout << "mTest[0] =" << std::endl << mTest[0] << std::endl;
std::cout << "mTest[1] =" << std::endl << mTest[1] << std::endl;
mTest[0].block<dimension,1>(2,2) = MatrixXd::Constant(dimension, 1, 1);
mTest[1].block<dimension,1>(2,4) = MatrixXd::Constant(dimension, 1, 1);
std::cout << "mTest[0] =" << std::endl << mTest[0] << std::endl;
std::cout << "mTest[1] =" << std::endl << mTest[1] << std::endl;
mTest[0].block<dimension,1>(2,2) += MatrixXd::Constant(dimension, 1, 10);
std::cout << "mTest[0] =" << std::endl << mTest[0] << std::endl;
std::cout << "mTest[1] =" << std::endl << mTest[1] << std::endl;
*/

//mTest[1]
//std::cout << "block " << std::endl << mTest[0]<1,1>(0,0) << std::endl;


/*
MatrixXf m,m2,mInv;
m = MatrixXf::Random(3, 3);
m2 = MatrixXf::Random(3, 3);

std::cout << "m =" << std::endl << m << std::endl;
std::cout << "m2 =" << std::endl << m2 << std::endl;
std::cout << "m + m2 =" << std::endl << m+m2 << std::endl;
m += 2*m2;
std::cout << "m =" << std::endl << m << std::endl;
mInv = m.inverse();
std::cout << "mInv =" << std::endl << mInv << std::endl;
std::cout << "m*mInv =" << std::endl << m*mInv << std::endl;
*/

	DynamicDensityControl(startingID, endingID, x1, x2, y1, y2, refresh_rate, R, G, B, opaqueness, shapeStr, size, functionR, functionG, functionB, functionOpaqueness, functionMarkerSize, functionMin, functionMax, functionNumHor, functionNumVer, k, k_linear, k_angular, logBool, logPath, controlChoice, distributionChoice, timeConstanta, verbose, visualizeDistribution, visualizeCM, visualizeVoronoiCells, UIControlled).loop();

	return 0;	
}


