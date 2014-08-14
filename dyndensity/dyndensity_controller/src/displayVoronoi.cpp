
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vicon_driver/ViconData.h>

#include "VoronoiDiagramGenerator.h"

#define VICONUNITS_TO_METERS 0.01
#define METERS_TO_VICONUNITS 100

class Projector{
	public: 
		Projector(int startingID, int endingID, float x1, float x2, float y1, float y2, int refresh_rate, double R, double G, double B, double opaqueness,std::string shapeStr, float size, int verbose);
		~Projector();
		void loop();

	private:
	void viconCallback(const vicon_driver::ViconData &vdata); // Called when new vicon data is recieved

//	ros::NodeHandle n;
	ros::NodeHandle mNodeHandle;

	ros::Subscriber mViconSubscriber;
	ros::Publisher marker_pub;

	uint32_t shape;

	VoronoiDiagramGenerator vdg;

	int startingID,endingID,refresh_rate,numSeeds;
	double x1,x2,y1,y2,R,G,B,opaqueness,size;
	int verbose;

	float *xValues ,*yValues;
	bool *seedsRefreshed;

};

Projector :: ~Projector(){
	delete []xValues;
	delete []yValues;
	delete []seedsRefreshed;
}

Projector :: Projector(int startingID, int endingID, float x1, float x2, float y1, float y2, int refresh_rate, double R, double G, double B, double opaqueness,std::string shapeStr, float size, int verbose){
	this->startingID = startingID;
	this->endingID = endingID;

	this->numSeeds = endingID - startingID + 1;

	//Coordinates for the voronoi seeds. Define them and then initialize them.
	xValues = new float[numSeeds];
	yValues = new float[numSeeds];
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

	if( shapeStr.compare("arrow") == 0 ){ shape = visualization_msgs::Marker::ARROW;}
	if( shapeStr.compare("cylinder") == 0 ){ shape = visualization_msgs::Marker::CYLINDER; this->size = this->size/2;}

	this->verbose = verbose;
	

	marker_pub = mNodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	mViconSubscriber = mNodeHandle.subscribe("/vicon/data", 1, &Projector::viconCallback, this); // subscribe to Optitrack data without noise

}

void Projector :: viconCallback(const vicon_driver::ViconData &vdata){
//	printf("Inside viconCallback\n");
	float X, Y, Z, theta;
	X = vdata.position.x * VICONUNITS_TO_METERS;
	Y = vdata.position.y * VICONUNITS_TO_METERS;
	Z = vdata.position.z * VICONUNITS_TO_METERS;
	theta = vdata.orientation.z;

	if( ( vdata.id <= this->endingID ) && ( this->startingID <= vdata.id ) ){
		xValues[vdata.id-startingID] = X;
		yValues[vdata.id-startingID] = Y;
		seedsRefreshed[vdata.id-startingID] = true;
//		printf("%dth agent's position refreshed to (%f,%f)\n",xValues[vdata.id-startingID],yValues[vdata.id-startingID]);
	}	

	//if any of the coordinates for the seeds are not updated, then we dont re-draw the voronoi region.
	bool go = true;
	for(int i = 0; i < numSeeds; i++){
		if(!seedsRefreshed[i]){
			go = false;
			break;
		}
	}

	if(go){
ros::Time algorithmDuration = ros::Time::now();
		//compute the Voronoi region.
		vdg.findOrderedList(x1,x2,y1,y2, xValues,yValues,numSeeds,0);
//		printf("Voronoi run.\n");
		//to extract the result, use vdg.voronoiOrderedList...this is a 2D vector.
/*
		//getting ready to publish geometry messages
		visualization_msgs::Marker points, line_strip, line_list;
		points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    		points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
		points.ns = line_strip.ns = line_list.ns = "points_and_lines";
		points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

//		points.id = 0;
//		line_strip.id = 1;
//		line_list.id = 2;

		points.type = visualization_msgs::Marker::POINTS;
		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
		line_list.type = visualization_msgs::Marker::LINE_LIST;

		line_strip.scale.x = this->size;

		line_strip.color.r = this->R;
		line_strip.color.g = this->G;
		line_strip.color.b = this->B;
*/		
//		printf("Preparing the line strip object.\n");

	//display the calculated voronoi vertices for each seed
/*
	for (unsigned int seed = 0; seed < vdg.voronoiOrderedList.size(); seed++){
		printf("For seed (%f,%f)\n",xValues[seed],yValues[seed]);
		for (unsigned int vertex = 0; vertex < vdg.voronoiOrderedList[seed].size(); vertex++){
			printf("(%f,%f) ",vdg.voronoiOrderedList[seed][vertex].x,vdg.voronoiOrderedList[seed][vertex].y);
		}
		printf("\n");
	}
*/

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
//			printf("Line strip object parameters set.\n");

			for (unsigned int vertex = 0; vertex < vdg.voronoiOrderedList[seed].size(); vertex++){
				geometry_msgs::Point p;
				p.x = vdg.voronoiOrderedList[seed][vertex].x;
				p.y = vdg.voronoiOrderedList[seed][vertex].y;
				p.z = 0;

				line_strip.points.push_back(p);
//				printf("Point attached to line strip ... voronoiOrderedList size %d\n", vdg.voronoiOrderedList[seed].size());
			}

			geometry_msgs::Point loopPoint;
			loopPoint.x = vdg.voronoiOrderedList[seed][0].x;
			loopPoint.y = vdg.voronoiOrderedList[seed][0].y;
			loopPoint.z = 0;	

			line_strip.points.push_back(loopPoint);	
	
//			printf("Loopback point attached to line strip\n");

			marker_pub.publish(line_strip);
//			printf("Published for %dth seed\n",seed);

		}

		//reset the flags for seed update.
		for(int i = 0; i < numSeeds; i++) seedsRefreshed[i] = false;
//		printf("All flags reset\n");

ros::Time algorithmDuration2 = ros::Time::now();
printf("Time for the algorithm to run: %f\n",algorithmDuration2.toSec()-algorithmDuration.toSec());

	}
}

void Projector :: loop()
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

	ros::init(argc, argv, "basic_shapes");
	
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
	
	int verbose;
	ros :: param :: get("~verbose",verbose);

	Projector(startingID, endingID, x1, x2, y1, y2, refresh_rate, R, G, B, opaqueness, shapeStr, size, verbose).loop();

	return 0;	
}


