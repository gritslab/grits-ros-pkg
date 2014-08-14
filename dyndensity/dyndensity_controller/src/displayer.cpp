
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <dyndensity_controller/displayerData.h>

#define VICONUNITS_TO_METERS 0.01
#define METERS_TO_VICONUNITS 100

class Displayer{
	public: 
		Displayer(int ID, int refresh_rate, double R, double G, double B, double opaqueness,std::string shapeStr, float size, int verbose);
		void loop();

	private:
	void visualizationCallback(const dyndensity_controller::displayerData &vdata); // Called when new visualization data is recieved

//	ros::NodeHandle n;
	ros::NodeHandle mNodeHandle;

	ros::Subscriber mDataSubscriber;
	ros::Publisher marker_pub;

	uint32_t shape;

	int ID,refresh_rate;
	double R,G,B,opaqueness,size;
	int verbose;

};

Displayer :: Displayer(int ID, int refresh_rate, double R, double G, double B, double opaqueness,std::string shapeStr, float size, int verbose){
	this->ID = ID;
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
	mDataSubscriber = mNodeHandle.subscribe("/displayer/data", 1, &Displayer::visualizationCallback, this); // subscribe to Optitrack data without noise

}

void Displayer :: visualizationCallback(const dyndensity_controller::displayerData &vdata){

	float theta;
	theta = vdata.orientation.z;

	//echo 
//	std::cout << "ID " << vdata.id << " X " << X << " Y " << Y << " Z " << Z << " Orientation " << theta << "\n";

	//A new shape will only be sent if the received optitrack ID is the same as what I am
	if(vdata.id == this->ID){

	//echo
//	std::cout << "Got in for ID: " << vdata.id << "\n";


		visualization_msgs::Marker marker;

		marker.header.frame_id = "/my_frame";
		marker.header.stamp = ros::Time::now();

		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "displayer";
		marker.id = ID;

		marker.type = shape;

		// Set the marker action.  Options are ADD and DELETE
		marker.action = visualization_msgs::Marker::ADD;


		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = vdata.position.x;
		marker.pose.position.y = vdata.position.y;
		marker.pose.position.z = vdata.position.z;

		marker.pose.orientation.x = cos(theta/2);
		marker.pose.orientation.y = sin(theta/2);
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = size;
		marker.scale.y = size;
		marker.scale.z = 0.01;

		marker.color.r = R;
		marker.color.g = G;
		marker.color.b = B;
		marker.color.a = opaqueness;

		marker.lifetime = ros::Duration();

		// Publish the marker
		marker_pub.publish(marker);

	}//end if
}

void Displayer :: loop()
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

	int ID,refresh_rate;
	double R,G,B,opaqueness,size;
	std::string shapeStr;

	ros::init(argc, argv, "basic_shapes");
	
	ros :: param :: get("~ID",ID);
	ros :: param :: get("~refresh_rate",refresh_rate);	

	ros :: param :: get("~R",R);
	ros :: param :: get("~G",G);
	ros :: param :: get("~B",B);
	ros :: param :: get("~opaqueness",opaqueness);
	ros :: param :: get("~shape",shapeStr);
	ros :: param :: get("~size",size);
	
	int verbose;
	ros :: param :: get("~verbose",verbose);

	Displayer(ID,refresh_rate,R,G,B,opaqueness,shapeStr,size,verbose).loop();

	return 0;	
}


