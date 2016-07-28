// include define namespace/*{{{*/
#include <ros/ros.h>
#include <math.h>            
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <tf/transform_datatypes.h>   // quaternion -> rpy

#define N 100   // path の分解能
#define LOOPRATE 10
#define X 0
#define Y 1
#define YAW 2

using namespace std;/*}}}*/

class BezerCurve{/*{{{*/
	private:
		geometry_msgs::PoseStamped bezer;
	public:
		void CalcPath();
		BezerCurve(){
		};
};/*}}}*/

// grobal variable /*{{{*/
bool QRSubFlag = false;
double goal0[3] = {0,0,0};   // initial pose {x,y,yaw}
double goal1[3];
nav_msgs::Path BezerPath;
geometry_msgs::PoseArray QRPoses;
/*}}}*/

void QRPoseCallback(const geometry_msgs::PoseStamped &msg)/*{{{*/
{
	QRSubFlag = true;

	tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	/*
	// display subscribed position pose(rpy)
	cout<<"===================="<<endl;
	cout<<"Roll: "<<roll<<", Pitch: "<<pitch<<", Yaw: "<<yaw<<endl;	
	cout<<"x: "<<msg.pose.position.x<<", y:"<<msg.pose.position.y<<", z:"<<msg.pose.position.z<<endl;
	*/

	QRPoses.poses.push_back(msg.pose);

	goal0[X] = goal1[X];
	goal0[Y] = goal1[Y];
	goal0[YAW] = goal1[YAW];
	goal1[X] = msg.pose.position.x;
	goal1[Y] = msg.pose.position.y;
	goal1[YAW] = yaw;
}/*}}}*/

void BezerCurve::CalcPath()/*{{{*/
{
	double t = 0;
	double CPoint0[4][2];
	double CPoint1[3][2];
	double CPoint2[2][2];

	//BezerPath.poses.clear();

	// 接線の長さを決定 2点の距離*Ktlen 暫定版
	static double Ktlen = 0.3;
	double TLength = Ktlen * sqrt( pow(goal1[X]-goal0[X],2) + pow(goal1[Y]-goal0[Y],2) );

	// goal0 goal1 から4つの制御点を指定
	CPoint0[0][X] = goal0[X];
	CPoint0[0][Y] = goal0[Y];

	CPoint0[1][X] = goal0[X] + TLength * cos(goal0[YAW]);
	CPoint0[1][Y] = goal0[Y] + TLength * sin(goal0[YAW]);

	CPoint0[3][X] = goal1[X];
	CPoint0[3][Y] = goal1[Y];

	CPoint0[2][X] = goal1[X] - TLength * cos(goal1[YAW]);
	CPoint0[2][Y] = goal1[Y] - TLength * sin(goal1[YAW]);

	/*
	// display TLength 4points
	cout<<"TLength: "<<TLength<<endl;
	cout<<"P0.X: "<<CPoint0[0][X]<<", P0.Y: "<<CPoint0[0][Y]<<endl;
	cout<<"P1.X: "<<CPoint0[1][X]<<", P1.Y: "<<CPoint0[1][Y]<<endl;
	cout<<"P2.X: "<<CPoint0[2][X]<<", P2.Y: "<<CPoint0[2][Y]<<endl;
	cout<<"P3.X: "<<CPoint0[3][X]<<", P3.Y: "<<CPoint0[3][Y]<<endl;
	*/

	for (int i=0; i<N; i++)
	{
		// CPoint1 detection
		for (int j=0; j<3; j++) {
			for (int h=0; h<2; h++) {
				CPoint1[j][h] = CPoint0[j][h] + i*(CPoint0[j+1][h]-CPoint0[j][h])/N;
			}
		}

		// CPoint2 detection
		for (int j=0; j<2; j++) {
			for (int h=0; h<2; h++) {
				CPoint2[j][h] = CPoint1[j][h] + i*(CPoint1[j+1][h]-CPoint1[j][h])/N;
			}
		}

		// CPoint3 detection
		for (int j=0; j<1; j++) {
				bezer.pose.position.x = CPoint2[j][X] + i*(CPoint2[j+1][X]-CPoint2[j][X])/N;
				bezer.pose.position.y = CPoint2[j][Y] + i*(CPoint2[j+1][Y]-CPoint2[j][Y])/N;
		}

		bezer.pose.position.z = 0;
		bezer.pose.orientation.x = 0;
		bezer.pose.orientation.y = 0;
		bezer.pose.orientation.z = 0;
		bezer.pose.orientation.w = 0;

		BezerPath.poses.push_back(bezer);
	}

}/*}}}*/

int main(int argc, char** argv)/*{{{*/
{
	ros::init(argc,argv,"path_detetion");
	ros::NodeHandle nh;
	BezerCurve bezercurve;
	BezerPath.header.frame_id = "world";
	QRPoses.header.frame_id = "world";
	ros::Publisher PathPub = nh.advertise<nav_msgs::Path>("/BezerPath",1);
	ros::Publisher QRPosesPub = nh.advertise<geometry_msgs::PoseArray>("/QRPoses",1);
	ros::Subscriber QRPoseSub = nh.subscribe("/move_base_simple/goal",1,QRPoseCallback);

	ros::Rate loop_rate(LOOPRATE);
	while (ros::ok())
	{
		if (QRSubFlag) {
			bezercurve.CalcPath();
			PathPub.publish(BezerPath);
			QRPosesPub.publish(QRPoses);
			QRSubFlag = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}/*}}}*/
