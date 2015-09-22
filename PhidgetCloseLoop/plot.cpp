#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <fstream>
#include <time.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "PhidgetClass.h"
#include "cvdrawingutils.h"
#include "spline.h"
#include "LU"
#include "Geometry"
#include "Dense"
#include "SVD"

//#include "ArucoTest.h"

using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;

bool DispThreadHasFinished = false;
bool StepThreadHasFinished = false;
bool MainThreadHasFinished = false;
bool camready = false;
int w = 1200; int h = 400; int leftM = 0;
Mat plotImage1( h, w, CV_8UC3, Scalar( 0,0,0) );
Mat plotImage2( h, w, CV_8UC3, Scalar( 0,0,0) );
double tick = 0.0, start = 0.0, RzKal1_current[2] = {0.0,0.0}, RzKal2_current[2] = {0.0,0.0}, dt=40.0/1000.0;
double target1 = 0.0, target2 = 0.0; double domf[2] = {0.0,0.0};
CPhidgetWrapper pw[2];
tk::spline s1,s2;
double t_max = -1, phi_d[2] = { 0.0,0.0 }, dphi_d[2] = { 0.0,0.0 }, startpt[2] = { 0.0,1 };

#define WMOT 2	//0,1,2
#define REC 1

struct PT {
	double x;
	double y;
};

double motor2body(double ang)
{
	return ang*12.7/115.0;
}
double body2motor(double ang)
{	
	return ang*115.0/12.7;
}
double softstep(double t)	//pi/4 en 5s.
{
	double a[3] = { 0.0628319, -0.0188496, 0.00150796 };// { 0.502655, -0.301593, 0.0482549 } { 0.125664, -0.0376991, 0.00301593 }; //{ 0.000581776,-0.0000290888, 0.000000387851 };
	t_max=5;
	if(t>t_max)
		t=t_max;

	return a[0]*pow(t,3)+a[1]*pow(t,4)+a[2]*pow(t,5);
}

double dsoftstep(double t)
{
	double a[3]= { 0.0628319, -0.0188496, 0.00150796 };//{ 0.125664,-0.0376991,0.00301593 }; //{ 0.000581776,-0.0000290888, 0.000000387851 };
	t_max=5;
	if(t>t_max)
		t=t_max;

	return 3*a[0]*pow(t,2)+4*a[1]*pow(t,3)+5*a[2]*pow(t,4);
}

template<typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<double>::epsilon())
{
  if(a.rows() < a.cols())
   return false;
  Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();
  result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
}

float remap(float value, float istart, float istop, float ostart, float ostop) {
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

/* From flywheel accelerations to body accel.*/
void Kanebodydyn(double phi[2], double dphi[2], double dfly[2], double(&res)[2]) {

	//cout << "Starting calcul..." <<endl;
	MatrixXd A(4, 4), A_inv(4, 4), b(4, 1), bomf(4, 2), acc(4,1); //row,col
																									  // CM *joint opposé au moteur
	double ma = 0.5, mb = 0.5, Iwash = 3 * 0.05*pow(0.06, 2);
	double If1 = 0.00015424; //+Iwash
	double Ia = 0.00164167 + If1;
	double If2 = If1; double Ib = Ia;
	//cout << "Initialize vectors..."<<endl;
	Vector2d initrj(120.322 / 1000.0 + 0.048901, -0.00004962);
	//cout << "RJ:" << initrj << endl;
	//cout << "Copy vectors..."<<endl;
	Vector2d raj; raj << initrj;
	Vector2d rbj; rbj << -initrj;

	double abi_u3 = -raj(1)*cos(phi[0]) - raj(0)*sin(phi[0]); double abi_u4 = rbj(1)*cos(phi[1]) + rbj(0)*sin(phi[1]);
	double abi_rst = -raj(0)*cos(phi[0])*pow(dphi[0], 2) + raj(1)*sin(phi[0])*pow(dphi[0], 2) + rbj(0)*cos(phi[1])*pow(dphi[1], 2) - rbj(1)*sin(phi[1])*pow(dphi[1], 2);
	double abj_u3 = -raj(1)*sin(phi[0]) + raj(0)*cos(phi[0]); double abj_u4 = rbj(1)*sin(phi[1]) - rbj(0)*cos(phi[1]);
	double abj_rst = -raj(0)*sin(phi[0])*pow(dphi[0], 2) - raj(1)*cos(phi[0])*pow(dphi[0], 2) + rbj(0)*sin(phi[1])*pow(dphi[1], 2) + rbj(1)*cos(phi[1])*pow(dphi[1], 2);
	double dvbdu3i = -raj(0)*sin(phi[0]) - raj(1)*cos(phi[0]); double dvbdu3j = raj(0)*cos(phi[0]) - raj(1)*sin(phi[0]);
	double dvbdu4i = rbj(0)*sin(phi[1]) + rbj(1)*cos(phi[1]); double dvbdu4j = -rbj(0)*cos(phi[1]) + rbj(1)*sin(phi[1]);
	//cout << "Filling matrices..." << endl;
	A << ma+mb,	0,	mb*abi_u3,	mb*abi_u4,
	0,	ma+mb,	mb*abj_u3,	mb*abj_u4,
	mb*dvbdu3i,	mb*dvbdu3j,	Ia+mb*dvbdu3i*abi_u3+mb*dvbdu3j*abj_u3,	mb*dvbdu3i*abi_u4+mb*dvbdu3j*abj_u4,
	mb*dvbdu4i,	mb*dvbdu4j,	mb*dvbdu4i*abi_u3+mb*dvbdu4j*abj_u3,	Ib+mb*dvbdu4i*abi_u4+mb*dvbdu4j*abj_u4;
	//cout << "A: " << A << endl;
	b << mb*abi_rst,
	mb*abj_rst,
	mb*dvbdu3i*abi_rst+mb*dvbdu3j*abj_rst,
	mb*dvbdu4i*abi_rst+mb*dvbdu4j*abj_rst;
		mb*dvbdu4i*abi_rst + mb*dvbdu4j*abj_rst;
	//cout << "b:" << b << endl;
	bomf << 0,0,
		0,0,
		-If1,0,
		0,-If2;
	//cout << "bomf: " << bomf << endl;
	pseudoInverse(A,A_inv,0.00000001);
	MatrixXd tmp(4, 1); tmp << dfly[0], dfly[1], 0, 0;
	acc = A_inv*(bomf*tmp - b);
	res[0] = acc(2); res[1] = acc(3);
	return;
}

/* From Actual pos\vit & desired pos\vit get the flywheel accelerations*/
void KaneINVbodydyn(double phi[2], double dphi[2], double error[2], double (&res)[2]) {

	//cout << "Starting calcul..." <<endl;
	MatrixXd Ap(2,2), Av(2,4), u(2, 1), bv(2,1), bp(2, 1), bomf(2,2), bomf_inv(2,2), domf(2,1); //row,col
	// CM *joint opposé au moteur
	double ma=0.5,mb=0.5, Iwash=3*0.05*pow(0.06,2);
	double If1=0.00015424; //+Iwash
	double Ia = 0.00164167 + If1;
	double If2=If1; double Ib=Ia; 
	//cout << "Initialize vectors..."<<endl;
	Vector2d initrj(120.322/1000.0+0.048901,-0.00004962);
	//cout << "RJ:" << initrj << endl;
	//cout << "Copy vectors..."<<endl;
	Vector2d raj;raj<<initrj;
	Vector2d rbj;rbj<<-initrj;

	double abi_u3=-raj(1)*cos(phi[0])-raj(0)*sin(phi[0]);double abi_u4=rbj(1)*cos(phi[1])+rbj(0)*sin(phi[1]);
	double abi_rst=-raj(0)*cos(phi[0])*pow(dphi[0],2)+raj(1)*sin(phi[0])*pow(dphi[0],2)+rbj(0)*cos(phi[1])*pow(dphi[1],2)-rbj(1)*sin(phi[1])*pow(dphi[1],2);
	double abj_u3=-raj(1)*sin(phi[0])+raj(0)*cos(phi[0]);double abj_u4=rbj(1)*sin(phi[1])-rbj(0)*cos(phi[1]);
	double abj_rst=-raj(0)*sin(phi[0])*pow(dphi[0],2)-raj(1)*cos(phi[0])*pow(dphi[0],2)+rbj(0)*sin(phi[1])*pow(dphi[1],2)+rbj(1)*cos(phi[1])*pow(dphi[1],2);
	double dvbdu3i=-raj(0)*sin(phi[0])-raj(1)*cos(phi[0]);double dvbdu3j=raj(0)*cos(phi[0])-raj(1)*sin(phi[0]);
	double dvbdu4i=rbj(0)*sin(phi[1])+rbj(1)*cos(phi[1]);double dvbdu4j=-rbj(0)*cos(phi[1])+rbj(1)*sin(phi[1]);
	//cout << "Filling matrices..." << endl;
	/*A << ma+mb,	0,	mb*abi_u3,	mb*abi_u4,
		0,	ma+mb,	mb*abj_u3,	mb*abj_u4,
		mb*dvbdu3i,	mb*dvbdu3j,	Ia+mb*dvbdu3i*abi_u3+mb*dvbdu3j*abj_u3,	mb*dvbdu3i*abi_u4+mb*dvbdu3j*abj_u4,
		mb*dvbdu4i,	mb*dvbdu4j,	mb*dvbdu4i*abi_u3+mb*dvbdu4j*abj_u3,	Ib+mb*dvbdu4i*abi_u4+mb*dvbdu4j*abj_u4;*/
	Ap << mb*abi_u3, mb*abi_u4,
		mb*abj_u3, mb*abj_u4;
	Ap = -Ap;
	Av << mb*dvbdu3i, mb*dvbdu3j, Ia + mb*dvbdu3i*abi_u3 + mb*dvbdu3j*abj_u3, mb*dvbdu3i*abi_u4 + mb*dvbdu3j*abj_u4,
		mb*dvbdu4i, mb*dvbdu4j, mb*dvbdu4i*abi_u3 + mb*dvbdu4j*abj_u3, Ib + mb*dvbdu4i*abi_u4 + mb*dvbdu4j*abj_u4;
	Av = -Av;
	//cout << "A: " << A << endl;
	/*b << mb*abi_rst,
		mb*abj_rst,
		mb*dvbdu3i*abi_rst+mb*dvbdu3j*abj_rst,
		mb*dvbdu4i*abi_rst+mb*dvbdu4j*abj_rst;*/
	bp << mb*abi_rst,
		mb*abj_rst;
	bv << mb*dvbdu3i*abi_rst+mb*dvbdu3j*abj_rst,
		mb*dvbdu4i*abi_rst+mb*dvbdu4j*abj_rst;
	//cout << "b:" << b << endl;
	/*bomf << -If1,0,
		0,-If2;*/
	bomf_inv << -1 / If1, 0,
		0, -1 / If2;
	//cout << "bomf: " << bomf << endl;
	VectorXd SEQ(4);
	//VectorXd a(4);a << ad[0],ad[1],ad[2],ad[3];
	VectorXd fd(2);//fd << 0,	0,	50*(phi_d[0]-phi[0])+10*(dphi_d[0]-dphi[0]),	50*(phi_d[1]-phi[1])+10*(dphi_d[1]-dphi[1]);
	fd << error[0], error[1];// 50 * (phi_d[0]) + 10 * (dphi_d[0]), 50 * (phi_d[1]) + 10 * (dphi_d[1]); for test.
	u = 1 / (ma + mb)*Ap*fd - bp;
	//cout << fd << endl;
	MatrixXd tmp(4, 1); tmp << u(0), u(1), fd(0), fd(1);
	//pseudoInverse(bomf,bomf_inv,0.00000001);
	domf=bomf_inv*(Av*tmp - bv);
	res[0]=domf(0);res[1]=domf(1);
	return;
}

void ThreadDisplay() {
	/// Display
	namedWindow("Plot Curves Left", CV_WINDOW_AUTOSIZE );
	namedWindow("Plot Curves Right", CV_WINDOW_AUTOSIZE );
	
	while ( !MainThreadHasFinished ) {
		/// Update
		imshow("Plot Curves Left", plotImage1 );
		imshow("Plot Curves Right", plotImage2 );
		waitKey(10);
	}
	
	DispThreadHasFinished=true;
	return;
}
void ThreadMot() {
	int stepcommand=0;
	double Rzerror1_old = 0.0, Rzerror1_int = 0.0, Rzerror2_old = 0.0, Rzerror2_int = 0.0, t=0.0;
	while ( !MainThreadHasFinished ) {
		if(camready && WMOT>0)
		{
			t = (getTickCount()-start)/getTickFrequency();
			//cout << "CtrLoop speed :" << t - oldt << endl;
			//target1 =  s1(t);//softstep(t);//s1(t);//
			//target2 =  s2(t);//oftstep(t);//s2(t);
			double phi[2] = {RzKal1_current[0], RzKal2_current[0]}; //{ phi_d[0], phi_d[1] } for test
			double dphi[2] = {RzKal1_current[1], RzKal2_current[1]}; //{ dphi_d[0], dphi_d[1] }
			//cout << "Theta: " << (phi[0] - phi[1]) * 180 / PI << endl;
			//double ad[4]={0,0,-sin(ts),0};
			phi_d[0]=startpt[0];phi_d[1]=startpt[1];
			dphi_d[0]= 0;dphi_d[1]=0;
			// Compute error
			double Rzerror1 = phi_d[0]-RzKal1_current[0];
			double Rzerror2 = phi_d[1]-RzKal2_current[0];
			/*if(t_max!=-1){	//stop the run if over max time
				if(t>t_max){
					target1 = 0;//s1(t_max);//softstep(t_max);//s1(t_max);
					//Rzerror1 = 0;Rzerror1_int = 0;
					if(WMOT>1){
						target2 = 0;//s2(t_max);//softstep(t_max);//s2(t_max);
						//Rzerror2 = 0;Rzerror2_int = 0;
					}
				}
			}*/
			double Rzerror1_der = dphi_d[0] - RzKal1_current[1];//(Rzerror1 - Rzerror1_old) / dt;
			double Rzerror2_der = dphi_d[1] - RzKal2_current[1];//(Rzerror2 - Rzerror2_old) / dt;
			//Rzerror1_int += Rzerror1*dt;Rzerror2_int += Rzerror2*dt;
			//if(abs(Rzerror1_int)>PI/10)	Rzerror1_int=sgn(Rzerror1_int)*PI/10;
			//if(abs(Rzerror2_int)>PI/10)	Rzerror2_int=sgn(Rzerror2_int)*PI/10;
			/*---------------------------
			// Run the stepper
			__int64 stepcommand = pw.rad2steps(body2motor(10*Rzerror+0.1*Rzerror_int+2*Rzerror_der)); //2 1.5 0.05
			if(abs(stepcommand)>10000)	stepcommand=sgn(stepcommand)*10000;
			pw.GoToStepper(0, stepcommand);
			----------------------------*/
			Rzerror1_old=Rzerror1;
			Rzerror2_old=Rzerror2;
			double damp = 0.4;
			double natfreq = 1.25;
			double command[2] = { pow(natfreq,2)* Rzerror1 + 2 * damp*natfreq * Rzerror1_der , pow(natfreq,2) * Rzerror2 + 2 * damp*natfreq * Rzerror2_der };
			cout << command[0] << ", " << command[1] << endl;
			KaneINVbodydyn(phi, dphi, command, domf);
			target1 = target1 + domf[0] * dt; //simple integration to get speed of flywheels.
			target2 = target2 + domf[1] * dt;
			// Run the dcmotor
			//cout << WMOT << "-t:" << t << ", r1:" << target1 << "(e:" << body2motor(Rzerror1) << "), r2:" << target2 << endl;
			/*----------------------
			// SCALING COMMANDS.
			double iMax = max(abs(body2motor(target1)), abs(body2motor(target2)));
			if (iMax < 100)
				iMax = 100;
			double t1tmp = sgn(body2motor(target1))*remap(abs(body2motor(target1)), 0, iMax, 0, 100); double t2tmp = sgn(body2motor(target2))*remap(abs(body2motor(target2)), 0, iMax, 0, 100);
			----------------------*/

			pw[leftM].setvel(body2motor(-target1));// -t1tmp);//150*Rzerror1+5*Rzerror1_int+2*Rzerror1_der));
			if (WMOT > 1)	pw[!leftM].setvel(body2motor(-target2));// -t2tmp);//150*Rzerror2+5*Rzerror2_int+2*Rzerror2_der));

			/*if (t > t_max)	t = t_max;
			pw[leftM].target=body2motor(2*s1(t));
			pw[leftM].started_pos=true;
			if(WMOT>1)	pw[!leftM].target=body2motor(2*s2(t));
			if(WMOT>1)	pw[!leftM].started_pos=true;*/

			/*if (t > t_max)	t = t_max;
			pw[leftM].target=body2motor(s1(t));
			pw[leftM].started_cur=true;
			if(WMOT>1)	pw[!leftM].target=body2motor(s2(t));
			if(WMOT>1)	pw[!leftM].started_cur=true;*/
		}
		waitKey(35);
	}
	
	StepThreadHasFinished=true;
	return;
}

void getspline(char* csvname)
{
vector<double> t,r1,r2;
	ifstream file(csvname);
	double tempr1, tempr2, tempt;
	string line;
	char comma;
	while(getline(file,line))
	{
		file >> tempt >> comma >> tempr1 >> comma >> tempr2;
		t.push_back(tempt);r1.push_back(tempr1);r2.push_back(tempr2);		//sens positif des moteurs inversé p/r au modèle - inversé par body2motor
	}
	t.pop_back();r1.pop_back();r2.pop_back();
	cout << "Dataset size: " << t.size() << " " << r1.size() << endl;
	for(int i=0;i<t.size();i++)
		cout << t[i] << " ";
	cout << endl;
	s1.set_points(t,r1);
	s2.set_points(t,r2);
	t_max=t[t.size()-1];
}

/**
 * @function main
 */
int main( int argc, char** argv )
{
  /// Get data point and spline
	//getspline("t_t1_t2.csv");
  /// CSV file log
	time_t t = time(0);   // get time now
	struct tm * now = localtime( & t );
	char buffer [80];
	ofstream csvfile;
	if(REC) {
		strftime(buffer,80,"%Y-%m-%d-%H%M.csv",now);
		csvfile.open(buffer);
		if(!csvfile.is_open())
		{
			cout<<"Error opening csv file."<<endl;
			return -1;
		}
	}
  /// START TRACKER
	VideoWriter outputVideo;
	MarkerDetector MDetector;VideoCapture TheVideoCapturer;vector<Marker> TheMarkers; Mat TheInputImage,TheInputImageCopy;CameraParameters TheCameraParameters;
	pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
	char key=0;Mat rotation1Init, rotation2Init;int leftT =0;
	try{
		//read from camera or from  file
		int vIdx=0;
		cout<<"Opening camera index "<<vIdx<<endl;
		TheVideoCapturer.open(vIdx);

		//check video is open
		if (!TheVideoCapturer.isOpened()) {
			cerr<<"Could not open video"<<endl;
			return -1;
		}
		
		//read first image to get the dimensions		
		//TheVideoCapturer.set(CV_CAP_PROP_CONVERT_RGB, 1);
		//TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, 800);
		//TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
		TheVideoCapturer>>TheInputImage;
		//read camera parameters
		TheCameraParameters.readFromXMLFile("camera.yml");
		TheCameraParameters.resize(TheInputImage.size());
		//Resize image?
		MDetector.pyrDown(1);
		//Create debug gui
		cv::namedWindow("in",1);   
		MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
		if(REC){
			strftime(buffer,80,"%Y-%m-%d-%H%M.avi",now);
			outputVideo.open(buffer,CV_FOURCC('I','Y','U','V'),25,TheInputImage.size(),true);
			if (!outputVideo.isOpened())
			{
				cout  << "Could not open the output video for write. " << endl;
				return -1;
			}
		}
	} catch (std::exception &ex) {cout<<"Exception :"<<ex.what()<<endl;}
	KalmanFilter KF1(3, 2, 0);KalmanFilter KF2(3, 2, 0);
	// intialization of KF...
	KF1.transitionMatrix = *(Mat_<float>(3, 3) << 1,dt,dt/2,   0,1,dt,  0,0,1);KF2.transitionMatrix = *(Mat_<float>(3, 3) << 1,dt,dt/2,   0,1,dt,  0,0,1);
	cout << KF1.measurementMatrix.size() << KF1.measurementNoiseCov.size() << endl;
	//setIdentity(KF.measurementMatrix);
	KF1.measurementMatrix = *(Mat_<float>(2, 3) << 1,0,0,	0,1,0);KF2.measurementMatrix = *(Mat_<float>(2, 3) << 1,0,0,	0,1,0);
	Mat measurement1 = *(Mat_<float>(2, 1) << 0,0);Mat measurement2 = *(Mat_<float>(2, 1) << 0,0);
	//setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	KF1.processNoiseCov = *(Mat_<float>(3, 3) << pow(dt,5)/20,pow(dt,4)/8,pow(dt,3)/6,   pow(dt,4)/8,pow(dt,3)/3,pow(dt,2)/2,  pow(dt,3)/6,pow(dt,2)/2,dt);
	KF2.processNoiseCov = *(Mat_<float>(3, 3) << pow(dt,5)/20,pow(dt,4)/8,pow(dt,3)/6,   pow(dt,4)/8,pow(dt,3)/3,pow(dt,2)/2,  pow(dt,3)/6,pow(dt,2)/2,dt);
	//setIdentity(KF.measurementNoiseCov, Scalar::all(0.1));
	KF1.measurementNoiseCov = *(Mat_<float>(2, 2) << 0.1,0,	0,1000); KF2.measurementNoiseCov = *(Mat_<float>(2, 2) << 0.1,0,	0,1000);
	setIdentity(KF1.errorCovPost, Scalar::all(1)); setIdentity(KF2.errorCovPost, Scalar::all(1));

  /// PLOT THREAD
	DWORD dwThreadId;
	CreateThread(NULL, //Choose default security
		0, //Default stack size
		(LPTHREAD_START_ROUTINE)&ThreadDisplay, //Routine to execute
		NULL, //Thread parameter
		0, //Immediately run the thread
		&dwThreadId //Thread Id
		);
	// Establish the number of bins
	int plotSize = 600;int index = 0; int plotrange=4*PI;
	int bin_w = cvRound( (double)w/(double)plotSize );
	Mat RzCam1, RzCam2, RzStep, RzCom1, RzCom2, KalmanR1, KalmanR2; RzCam1.create(plotSize,1,CV_32FC1); RzCam2.create(plotSize,1,CV_32FC1); RzCom1.create(plotSize,1,CV_32FC1); RzCom2.create(plotSize,1,CV_32FC1); KalmanR1.create(plotSize,1,CV_32FC1); KalmanR2.create(plotSize,1,CV_32FC1);RzStep.create(plotSize,1,CV_32FC1); 
	RzCam1.at<float>(0)=0.0;RzCam2.at<float>(0)=0.0;RzStep.at<float>(0)=0.0;float scale=(float)h/(plotrange);

  /// Run stepper THREAD
	if(WMOT>0){
		pw[0].Init(0); // init 0 for dc motor, 1 for stepper
		pw[0].setGains(30, 0, 1, 100, 0);
		if (WMOT > 1) {
			pw[1].Init(0);
			pw[1].setGains(30, 0, 1, 100, 0);
		}
		if(pw[0].devid==394129)
			leftM=0;
		else
			leftM=1;
		CreateThread(NULL, //Choose default security
			0, //Default stack size
			(LPTHREAD_START_ROUTINE)&ThreadMot, //Routine to execute
			NULL, //Thread parameter
			0, //Immediately run the thread
			&dwThreadId //Thread Id
			);
	}
	double phiInit[2] = { 0.0,0.0 }, thetaInit=0.0;
	tick = (double)getTickCount();//for checking the speed

	/*------------------------
	Initialize marker angles
	-------------------------*/
	TheVideoCapturer.retrieve(TheInputImage);
	MDetector.detect(TheInputImage, TheMarkers, TheCameraParameters, 0.046);
	for (unsigned int i = 0; i < TheMarkers.size(); i++) {
		if (TheMarkers[i].id == 599) {
			Mat rotation1;
			Rodrigues(TheMarkers[i].Rvec, rotation1);
			phiInit[0] = atan2(-rotation1.at<float>(0, 1), rotation1.at<float>(0, 0));
		} else {
			Mat rotation2;
			Rodrigues(TheMarkers[i].Rvec, rotation2);
			phiInit[1] = atan2(-rotation2.at<float>(0, 1), rotation2.at<float>(0, 0));
		}
	}
	thetaInit = phiInit[1] - phiInit[0];
	cout << "Theta inital: " << thetaInit << endl;
	RzCam1.at<float>(0) = 0;
	RzCam2.at<float>(0) = thetaInit;

	KF1.statePre.at<float>(0) = RzCam1.at<float>(0);
	KalmanR1.at<float>(0) = RzCam1.at<float>(0);
	KF2.statePre.at<float>(0) = RzCam2.at<float>(0);
	KalmanR2.at<float>(0) = RzCam2.at<float>(0);
  /*--------------------
  * LOOP
  ----------------------*/
	do 
	{

		if(index>=plotSize-1){
			plotImage1=Mat::zeros( h, w, CV_8UC3 );
			plotImage2=Mat::zeros( h, w, CV_8UC3 );
			index=0;
		}

		TheVideoCapturer.retrieve(TheInputImage);//get image
		index++; //number of images captured
		//Detection of markers in the image passed
		MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,0.046); //markers size 46mm

		//print marker info and draw the markers in image
		TheInputImage.copyTo(TheInputImageCopy);
	    
		if (TheMarkers.size() > 0) {
			for (unsigned int i = 0; i < TheMarkers.size(); i++) {
				//cout << "ID: " << TheMarkers[i].id << endl;

				if (TheMarkers.size() < 2) {
					cout << "Lost a marker!" << endl;
					continue;
				}

				if (TheMarkers[i].id == 599) {
					Mat rotation1;
					Rodrigues(TheMarkers[i].Rvec, rotation1);
					RzCam1.at<float>(index) = atan2(-rotation1.at<float>(0, 1), rotation1.at<float>(0, 0)) - phiInit[0];//-RzCam1.at<float>(0);
					if (abs(RzCam1.at<float>(index)) > 30)
						RzCam1.at<float>(index) = RzCam1.at<float>(index - 1);
				}else {
					Mat rotation2;
					Rodrigues(TheMarkers[i].Rvec, rotation2);
					RzCam2.at<float>(index) = atan2(-rotation2.at<float>(0, 1), rotation2.at<float>(0, 0)) - phiInit[1]+thetaInit;//-RzCam2.at<float>(0);
					if (abs(RzCam2.at<float>(index)) > 30)
						RzCam2.at<float>(index) = RzCam2.at<float>(index - 1);
				}
			}	//out of for i markers
			line(plotImage1, Point(bin_w*(index - 1), h - cvRound((RzCam1.at<float>(index - 1) + plotrange / 2)*scale)),
				Point(bin_w*(index), h - cvRound((RzCam1.at<float>(index) + plotrange / 2)*scale)),
				Scalar(255, 0, 0), 2, 8, 0);
			line(plotImage2, Point(bin_w*(index - 1), h - cvRound((RzCam2.at<float>(index - 1) + plotrange / 2)*scale)),
				Point(bin_w*(index), h - cvRound((RzCam2.at<float>(index) + plotrange / 2)*scale)),
				Scalar(255, 0, 0), 2, 8, 0);
		}//out of ifmarker>0
		measurement1.at<float>(0) = RzCam1.at<float>(index);
		measurement2.at<float>(0) = RzCam2.at<float>(index);

		//Velocities
		measurement1.at<float>(1) = motor2body(-pw[leftM].curr_vel);
		measurement2.at<float>(1) = motor2body(-pw[!leftM].curr_vel);

		//draw a 3d cube in each marker if there is 3d info
		if (  TheCameraParameters.isValid())
			for (unsigned int i=0;i<TheMarkers.size();i++) {
				CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
				CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
			}

		// Motor Pos 
		if(WMOT>0){
			/*RzStep.at<float>(index)=motor2body(pw[leftM].curr_pos);
			//cout<<"stepper rz: "<<RzStep.at<float>(index);
			line( plotImage1, Point( bin_w*(index-1), h - cvRound((RzStep.at<float>(index-1)+plotrange/2)*scale) ) ,
							Point( bin_w*(index), h - cvRound((RzStep.at<float>(index)+plotrange/2)*scale) ),
							Scalar( 0, 255, 0), 2, 8, 0  );*/
			if(camready){
				//if((tick-start)/getTickFrequency()<t_max) RzCom.at<float>(index)=softstep((tick-start)/getTickFrequency());
				//else RzCom.at<float>(index)=softstep(t_max);
				

				RzCom1.at<float>(index)=phi_d[0];
				RzCom2.at<float>(index)=phi_d[1];
				//cout<<"stepper rz: "<<RzStep.at<float>(index);
				line( plotImage1, Point( bin_w*(index-1), h - cvRound((RzCom1.at<float>(index-1)+plotrange/2)*scale) ) ,
							   Point( bin_w*(index), h - cvRound((RzCom1.at<float>(index)+plotrange/2)*scale) ),
							   Scalar( 255, 255, 255), 2, 8, 0  );
				line( plotImage2, Point( bin_w*(index-1), h - cvRound((RzCom2.at<float>(index-1)+plotrange/2)*scale) ) ,
							   Point( bin_w*(index), h - cvRound((RzCom2.at<float>(index)+plotrange/2)*scale) ),
							   Scalar( 255, 255, 255), 2, 8, 0  );
			}
		}
		//chekc the speed by calculating the mean speed of all iterations
		AvrgTime.first += ((double)getTickCount() - tick) / getTickFrequency();
		AvrgTime.second++;
		dt = AvrgTime.first / AvrgTime.second;
		tick = (double)getTickCount();//for checking the speed

		//show input with augmented information and  the thresholded image
		char text[255];
		sprintf(text, "FPS %3.3f", 1.0 / dt);
		putText(TheInputImageCopy, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
		cv::imshow("in", TheInputImageCopy);
		if (REC)	outputVideo.write(TheInputImageCopy);
		//cout<<"\rTime detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds nmarkers="<<TheMarkers.size()<<std::flush<<endl;

		// KF predict, to update the internal statePre variable
		KF1.transitionMatrix = *(Mat_<float>(3, 3) << 1,dt,dt/2,   0,1,dt,  0,0,1);
		KF1.processNoiseCov = *(Mat_<float>(3, 3) << pow(dt,5)/20,pow(dt,4)/8,pow(dt,3)/6,   pow(dt,4)/8,pow(dt,3)/3,pow(dt,2)/2,  pow(dt,3)/6,pow(dt,2)/2,dt);
		KF2.transitionMatrix = *(Mat_<float>(3, 3) << 1,dt,dt/2,   0,1,dt,  0,0,1);
		KF2.processNoiseCov = *(Mat_<float>(3, 3) << pow(dt,5)/20,pow(dt,4)/8,pow(dt,3)/6,   pow(dt,4)/8,pow(dt,3)/3,pow(dt,2)/2,  pow(dt,3)/6,pow(dt,2)/2,dt);
		KF1.predict(); KF2.predict();
		// KF update
		Mat estimated1 = KF1.correct(measurement1); Mat estimated2 = KF2.correct(measurement2);
		KalmanR1.at<float>(index)=estimated1.at<float>(0);RzKal1_current[0]=estimated1.at<float>(0);RzKal1_current[1]=estimated1.at<float>(1);
		line( plotImage1, Point( bin_w*(index-1), h - cvRound((KalmanR1.at<float>(index-1)+plotrange/2)*scale) ) ,
					Point( bin_w*(index), h - cvRound((KalmanR1.at<float>(index)+plotrange/2)*scale) ),
					Scalar( 0, 0, 255), 2, 8, 0  );
		KalmanR2.at<float>(index)=estimated2.at<float>(0);RzKal2_current[0]=estimated2.at<float>(0);RzKal2_current[1]=estimated2.at<float>(1);
		line( plotImage2, Point( bin_w*(index-1), h - cvRound((KalmanR2.at<float>(index-1)+plotrange/2)*scale) ) ,
					Point( bin_w*(index), h - cvRound((KalmanR2.at<float>(index)+plotrange/2)*scale) ),
					Scalar( 0, 0, 255), 2, 8, 0  );

		if(camready && REC){
			if(WMOT)	csvfile << (getTickCount()-start)/getTickFrequency() << ";" << phi_d[0] << ";" << phi_d[1] << ";" << target1 << ";" << target2 << ";" << domf[0] << ";" << domf[1] << ";" << RzCam1.at<float>(index)<< ";" << RzCam2.at<float>(index) << ";" << RzKal1_current[0] << ";" << RzKal2_current[0] << ";" << RzKal1_current[1] << ";" << RzKal2_current[1] << ";" << pw[leftM].curr_pos << ";" << pw[!leftM].curr_pos << endl;
			else	csvfile << (getTickCount()-start)/getTickFrequency() << ";" << RzCam1.at<float>(index) << ";" << KalmanR1.at<float>(index) << endl;
		}
		
		if (index>50 && !camready) {
			start = tick;
			startpt[0] = RzKal1_current[0]; startpt[1] = RzKal2_current[0];
			camready = true;
		}

		key=cv::waitKey(1);//wait for key to be pressed
	}while(key!=27 && TheVideoCapturer.grab());

  /// CLOSING
	MainThreadHasFinished = true;
	while ( !DispThreadHasFinished ) {
		waitKey(50);
	}
	if(WMOT){
		pw[0].closeMot();
		if(WMOT>1)	pw[1].closeMot();
	}
	if(REC)	csvfile.close();

	return 0;
}
