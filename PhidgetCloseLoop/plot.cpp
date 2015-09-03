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

//#include "ArucoTest.h"

using namespace std;
using namespace cv;
using namespace aruco;

bool DispThreadHasFinished = false;
bool StepThreadHasFinished = false;
bool MainThreadHasFinished = false;
bool camready = false;
int w = 1200; int h = 400; int leftM = 0;
Mat plotImage1( h, w, CV_8UC3, Scalar( 0,0,0) );
Mat plotImage2( h, w, CV_8UC3, Scalar( 0,0,0) );
double tick = 0.0, start = 0.0, RzKal_current = 0.0, dt=40.0/1000.0;
CPhidgetWrapper pw[2];
tk::spline s1,s2;
double t_max=-1;

#define WMOT 1	//0,1,2
#define REC 0

struct PT {
	double x;
	double y;
};

double motor2body(double ang)
{
	return -ang*19/115.0;
}
double body2motor(double ang)
{
	double Irot=0.0000054, Ifly=0.000154, Ibase=0.00164; //kg.m2
	double Iaj=3*0.05*pow(0.06,2); //piles de washers 50g
	double y = -(Irot+19/115*(Ifly+Iaj))*ang/(Ibase+Ifly+Iaj+Irot);
	
	return -ang*115.0/15.0;
}
double softstep(double t)
{
	double a[3]={1.9635,-1.4726,0.2945};
	t_max=2;

	return a[0]*pow(t,3)+a[1]*pow(t,4)+a[2]*pow(t,5);
}

void ThreadDisplay() {
	/// Display
	namedWindow("Plot Curves 1", CV_WINDOW_AUTOSIZE );
	namedWindow("Plot Curves 2", CV_WINDOW_AUTOSIZE );
	
	while ( !MainThreadHasFinished ) {
		/// Update
		imshow("Plot Curves 1", plotImage1 );
		imshow("Plot Curves 2", plotImage2 );
		waitKey(10);
	}
	
	DispThreadHasFinished=true;
	return;
}
void ThreadMot() {
	int stepcommand=0;
	double Rzerror_old = 0.0, Rzerror_int = 0.0;
	while ( !MainThreadHasFinished ) {
		double t = (getTickCount()-start)/getTickFrequency();
		double target1 = softstep(t);//s1(t);
		double target2 = PI;//s2(t);
		if(camready && WMOT>0)
		{
			// Compute error
			double Rzerror = target1-RzKal_current;
			if(t_max!=-1){	//stop the run if over max time
				if(t>t_max){
					target1 = softstep(t_max);//s1(t_max);
					if(WMOT>1)	target2 = PI;//s2(t_max);
					Rzerror = 0;Rzerror_int = 0;
				}
			}
			double Rzerror_der = (Rzerror-Rzerror_old)/dt;
			Rzerror_int += Rzerror*dt;
			if(abs(Rzerror_int)>1.6)	Rzerror_int=sgn(Rzerror_int)*1.6;
			// Run the stepper
			//__int64 stepcommand = pw.rad2steps(body2motor(10*Rzerror+0.1*Rzerror_int+2*Rzerror_der)); //2 1.5 0.05
			//if(abs(stepcommand)>10000)	stepcommand=sgn(stepcommand)*10000;
			//pw.GoToStepper(0, stepcommand);
			Rzerror_old=Rzerror;
			// Run the dcmotor
			//cout << WMOT << "-t:" << t << ", r1:" << target1 << "(e:" << body2motor(Rzerror) << "), r2:" << target2 << endl;
			//pw[leftM].target=body2motor(target1);
			//pw[leftM].started=true;
			//pw[leftM].setvel(target1*25);
			pw[leftM].setvel(body2motor(150*Rzerror+5*Rzerror_int+2*Rzerror_der));
			if(WMOT>1)	pw[!leftM].target=-target2;
			if(WMOT>1)	pw[!leftM].started=true;
		}
		waitKey(dt*1000);
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
		t.push_back(tempt);r1.push_back(tempr1);r2.push_back(tempr2);
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
	char key=0;Mat rotation, rotation1Init, rotation2Init;
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
	KalmanFilter KF(3, 2, 0);
	// intialization of KF...
	KF.transitionMatrix = *(Mat_<float>(3, 3) << 1,dt,dt/2,   0,1,dt,  0,0,1);
	cout << KF.measurementMatrix.size() << KF.measurementNoiseCov.size() << endl;
	//setIdentity(KF.measurementMatrix);
	KF.measurementMatrix = *(Mat_<float>(2, 3) << 1,0,0,	0,1,0);
	Mat measurement = *(Mat_<float>(2, 1) << 0,0);
	//setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	KF.processNoiseCov = *(Mat_<float>(3, 3) << pow(dt,5)/20,pow(dt,4)/8,pow(dt,3)/6,   pow(dt,4)/8,pow(dt,3)/3,pow(dt,2)/2,  pow(dt,3)/6,pow(dt,2)/2,dt);
	//setIdentity(KF.measurementNoiseCov, Scalar::all(0.1));
	KF.measurementNoiseCov = *(Mat_<float>(2, 2) << 0.1,0,	0,10);
	setIdentity(KF.errorCovPost, Scalar::all(1));

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
	Mat RzCam1, RzCam2, RzStep, RzCom, KalmanR; RzCam1.create(plotSize,1,CV_32FC1); RzCam2.create(plotSize,1,CV_32FC1); RzCom.create(plotSize,1,CV_32FC1); KalmanR.create(plotSize,1,CV_32FC1);RzStep.create(plotSize,1,CV_32FC1); 
	RzCam1.at<float>(0)=0.0;RzCam2.at<float>(0)=0.0;RzStep.at<float>(0)=0.0;float scale=(float)h/(plotrange);

  /// Run stepper THREAD
	if(WMOT>0){
		pw[0].Init(0); // init 0 for dc motor, 1 for stepper
		if(WMOT>1)	pw[1].Init(0);
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

  /*
  * LOOP
  */
	do 
	{
		if(index>=plotSize-1){
			plotImage1=Mat::zeros( h, w, CV_8UC3 );
			plotImage2=Mat::zeros( h, w, CV_8UC3 );
			index=0;
		}

		TheVideoCapturer.retrieve(TheInputImage);//get image
		index++; //number of images captured
		tick = (double)getTickCount();//for checking the speed
		//Detection of markers in the image passed
		MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,0.046); //markers size 46mm

		//print marker info and draw the markers in image
		TheInputImage.copyTo(TheInputImageCopy);
	    
		if(TheMarkers.size()>0) {//for (unsigned int i=0;i<TheMarkers.size();i++) {
			if(rotation1Init.empty()){
				//Rvec1Init=TheMarkers[0].Rvec;
				Rodrigues(TheMarkers[0].Rvec,rotation1Init);
				RzCam1.at<float>(0)=atan2(-rotation1Init.at<float>(0,1),rotation1Init.at<float>(0,0));
				if(TheMarkers.size()>1){
					Rodrigues(TheMarkers[1].Rvec,rotation2Init);
					RzCam2.at<float>(0)=atan2(-rotation1Init.at<float>(0,1),rotation1Init.at<float>(0,0));
				}
				KF.statePre.at<float>(0) = RzCam1.at<float>(0);
				KalmanR.at<float>(0) = RzCam1.at<float>(0);
			}

			Rodrigues(TheMarkers[0].Rvec,rotation);//rotation=rotation-rotationInit;
			//cout<<"Rmat:"<<rotation<<endl;
			RzCam1.at<float>(index)=atan2(-rotation.at<float>(0,1),rotation.at<float>(0,0))-RzCam1.at<float>(0);
			if(TheMarkers.size()>1){
				Rodrigues(TheMarkers[1].Rvec,rotation);
				RzCam2.at<float>(index)=atan2(-rotation.at<float>(0,1),rotation.at<float>(0,0))-RzCam2.at<float>(0);
			}
			//cout<<"Rz:"<<RzCam.at<float>(index);
			measurement.at<float>(0)=RzCam1.at<float>(index);
			measurement.at<float>(1)=motor2body(pw[leftM].curr_vel);
			line( plotImage1, Point( bin_w*(index-1), h - cvRound((RzCam1.at<float>(index-1)+plotrange/2)*scale) ) ,
                       Point( bin_w*(index), h - cvRound((RzCam1.at<float>(index)+plotrange/2)*scale) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
			line( plotImage2, Point( bin_w*(index-1), h - cvRound((RzCam2.at<float>(index-1)+plotrange/2)*scale) ) ,
                       Point( bin_w*(index), h - cvRound((RzCam2.at<float>(index)+plotrange/2)*scale) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
		}
		//if (TheMarkers.size()!=0) cout<<endl;

		//draw a 3d cube in each marker if there is 3d info
		if (  TheCameraParameters.isValid())
			for (unsigned int i=0;i<TheMarkers.size();i++) {
				CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
				CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
			}
		//show input with augmented information and  the thresholded image
		cv::imshow("in",TheInputImageCopy);
		if(REC)	outputVideo.write(TheInputImageCopy);

		// Motor Pos 
		if(WMOT){
			RzStep.at<float>(index)=motor2body(pw[leftM].curr_pos)/10;
			//cout<<"stepper rz: "<<RzStep.at<float>(index);
			line( plotImage1, Point( bin_w*(index-1), h - cvRound((RzStep.at<float>(index-1)+plotrange/2)*scale) ) ,
							Point( bin_w*(index), h - cvRound((RzStep.at<float>(index)+plotrange/2)*scale) ),
							Scalar( 0, 255, 0), 2, 8, 0  );
			if(camready){
				if((tick-start)/getTickFrequency()<t_max) RzCom.at<float>(index)=softstep((tick-start)/getTickFrequency());
				else RzCom.at<float>(index)=softstep(t_max);
				//cout<<"stepper rz: "<<RzStep.at<float>(index);
				line( plotImage1, Point( bin_w*(index-1), h - cvRound((RzCom.at<float>(index-1)+plotrange/2)*scale) ) ,
							   Point( bin_w*(index), h - cvRound((RzCom.at<float>(index)+plotrange/2)*scale) ),
							   Scalar( 255, 255, 255), 2, 8, 0  );
			}
		}

		//chekc the speed by calculating the mean speed of all iterations
		AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
		AvrgTime.second++;
		dt=AvrgTime.first/AvrgTime.second;
		//cout<<"\rTime detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds nmarkers="<<TheMarkers.size()<<std::flush<<endl;

		// KF predict, to update the internal statePre variable
		KF.transitionMatrix = *(Mat_<float>(3, 3) << 1,dt,dt/2,   0,1,dt,  0,0,1);
		KF.processNoiseCov = *(Mat_<float>(3, 3) << pow(dt,5)/20,pow(dt,4)/8,pow(dt,3)/6,   pow(dt,4)/8,pow(dt,3)/3,pow(dt,2)/2,  pow(dt,3)/6,pow(dt,2)/2,dt);
		Mat prediction = KF.predict();
		// KF update
		Mat estimated = KF.correct(measurement);
		KalmanR.at<float>(index)=estimated.at<float>(0);RzKal_current=estimated.at<float>(0);
		line( plotImage1, Point( bin_w*(index-1), h - cvRound((KalmanR.at<float>(index-1)+plotrange/2)*scale) ) ,
					Point( bin_w*(index), h - cvRound((KalmanR.at<float>(index)+plotrange/2)*scale) ),
					Scalar( 0, 0, 255), 2, 8, 0  );

		if(index>20 && !camready){
			start=tick;
			camready= true;
		}

		if(camready && REC){
			if(WMOT)	csvfile << (getTickCount()-start)/getTickFrequency() << ";" << pw[leftM].target << ";" << pw[!leftM].target << ";" << RzCam1.at<float>(index) << ";" << KalmanR.at<float>(index) << ";" << pw[leftM].curr_pos << ";" << pw[!leftM].curr_pos << endl;
			else	csvfile << (getTickCount()-start)/getTickFrequency() << ";" << RzCam1.at<float>(index) << ";" << KalmanR.at<float>(index) << endl;
		}

		key=cv::waitKey(10);//wait for key to be pressed
	}while(key!=27 && TheVideoCapturer.grab());

  /// CLOSING
	if(WMOT){
		pw[0].started=false;
		if(WMOT>1)	pw[1].started=false;
	}
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
