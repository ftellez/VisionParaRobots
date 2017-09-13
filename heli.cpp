#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"
/*
 * A simple 'getting started' interface to the ARDrone, v0.2 
 * author: Tom Krajnik
 * The code is straightforward,
 * check out the CHeli class and main() to see 
 */
#include <stdlib.h>
#include "CHeli.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

bool stop = false;
CRawImage *image;
CHeli *heli;
float pitch, roll, yaw, height;
int hover=0;
// Joystick related
SDL_Joystick* m_joystick;
bool useJoystick;
int joypadRoll, joypadPitch, joypadVerticalSpeed, joypadYaw;
bool navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover;
string ultimo = "init";

int Px;
int Py;
int vR;
int vG;
int vB;
/////////Ane
int vH;
int vS;
int vV;
int vY;
int vQ;
int vI;

int thresh = 100;
int max_thresh = 255;
////////////

Mat imagenClick;
bool saved;
bool showFreezed;
int clickminR = 256;
int clickmaxR = -1;
int clickminG = 256;
int clickmaxG = -1;
int clickminB = 256;
int clickmaxB = -1;
int clickminH = 256;
int clickmaxH = -1;
int clickminS = 256;
int clickmaxS = -1;
int clickminV = 256;
int clickmaxV = -1;
int clickmaxH30 = -1;
int clickminH150 = 256;
int clickminY = 256;
int clickmaxY = -1;
int clickminCr = 256;
int clickmaxCr = -1;
int clickminCb = 256;
int clickmaxCb = -1;


////////////Sandra
Mat HSVimage;
Mat YIQimage;
int blue_slider;
int green_slider;
int red_slider;
float meanB;
float meanG;
float meanR;
int hue_slider;
int sat_slider;
int val_slider;
float meanH;
float meanS;
float meanV;
int y_slider;
int cr_slider;
int cb_slider;
float meanY;
float meanCr;
float meanCb;

/////los filtros
Mat currentImage;
Mat maskBGR;
Mat maskHSV;
Mat helperHSV;
Mat maskYCrCb;
Mat filterBGR;
Mat filterHSV;
Mat filterYCrCb;

///////////////////////////////////////////////////////////////////

void setToBlack(Mat &orig, Mat &binMask, int y) {
    for (int i = 0; i < binMask.rows; i++)
    {
	for (int j = 0; j < binMask.cols; j++) {
	    if (binMask.at<uchar>(i, j) == 0) {
		orig.at<Vec3b>(i,j)[0] = 0;
		orig.at<Vec3b>(i,j)[1] = y;
		orig.at<Vec3b>(i,j)[2] = y;
	    }
	}
    }
}

void restOf1(Mat &helper, Mat &mask) {
    for (int i = 0; i < helper.rows; i++)
    {
	for (int j = 0; j < helper.cols; j++) {
	    if (helper.at<uchar>(i, j) == 256) {
		mask.at<uchar>(i,j) = 256;
	    }
	}
    }
}
// Convert CRawImage to Mat
void rawToMat( Mat &destImage, CRawImage* sourceImage)
{	
	uchar *pointerImage = destImage.ptr(0);
	
	for (int i = 0; i < 240*320; i++)
	{
		pointerImage[3*i] = sourceImage->data[3*i+2];
		pointerImage[3*i+1] = sourceImage->data[3*i+1];
		pointerImage[3*i+2] = sourceImage->data[3*i];
	}
}

void histogram(Mat &srcPlane, Mat &histDraw, float* range, Scalar color)
{
    int histSize = 16;
    const float* histRange = range;
    Mat hist;
    // Compute the histograms:
    calcHist(&srcPlane, 1, 0, Mat(), hist, 1, &histSize, &histRange, true, false);
    int bin_w = cvRound( (double) 512/histSize);
    /// Normalize the result to [ 0, histImage.rows ]
    normalize(hist, hist, 0, histDraw.rows, NORM_MINMAX, -1, Mat());
    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
	line(histDraw, Point( bin_w*(i-1), 400 - cvRound(hist.at<float>(i-1)) ) ,
    	    Point( bin_w*(i), 400 - cvRound(hist.at<float>(i)) ),
	    color, 2, 8, 0  );
    }
}

//Add bars to histograms
Mat joinImages(Mat im1, Mat im2){
    Size sz1 = im1.size();
    Size sz2 = im2.size();
    Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
    Mat left(im3, Rect(0, 0, sz1.width, sz1.height));
    im1.copyTo(left);
    Mat right(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
    im2.copyTo(right);
    //imshow("histogram", im3);
	return im3;
}

//Color bar
void drawColorBar(Mat& imagen, bool blue, bool green, bool red, int selectedPoint ){
    /*Aim:- Generate a 20x255 colorbar starting at point (200,50)*/
	int b=0,g=0,r=0;//start at yellow
    int y=50;//start at y=50, then increment
	Scalar color;
	if(blue){
	    while(b<256)//run till green color reaches 0
        {
            b++;
            y++;
			if(b == selectedPoint){
				color=Scalar(0,255,255);
			} else {
			    color=Scalar(b,g,r);	
			}
            rectangle(imagen,Point(50,y),Point(70,y+1),color,1);
	    }
	}
	
	if(green){
	    while(g<256)//run till green color reaches 0
        {
            g++;
            y++;
            if(g == selectedPoint){
				color=Scalar(0,255,255);
			} else {
			    color=Scalar(b,g,r);	
			}
            rectangle(imagen,Point(50,y),Point(70,y+1),color,1);
	    }	
	}
	
	if(red){
	    while(r<256)//run till green color reaches 0
        {
            r++;
            y++;
            if(r == selectedPoint){
				color=Scalar(0,255,255);
			} else {
			    color=Scalar(b,g,r);	
			}
            rectangle(imagen,Point(50,y),Point(70,y+1),color,1);
	    }	
	}
}

//codigo del click en pantalla
void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param)
{
    uchar* destination;
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            Px=x;
            Py=y;
            destination = (uchar*) imagenClick.ptr<uchar>(Py);
            vB=destination[Px * 3];
            vG=destination[Px*3+1];
            vR=destination[Px*3+2];
			
			if(saved == false){
               //saved = true;
               showFreezed = true;
            }
			
            break;
        case CV_EVENT_MOUSEMOVE:
            break;
        case CV_EVENT_LBUTTONUP:
            break;
        case CV_EVENT_RBUTTONDOWN:
        //flag=!flag;
            break;
    }
}

void colorOfPoint(int event, int x, int y, int flags, void* param)
{
    uchar* destination;
    Vec3b HSVpoint, YCrCbpoint;
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            Px=x;
            Py=y;
            destination = (uchar*) currentImage.ptr<uchar>(Py);
            vB=destination[Px * 3];
            vG=destination[Px*3+1];
            vR=destination[Px*3+2];
	    HSVpoint = HSVimage.at<Vec3b>(Py, Px);
	    YCrCbpoint = YIQimage.at<Vec3b>(Py, Px);
	    if(vB < clickminB ){clickminB = vB;}
	    if(vG < clickminG ){clickminG = vG;}
	    if(vR < clickminR ){clickminR = vR;}
	    if(vB > clickmaxB ){clickmaxB = vB;}
	    if(vG > clickmaxG ){clickmaxG = vG;}
	    if(vR > clickmaxR ){clickmaxR = vR;}

	    if(HSVpoint[0] < clickminH ){clickminH = HSVpoint[0];}
	    if(HSVpoint[1] < clickminS ){clickminS = HSVpoint[1];}
	    if(HSVpoint[2] < clickminV ){clickminV = HSVpoint[2];}
	    if(HSVpoint[0] > clickmaxH ){clickmaxH = HSVpoint[0];}
	    if(HSVpoint[1] > clickmaxS ){clickmaxS = HSVpoint[1];}
	    if(HSVpoint[2] > clickmaxV ){clickmaxV = HSVpoint[2];}
//
	    if(HSVpoint[0] >= 150 && HSVpoint[0] < clickminH150){clickminH150 = HSVpoint[0];}
	    if(HSVpoint[0] <= 30 && HSVpoint[0] > clickmaxH30 ){clickmaxH30 = HSVpoint[0];}
//

	    if(YCrCbpoint.val[0] < clickminY ){clickminY = YCrCbpoint.val[0];}
	    if(YCrCbpoint.val[1] < clickminCr){clickminCr = YCrCbpoint.val[1];}
	    if(YCrCbpoint.val[2] < clickminCb){clickminCb = YCrCbpoint.val[2];}
	    if(YCrCbpoint.val[0] > clickmaxY ){clickmaxY = YCrCbpoint.val[0];}
	    if(YCrCbpoint.val[1] > clickmaxCr){clickmaxCr = YCrCbpoint.val[1];}
	    if(YCrCbpoint.val[2] > clickmaxCb){clickmaxCb = YCrCbpoint.val[2];}

	    inRange(currentImage, Scalar(clickminB-3, clickminG-3, clickminR-3), Scalar(clickmaxB+3, clickmaxG+3, clickmaxR+3), maskBGR);
	    filterBGR = currentImage.clone();
	    setToBlack(filterBGR, maskBGR, 0);
	    imshow("Filter BGR", filterBGR);
//
	    if (clickminH150 != 256 && clickmaxH30 != -1) {
		inRange(HSVimage, Scalar(0, clickminS-3, clickminV-3), Scalar(clickmaxH30, clickmaxS+3, clickmaxV+3), helperHSV);
		inRange(HSVimage, Scalar(clickminH150, clickminS-3, clickminV-3), Scalar(180, clickmaxS+3, clickmaxV+3), maskHSV);
		restOf1(helperHSV, maskHSV);
	    } else {
		inRange(HSVimage, Scalar(clickminH-2, clickminS-3, clickminV-3), Scalar(clickmaxH+2, clickmaxS+3, clickmaxV+3), maskHSV);
	    }
	    filterHSV = HSVimage.clone();
	    setToBlack(filterHSV, maskHSV, 0);
	    cvtColor(filterHSV, filterHSV, COLOR_HSV2BGR);
	    imshow("Filter HSV", filterHSV);
	    inRange(YIQimage, Scalar(clickminY-3, clickminCr-3, clickminCb-3), Scalar(clickmaxY+3, clickmaxCr+3, clickmaxCb+3), maskYCrCb);
	    filterYCrCb = YIQimage.clone();
	    setToBlack(filterYCrCb, maskYCrCb, 128);
	    cvtColor(filterYCrCb, filterYCrCb, COLOR_YCrCb2BGR);
	    imshow("Filter YCrCb", filterYCrCb);
            break;

        case CV_EVENT_MOUSEMOVE:
            break;
        case CV_EVENT_LBUTTONUP:
            break;
        case CV_EVENT_RBUTTONDOWN:
        //flag=!flag;
            break;
        
    }
}

/////////////////////////////////////////////Ane
void contours(Mat myGrayImage)
{
	//contour
		RNG rng(12345);
		Mat canny_output, newGray;
		myGrayImage.copyTo(newGray);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		/// Detect edges using canny
		Canny( newGray, canny_output, thresh, thresh*2, 3 );
		/// Find contours				
		findContours( canny_output, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		namedWindow( "Canny");
		imshow( "Canny", canny_output );

		//Draw contours with random colors/
		  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
		  for( int i = 0; i< contours.size(); i++ )
			 {
			   Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			   //drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
				 drawContours(drawing, contours, i, color, CV_FILLED);
			 }
		  /// Show in a window
		  namedWindow( "Contours" );
		  imshow( "Contours", drawing );
}

void intensityValues(Mat histImage, int Px, int Py, int vColor)
{
	Scalar color = Scalar(0, 255, 255);
	rectangle(histImage, Point(vColor-2, -300), Point(vColor+2, 300), color, 1);
	
}
////////////////////////////////////////////////////////////////////////////


int main(int argc,char* argv[])
{
    //establishing connection with the quadcopter
    heli = new CHeli();
	
    //this class holds the image from the drone	
    image = new CRawImage(320,240);
	
    // Initial values for control	
    pitch = roll = yaw = height = 0.0;
    joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0.0;

    // Destination OpenCV Mat	
    currentImage = Mat(240, 320, CV_8UC3);
	Mat currentVideo = Mat(240, 320, CV_8UC3);
	namedWindow("ParrotCam", 0);
    // Show it	
    imshow("ParrotCam", currentVideo);

    //image is captured
    heli->renewImage(image);
	
	//saving and freeze
    saved = false;
    showFreezed = false;

    // Copy to OpenCV Mat
    rawToMat(currentVideo, image);
    imshow("ParrotCam", currentVideo);

    //self made black and white image with luminosity method
    Mat myGrayImage(currentImage.rows, currentImage.cols, CV_8UC1, Scalar(0));
    Vec3b intensity;
    Mat binarizedImage;

    // Initialize joystick
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    useJoystick = SDL_NumJoysticks() > 0;
    if (useJoystick)
    {
        SDL_JoystickClose(m_joystick);
        m_joystick = SDL_JoystickOpen(0);
    }

    namedWindow("Click", 0);
    setMouseCallback("Click", mouseCoordinatesExampleCallback);
	
    while (stop == false)
    {

        // Clear the console
        printf("\033[2J\033[1;1H");

        if (useJoystick)
        {
            SDL_Event event;
            SDL_PollEvent(&event);

            joypadRoll = SDL_JoystickGetAxis(m_joystick, 2);
            joypadPitch = SDL_JoystickGetAxis(m_joystick, 3);
            joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1);
            joypadYaw = SDL_JoystickGetAxis(m_joystick, 0);
            joypadTakeOff = SDL_JoystickGetButton(m_joystick, 1);
            joypadLand = SDL_JoystickGetButton(m_joystick, 2);
            joypadHover = SDL_JoystickGetButton(m_joystick, 0);
        }

        // prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
        printf("===================== Parrot Basic Example =====================\n\n");
        fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
        fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
        fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
        fprintf(stdout, "Hover   : %d \n", hover);
        fprintf(stdout, "Joypad  : %d \n", useJoystick ? 1 : 0);
        fprintf(stdout, "  Roll    : %d \n", joypadRoll);
        fprintf(stdout, "  Pitch   : %d \n", joypadPitch);
        fprintf(stdout, "  Yaw     : %d \n", joypadYaw);
        fprintf(stdout, "  V.S.    : %d \n", joypadVerticalSpeed);
        fprintf(stdout, "  TakeOff : %d \n", joypadTakeOff);
        fprintf(stdout, "  Land    : %d \n", joypadLand);
        fprintf(stdout, "Navigating with Joystick: %d \n", navigatedWithJoystick ? 1 : 0);
        cout<<"Pos X: "<<Px<<" Pos Y: "<<Py<<" \nValor RGB: ("<<vR<<","<<vG<<","<<vB<<")"<<endl;
		cout<<"Valor HSV: ("<<vH<<","<<vS<<","<<vV<<")"<<endl; ////////////Ane
        cout<<"\nValor Min H: "<<clickminH<<"\nValor Min S: "<<clickminS<<"\nValor Min V: "<<clickminV;
		cout<<"\nValor Max H: "<<clickmaxH<<"\nValor Max S: "<<clickmaxS<<"\nValor Max V: "<<clickmaxV;
	    cout<<"\nPrueba Sandra"<<endl;
		
	
	    //image is captured
	    heli->renewImage(image);

	    // Copy to OpenCV Mat
	    rawToMat(currentVideo, image);
	    imshow("ParrotCam", currentVideo);
		resizeWindow("ParrotCam", 265, 200);
		
		//Window to activate analysis
		imagenClick=currentVideo;
        imshow("Click", imagenClick);
        resizeWindow("Click", 265, 200);
		
		if(showFreezed) {
			currentImage = currentVideo.clone();
			
			//Saved image
			namedWindow("Saved", 0);
			setMouseCallback("Saved", colorOfPoint);
			
			//Binarized window
			namedWindow("Binarized Image", 0);

	        //Gray Window
	        namedWindow("My Gray", 0);
	
            //YIQ
            namedWindow("YCrCb", 0);

            //HSV
            namedWindow("HSV", 0);
	
        	//histograms
	        namedWindow("BGR-B", 0);
	        namedWindow("BGR-G", 0);
	        namedWindow("BGR-R", 0);
			namedWindow("HSV-H", 0);
			namedWindow("HSV-S", 0);
			namedWindow("HSV-V", 0);
			namedWindow("YCrCb-Y", 0);
			namedWindow("YCrCb-Cr", 0);
			namedWindow("YCrCb-Cb", 0);
			
            imshow("Saved", currentImage);
			resizeWindow("Saved", 265, 200);
			
			//Start here
	        //self made black and white image with luminosity method
	        for (int i = 0; i < myGrayImage.rows; i++)
     	    {
	            for (int j = 0; j < myGrayImage.cols; j++) {
      	    	    intensity = currentImage.at<Vec3b>(i, j);
        	    	myGrayImage.at<uchar>(i, j) = (unsigned int) 0.07*intensity.val[0]+0.72*intensity.val[1]+0.21*intensity.val[2];
	            }
	        }
	    
		    imshow("My Gray", myGrayImage);
		    resizeWindow("My Gray", 265, 200);
	        threshold(myGrayImage, binarizedImage, 128, 255, 0);
	        imshow("Binarized Image", binarizedImage);
		    resizeWindow("Binarized Image", 265, 200);
	        cvtColor(currentImage, YIQimage, COLOR_BGR2YCrCb);
	        imshow("YCrCb", YIQimage);
		    resizeWindow("YCrCb", 265, 200);
	        cvtColor(currentImage, HSVimage, COLOR_BGR2HSV);
	        imshow("HSV", HSVimage);
		    resizeWindow("HSV", 265, 200);
	        
			//contours
			contours(myGrayImage);  /////////Ane

	        //histograms
	        //bgr image
	        Mat histImageB(400, 512, CV_8UC3, Scalar(0,0,0));
	        Mat histImageG(400, 512, CV_8UC3, Scalar(0,0,0));
	        Mat histImageR(400, 512, CV_8UC3, Scalar(0,0,0));
			Mat barImageB(400, 100, CV_8UC3);
			Mat barImageG(400, 100, CV_8UC3);
			Mat barImageR(400, 100, CV_8UC3);
			Mat joinedHistB;
			Mat joinedHistG;
			Mat joinedHistR;
	        vector<Mat> bgr_planes;
	        split(currentImage, bgr_planes);
	        float range0256[] = {0, 256};
	        histogram(bgr_planes[0], histImageB, range0256, Scalar(255, 0, 0));
			intensityValues(histImageB, Px, Py, vB ); ////////////////Ane
	        histogram(bgr_planes[1], histImageG, range0256, Scalar(0, 255, 0));
			intensityValues(histImageG, Px, Py, vG ); ////////////////Ane
	        histogram(bgr_planes[2], histImageR, range0256, Scalar(0, 0, 255));
			intensityValues(histImageR, Px, Py, vR ); ////////////////Ane
			drawColorBar(barImageB, true, false, false, vB);
			joinedHistB = joinImages(histImageB, barImageB);
	        imshow("Histogram Blue", joinedHistB);
			drawColorBar(barImageG, false, true, false, vG);
			joinedHistG = joinImages(histImageG, barImageG);
	        imshow("Histogram Green", joinedHistG);
			drawColorBar(barImageR, false, false, true, vR);
			joinedHistR = joinImages(histImageR, barImageR);
	        imshow("Histogram Red", joinedHistR);
		    resizeWindow("Histogram Blue", 400, 250);
		    resizeWindow("Histogram Green", 400, 250);
		    resizeWindow("Histogram Red", 400, 250);
			
			////////////////////////////////Ane
			//hsv image
			vector<Mat> hsv_planes;
			split(HSVimage, hsv_planes);
			float h_ranges[] = { 0, 256 };
			float s_ranges[] = { 0, 180 };
			float v_ranges[] = { 0, 256 };
			Mat histImageH(400, 512, CV_8UC3, Scalar(0,0,0));
	        Mat histImageS(400, 512, CV_8UC3, Scalar(0,0,0));
	        Mat histImageV(400, 512, CV_8UC3, Scalar(0,0,0));
			histogram( hsv_planes[0], histImageH, h_ranges, Scalar(255, 255, 255) );
			intensityValues(histImageB, Px, Py, vH );
			histogram( hsv_planes[1], histImageS, s_ranges, Scalar(255, 255, 255) );
			intensityValues(histImageB, Px, Py, vS );
			histogram( hsv_planes[2],  histImageV, v_ranges, Scalar(255, 255, 255) );
			intensityValues(histImageB, Px, Py, vV );
			imshow("HSV-H", histImageH);
			imshow("HSV-S", histImageS);
			imshow("HSV-V", histImageV);
			resizeWindow("HSV-H", 325, 250);
			resizeWindow("HSV-S", 325, 250);
			resizeWindow("HSV-V", 325, 250);
			
			//YIQ image
	        vector<Mat> yiq_planes;
	        split(YIQimage, yiq_planes);
			Mat histImageY(400, 512, CV_8UC3, Scalar(0,0,0));
	        Mat histImageI(400, 512, CV_8UC3, Scalar(0,0,0));
	        Mat histImageQ(400, 512, CV_8UC3, Scalar(0,0,0));
	        float range02561[] = {0, 256};
	        histogram(yiq_planes[0], histImageY, range02561, Scalar(255, 255, 255));
	        histogram(yiq_planes[1], histImageI, range02561, Scalar(255, 255, 255));
	        histogram(yiq_planes[2], histImageQ, range02561, Scalar(255, 255, 255));
		    
		    imshow("YIQ-Y", histImageY);
		    imshow("YIQ-I", histImageI);
		    imshow("YIQ-Q", histImageQ);
		    resizeWindow("YIQ-Y", 325, 250);
		    resizeWindow("YIQ-I", 325, 250);
		    resizeWindow("YIQ-Q", 325, 250);

			///filters
			namedWindow("Filter BGR", 0);
			resizeWindow("Filter BGR", 265, 200);
			namedWindow("Filter HSV", 0);
			resizeWindow("Filter HSV", 265, 200);
			namedWindow("Filter YCrCb", 0);
			resizeWindow("Filter YCrCb", 265, 200);
			//////////////////////////////////////////
			//////////////////////////////////////////
			
			/////////////////////////////////////////////////////////////////Sandra
/*
			//umbrales BGR
		    meanB = mean(bgr_planes[0]);
		    meanG = mean(bgr_planes[1]);
		    meanR = mean(bgr_planes[2]);
		    namedWindow("Treshold BGR", 0);
		    resizeWindow("Treshold BGR", 265, 400);
		    blue_slider = 0;
		    int max_gradient_blue = (128 > meanB) ? (int) meanB : (int) 256-meanB;
		    green_slider = 0;
		    int max_gradient_green = (128 > meanG) ? (int) meanG : (int) 256-meanG;
		    red_slider = 0;
		    int max_gradient_red = (128 > meanR) ? (int) meanR : (int) 256-meanR;
		    createTrackbar("B", "Treshold BGR", &blue_slider, max_gradient_blue, on_trackbar, (void*)&currentImage);
		    createTrackbar("G", "Treshold BGR", &green_slider, max_gradient_green, on_trackbar, (void*)&currentImage);
		    createTrackbar("R", "Treshold BGR", &red_slider, max_gradient_red, on_trackbar, (void*)&currentImage);

		    //umbrales HSV
		    meanH = mean(hsv_planes[0]);
		    meanS = mean(hsv_planes[1]);
		    meanV = mean(hsv_planes[2]);
		    namedWindow("Treshold HSV", 0);
		    resizeWindow("Treshold HSV", 265, 400);
		    hue_slider = 0;
		    int max_gradient_hue = (90 > meanH) ? (int) meanH : (int) 180-meanH;
		    sat_slider = 0;
		    int max_gradient_sat = (128 > meanS) ? (int) meanS : (int) 256-meanS;
		    val_slider = 0;
		    int max_gradient_val = (128 > meanR) ? (int) meanV : (int) 256-meanV;
		    createTrackbar("H", "Treshold HSV", &hue_slider, max_gradient_hue, on_trackbar2, (void*)&HSVimage);
		    createTrackbar("S", "Treshold HSV", &sat_slider, max_gradient_sat, on_trackbar2, (void*)&HSVimage);
		    createTrackbar("V", "Treshold HSV", &val_slider, max_gradient_val, on_trackbar2, (void*)&HSVimage);

		    //umbrales YCrCb
		    meanY = mean(yiq_planes[0]);
		    meanCr= mean(yiq_planes[1]);
		    meanCb = mean(yiq_planes[2]);
		    namedWindow("Treshold YCrCb", 0);
		    resizeWindow("Treshold YCrCb", 265, 400);
		    y_slider = 0;
		    int max_gradient_y = (128 > meanY) ? (int) meanY : (int) 256-meanY;
		    cr_slider = 0;
		    int max_gradient_cr = (128 > meanCr) ? (int) meanCr : (int) 256-meanCr;
		    cb_slider = 0;
		    int max_gradient_cb = (128 > meanCb) ? (int) meanCb : (int) 256-meanCb;
		    createTrackbar("Y", "Treshold YCrCb", &y_slider, max_gradient_y, on_trackbar3, (void*)&YIQimage);
		    createTrackbar("Cr", "Treshold YCrCb", &cr_slider, max_gradient_cr, on_trackbar3, (void*)&YIQimage);
		    createTrackbar("Cb", "Treshold YCrCb", &cb_slider, max_gradient_cb, on_trackbar3, (void*)&YIQimage);
            /////////////////////////////////////////////////////////////////////////
			*/
			
			showFreezed = false;
        }

	   

        char key = waitKey(5);
	    switch (key) {
		    case 'a': yaw = -20000.0; break;
		    case 'd': yaw = 20000.0; break;
		    case 'w': height = -20000.0; break;
		    case 's': height = 20000.0; break;
		    case 'q': heli->takeoff(); break;
		    case 'e': heli->land(); break;
		    case 'z': heli->switchCamera(0); break;
		    case 'x': heli->switchCamera(1); break;
		    case 'c': heli->switchCamera(2); break;
		    case 'v': heli->switchCamera(3); break;
		    case 'j': roll = -20000.0; break;
		    case 'l': roll = 20000.0; break;
		    case 'i': pitch = -20000.0; break;
		    case 'k': pitch = 20000.0; break;
    		case 'h': hover = (hover + 1) % 2; break;
    		case 27: stop = true; break;
    		default: pitch = roll = yaw = height = 0.0;
	    }

        if (joypadTakeOff) {
            heli->takeoff();
        }
        if (joypadLand) {
            heli->land();
        }
        //hover = joypadHover ? 1 : 0;

        //setting the drone angles
        if (joypadRoll != 0 || joypadPitch != 0 || joypadVerticalSpeed != 0 || joypadYaw != 0)
        {
            heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, hover);
            navigatedWithJoystick = true;
        }
        else
        {
            heli->setAngles(pitch, roll, yaw, height, hover);
            navigatedWithJoystick = false;
        }

        usleep(15000);
	}
	
	heli->land();
	SDL_JoystickClose(m_joystick);
	delete heli;
	delete image;
	return 0;
}
