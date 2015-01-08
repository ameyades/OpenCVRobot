#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "wiringPi.h"
#include  <iostream> 
#include <cmath>
#include <sstream>
#include <cstring>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;



void setup() {
	wiringPiSetup();
}



 int main( int argc, char** argv )
 {
 
 
 	FILE *fp;
        fp = fopen("/dev/servoblaster", "w");
        if (fp == NULL) {
                printf("Error opening file\n");
                exit(0);
        }
	setup();
 
    VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

cap.set(CV_CAP_PROP_FRAME_WIDTH, 160);
cap.set(CV_CAP_PROP_FRAME_HEIGHT, 120);

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  int iLowH = 170;
 int iHighH = 179;

  int iLowS = 150; 
 int iHighS = 255;

  int iLowV = 60;
 int iHighV = 255;

  //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

  createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

  int iLastX = -1; 
 int iLastY = -1;

  //Capture a temporary image from the camera
 Mat imgTmp;
    imgTmp = Mat::zeros(301, 301, CV_8UC3);

 cap.read(imgTmp); 

  //Create a black image with the size as the camera output
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
 

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

    Mat imgHSV;

   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholded;

   inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

   //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

   //Calculate the moments of the thresholded image
  Moments oMoments = moments(imgThresholded);

   double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;

   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 10000)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;        
        
   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
    //Draw a red line from the previous point to the current point
    line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
	cout << "Position X, Y: " << posX << " " << posY << "  Last X, Y: " << iLastX << " " <<iLastY <<endl;  
	/*if(dArea <45000 )
	{
		std::cout<<"Going forward!" << std::endl;
		fprintf(fp, "1=182\n");
		fprintf(fp, "0=100\n");
		fflush(fp);
		delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(50);
	}
	else if(dArea > 45000)
	{
		std::cout<<"Going backward!" << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=100\n");
		fflush(fp);
		delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(50);
	}*/
	//Left & Right

	if(dArea <35000)
	{
		std::cout<<"Going forward!" << std::endl;
		fprintf(fp, "1=182\n");
		fprintf(fp, "0=100\n");
		fflush(fp);
		delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
		if(posX > iLastX + 2 )
		{	
		std::cout<<"Turning right " << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=182\n");	
        fflush(fp);
        delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
		}
		else if(posX < iLastX)
		{
		std::cout<<"Turning left " << std::endl;
		
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(100);
		
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
		delay(100);
		}
		//Forward & Backward
	}
	else if(dArea > 45000)
	{
		std::cout<<"Going backward!" << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=100\n");
		fflush(fp);
		delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
		if(posX > iLastX + 2 )
		{	
		std::cout<<"Turning right " << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=182\n");	
        fflush(fp);
        delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
		}
		else if(posX < iLastX)
		{
		std::cout<<"Turning left " << std::endl;
		
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(100);
		
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
		delay(100);
		}
	}












/*
	if(posX > iLastX + 2 )
	{
	
		std::cout<<"Turning right " << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=182\n");	
        fflush(fp);
        delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
		//Forward & Backward
	if(dArea <35000)
	{
		std::cout<<"Going forward!" << std::endl;
		fprintf(fp, "1=182\n");
		fprintf(fp, "0=100\n");
		fflush(fp);
		delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
	}
	else if(dArea > 45000)
	{
		std::cout<<"Going backward!" << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=100\n");
		fflush(fp);
		delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
	}
		
	} 
	else if(posX < iLastX)
	{
		std::cout<<"Turning left " << std::endl;
		
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(100);
		
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
		delay(100);
		if(dArea <45000 )
	{
		std::cout<<"Going forward!" << std::endl;
		fprintf(fp, "1=182\n");
		fprintf(fp, "0=100\n");
		fflush(fp);
		delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
	}
	else if(dArea > 45000)
	{
		std::cout<<"Going backward!" << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=100\n");
		fflush(fp);
		delay(100);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
        fflush(fp);
		delay(100);
	}
	}
*/
	delay(200);
    //here
   }

    iLastX = posX;
   iLastY = posY;
  }

   imshow("Thresholded Image", imgThresholded); //show the thresholded image

   imgOriginal = imgOriginal + imgLines;
  imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }

   return 0;
}





/* #include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

 int main( int argc, char** argv )
 {
    VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

cap.set(CV_CAP_PROP_FRAME_WIDTH, 160);
cap.set(CV_CAP_PROP_FRAME_HEIGHT, 120);

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  int iLowH = 170;
 int iHighH = 179;

  int iLowS = 150; 
 int iHighS = 255;

  int iLowV = 60;
 int iHighV = 255;

  //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

  createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

  int iLastX = -1; 
 int iLastY = -1;

  //Capture a temporary image from the camera
 Mat imgTmp;
    imgTmp = Mat::zeros(301, 301, CV_8UC3);

 cap.read(imgTmp); 

  //Create a black image with the size as the camera output
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
 

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

    Mat imgHSV;

   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholded;

   inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

   //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

   //Calculate the moments of the thresholded image
  Moments oMoments = moments(imgThresholded);

   double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;

   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 10000)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;        
        
   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
    //Draw a red line from the previous point to the current point
    line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
   }

    iLastX = posX;
   iLastY = posY;
  }

   imshow("Thresholded Image", imgThresholded); //show the thresholded image

   imgOriginal = imgOriginal + imgLines;
  imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }

   return 0;
}
*/
