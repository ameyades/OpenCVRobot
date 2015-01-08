#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "wiringPi.h"
#include  <iostream> 
#include <cmath>
#include <sstream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h> 
#include <string.h> 
#include <sys/ioctl.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <linux/i2c-dev.h> 
#include <math.h> 


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv/cv.h>

#include "Util.h"
#include "Util_Map.h"
#include "Particle.h"


#define TRIG 5


using namespace cv;
using namespace std;

//void drawParts(std::vector<Particle> &particles);


int main()

{
    char image_window[] = "Lab7funtime";

    namedWindow("result");
    //    namedWindow("result2");
    
    Mat original = Mat::zeros( 300, 300, CV_8UC3 );
    Mat image = Mat::zeros( 300, 300, CV_8UC3 );
    Mat image2;
    Mat image3;
    Mat image4;
    Mat image5;

    Mat kanye = imread("/Home Directory/ServoMotor/kanye.jpeg", 1);   // Read the file
    int numberOfParticles = 25;
    double variance = 10.0;
    vector<Particle> parts(numberOfParticles);
    int x=0,z=0;
    for(int i = 0;i<numberOfParticles;i++){
        //        x = (rand()%280)+10;
        //        z = (rand()%80)+5;
        //        cout <<"x: "<<x<<", z: "<<z<<endl;
        parts[i].setWeight(0.1);
        //        parts[i].moveParticle(z, x, 0.8);
    }
    
    parts[9].setPosition(cv::Point(5, 1));
    
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    std::vector<Point> edge;
    std::vector<Point> rock;
    std::vector<Point> dock;
    Point p;
    Particle mainpt(cv::Point(5,5));
    vector<Point> shootPoints(25);
    mainpt.setWeight(2.0);
    edge.push_back(Point(0,0));
    edge.push_back(Point(0,300));
    edge.push_back(Point(300,0));
    edge.push_back(Point(300,300));
    edge.push_back(Point(0,0));
    edge.push_back(Point(300,0));
    edge.push_back(Point(0,300));
    edge.push_back(Point(300,300));
    
    char ctrl;
    char ctrl2 = 'w';
    //    imshow("result", original);
    image = Scalar::all(0);
    image = original.clone();
    drawMap(image, edge, rock, dock, p);
    waitKey(500);
    //    cv::Rect roi(cv::Point(5,5),kanye.size());
    cv::Rect roi(mainpt.getPosition(),kanye.size());
    image2 = Scalar::all(0);
    image2 = image.clone();


    kanye.copyTo(image2(roi));
  
    drawParts(parts, image2, mainpt.getPosition());
    
    imshow("result", image2);
    ctrl2 = 'd';
    int show=0;
    int show1=0;
    cout <<"before"<<endl;
    double angle = 0.0;
    double movement = 0.0;
    while(true){
        updateProbability(parts, shootPoints, show);
        cout << "Enter key"<<endl;
        cin >> ctrl;
        if(ctrl == 'w'){
            //Up
            //   waitKey(0);
            //            cv::Rect roi(mainpt.getPosition(),kanye.size());
            if(ctrl == ctrl2){
                //                cout <<"Moving"<<endl;
                mainpt.moveParticle(270.0, 15, 0);
            }
            if(ctrl2 == 'a'){
                //turns left by 90˚ or right by 270˚
                angle = 180.0;
                movement = 0.0;
            }
            else if(ctrl2 == 's'){
                //turns around by 180˚. Turns left 180˚ or right 180˚
                angle = 90.0;
                movement = 0.0;
            }
            else if(ctrl2 == 'd'){
                //turns right by 90˚ or left by 270˚
                angle = 0.0;
                movement = 0.0;
            }
            else{
                //                cout <<"Moving"<<endl;
                angle = 270.0;
                movement = 15.0;
                //                mainpt.moveParticle(270.0, 15, 0);
            }
            cv::Rect roi(mainpt.getPosition(),kanye.size());
            Mat r = cv::getRotationMatrix2D(Point(25.0,25.0), 90, 1.0);
            Mat rotatedKanye;
            warpAffine(kanye, rotatedKanye, r, kanye.size());
            image2 = image.clone();
            rotatedKanye.copyTo(image2(roi));
            //            cout <<"showing"<<endl;
            for(int i = 0;i<parts.size();i++){
                parts[i].moveParticle(angle, movement, variance);
            }
            drawParts(parts, image2, mainpt.getPosition());
            imshow("result", image2);
            show = getOrientation(0, mainpt.getPosition());
            for(int i = 0; i<numberOfParticles;i++){
                show1 = getOrientation(0, parts[i].getPosition());
                //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                shootPoints[i] = Point(parts[i].getXPosition(),0);//mainpt.getYPosition());
            }
            //            cout <<show<<endl;
            waitKey(100);
        }
        else if(ctrl == 'a'){
            //Left
            //   waitKey(0);
            //            cv::Rect roi(mainpt.getPosition(),kanye.size());
            if(ctrl == ctrl2){
                //                cout <<"Moving"<<endl;
                mainpt.moveParticle(180.0, 15, 0);
            }
            if(ctrl2 == 'w'){
                //turns left by 90˚ or right by 270˚
                angle = 270.0;
                movement = 0.0;
            }
            else if(ctrl2 == 's'){
                //turns around by 180˚. Turns left 180˚ or right 180˚
                angle = 90.0;
                movement = 0.0;
            }
            else if(ctrl2 == 'd'){
                //turns right by 90˚ or left by 270˚
                angle = 0.0;
                movement = 0.0;
            }
            else{
                angle = 180.0;
                movement = 15.0;
            }
            cv::Rect roi(mainpt.getPosition(),kanye.size());
            Mat r = cv::getRotationMatrix2D(Point(25.0,25.0), 180, 1.0);
            Mat rotatedKanye;
            warpAffine(kanye, rotatedKanye, r, kanye.size());
            image2 = image.clone();
            rotatedKanye.copyTo(image2(roi));
            //            cout <<"showing"<<endl;
            for(int i = 0;i<parts.size();i++){
                parts[i].moveParticle(angle, movement, variance);
            }
            drawParts(parts, image2, mainpt.getPosition());
            imshow("result", image2);
            show = getOrientation(3, mainpt.getPosition());
            for(int i = 0; i<numberOfParticles;i++){
                show1 = getOrientation(3, parts[i].getPosition());
                //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                shootPoints[i] = Point(0,parts[i].getYPosition());//mainpt.getXPosition(),show);
            }
            //            cout <<show<<endl;
            waitKey(100);
        }
        else if(ctrl == 's'){
            //Down
            //   waitKey(0);
            //            cv::Rect roi(mainpt.getPosition(),kanye.size());
            if(ctrl == ctrl2){
                //                cout <<"Moving"<<endl;
                mainpt.moveParticle(90.0, 15, 0);
            }
            if(ctrl2 == 'a'){
                //turns left by 90˚ or right by 270˚
                angle = 180.0;
                movement = 0.0;
            }
            else if(ctrl2 == 'w'){
                //turns around by 180˚. Turns left 180˚ or right 180˚
                angle = 270.0;
                movement = 0.0;
            }
            else if(ctrl2 == 'd'){
                //turns right by 90˚ or left by 270˚
                angle = 0.0;
                movement = 0.0;
            }
            else{
                angle = 90.0;
                movement = 15.0;
            }
            cv::Rect roi(mainpt.getPosition(),kanye.size());
            Mat r = cv::getRotationMatrix2D(Point(25.0,25.0), 270, 1.0);
            Mat rotatedKanye;
            warpAffine(kanye, rotatedKanye, r, kanye.size());
            image2 = image.clone();
            rotatedKanye.copyTo(image2(roi));
            //            cout <<"showing"<<endl;
            for(int i = 0;i<parts.size();i++){
                parts[i].moveParticle(angle, movement, variance);
            }
            drawParts(parts, image2, mainpt.getPosition());
            imshow("result", image2);
            show = getOrientation(2, mainpt.getPosition());
            for(int i = 0; i<numberOfParticles;i++){
                show1 = getOrientation(2, parts[i].getPosition());
                //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                shootPoints[i] = Point(parts[i].getXPosition(),250);//mainpt.getYPosition());
            }
            //            cout <<show<<endl;
            waitKey(100);
        }
        else if(ctrl == 'd'){
            //Right
            //   waitKey(0);
            //            cv::Rect roi(mainpt.getPosition(),kanye.size());
            if(ctrl == ctrl2){
                //                cout <<"Moving"<<endl;
                mainpt.moveParticle(0.0, 15, 0);
            }
            if(ctrl2 == 'a'){
                //turns left by 90˚ or right by 270˚
                angle = 180.0;
                movement = 0.0;
            }
            else if(ctrl2 == 's'){
                //turns around by 180˚. Turns left 180˚ or right 180˚
                angle = 90.0;
                movement = 0.0;
            }
            else if(ctrl2 == 'w'){
                //turns right by 90˚ or left by 270˚
                angle = 0.0;
                movement = 270.0;
            }
            else{
                angle = 0.0;
                movement = 15.0;
            }
            cv::Rect roi(mainpt.getPosition(),kanye.size());
            Mat r = cv::getRotationMatrix2D(Point(25.0,25.0), 0, 1.0);
            Mat rotatedKanye;
            warpAffine(kanye, rotatedKanye, r, kanye.size());
            image2 = image.clone();
            rotatedKanye.copyTo(image2(roi));
            //            cout <<"showing"<<endl;
            for(int i = 0;i<parts.size();i++){
                parts[i].moveParticle(angle, movement, variance);
            }
            drawParts(parts, image2, mainpt.getPosition());
            imshow("result", image2);
            show = getOrientation(1, mainpt.getPosition());
            for(int i = 0; i<numberOfParticles;i++){
                show1 = getOrientation(1, parts[i].getPosition());
                //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                shootPoints[i] = Point(250, parts[i].getYPosition());//mainpt.getXPosition(),show);
            }
            //            cout <<show<<endl;
            waitKey(100);
        }
        ctrl2=ctrl;
        parts = resampleParticles(parts);
        //        updateProbability(parts, shootPoints, show);
    }
    return 0;
    
}
*/





/*
const int HMC5883L_I2C_ADDR = 0x1E;
double angle;
int fd; 
unsigned char buf[16];
void selectDevice(int fd, int addr, char * name) {
	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		fprintf(stderr, "%s not present\n", name); 
	}
}

void writeToDevice(int fd, int reg, int val){
	char buf[2]; buf[0]=reg; buf[1]=val;
	if (write(fd, buf, 2) != 2) {
		fprintf(stderr, "Can't write to ADXL345\n");
	}
}



void setup(){
	wiringPiSetup();
}

int getCM(){
  	pinMode(TRIG, OUTPUT);
  	digitalWrite(TRIG, LOW);
  	delay(30);
  	digitalWrite(TRIG, HIGH);
  	delay(50);
  	digitalWrite(TRIG, LOW);
	pinMode(TRIG, INPUT);

        //Wait for echo start
        while(digitalRead(TRIG) == LOW);

        //Wait for echo end
        long startTime = micros();
        while(digitalRead(TRIG) == HIGH);
        long travelTime = micros() - startTime;

        //Get distance in cm
        int distance = travelTime / 58;

        return distance;
}

void WheelFlush(int nP, int nI, int nD)
{
	FILE *fp;
 	fp = fopen("/dev/servoblaster", "w");
 	if (fp == NULL) {
 		printf("Error opening file\n");
 		exit(0);
 	}
     	std::stringstream ss;
 	std::stringstream  s2;
	int Wheel1 = 200  +(nP+nI+nD);
	int Wheel2 = 200 -(nP+nI+nD);
        ss << Wheel1;
	s2 << Wheel2;
        std::string W1 = ss.str();
	std::string W2 = s2.str();
	std::cout<<"Wheel1: " << Wheel1 << " , Wheel2: "<< Wheel2 <<std::endl;
	fprintf(fp,("0=" + W1  + "\n").c_str()) ;
	fprintf(fp, ("1=" +  W2 + "\n").c_str());
	fflush(fp);
	delay(1000); 
 	return;
}

void  Compass(){ 
//	int fd; 
//	unsigned char buf[16];
//	system("/home/pi/enableMotor.sh");
//	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) { 
//		fprintf(stderr, "Failed to open i2c bus\n"); 
//	} 
//       char *foo = "HMC5883L";
//	selectDevice(fd, HMC5883L_I2C_ADDR, foo); 
//	writeToDevice(fd, 0x01, 32); 
//	writeToDevice(fd, 0x02, 0); 
	for (int i = 0; i < 1; ++i) { 
		buf[0] = 0x03;
		if ((write(fd, buf, 1)) != 1) { 
			fprintf(stderr, "Error writing to i2c slave\n"); 
                        std::cout << "fail compass"  << std::endl;
 //                       return 0;
		}
		if (read(fd, buf, 6) != 6) {
			fprintf(stderr, "Unable to read from HMC5883L\n");
                        std::cout << "fail compass 2" << std::endl;
//                        return 0;
		} 

		else { 
			short x = (buf[0] << 8) | buf[1]; 
			short y = (buf[4] << 8) | buf[5]; 
			short z = (buf[2] << 8) | buf[3]; 
		        angle = atan2(y, x) * 180 / M_PI; 
			printf("x=%d, y=%d, z=%d\n", x, y, z); 
			printf("angle = %0.1f\n\n", angle); 
///			return angle;
		} 
		usleep(600 * 1000); 
	}
}



void moveForwardAndBackward(char direction, int distance){
        FILE *fp;
        fp = fopen("/dev/servoblaster", "w");
	int d = 0;
	d = distance * 167;
        if(direction == 'w'){           //moving forwards
//                std::cout << "Going forward"  << std::endl;
                fprintf(fp,  "1=182\n");
                fprintf(fp, "0=100\n");
                fflush(fp);
                delay(d);
                fprintf(fp, "0=152\n");
                fprintf(fp, "1=152\n");
                fflush(fp);
                delay(500);
        }
        else if(direction ==  's'){     //moving backwards
//                std::cout << "Going forward"  << std::endl;
                fprintf(fp,  "0=182\n");
                fprintf(fp, "1=100\n");
                fflush(fp);
                delay(d);
                fprintf(fp, "0=152\n");
                fprintf(fp, "1=152\n");
                fflush(fp);
                delay(500);
        }
        else{
              std::cout<<"To move forward or backwards, enter w or s"<<std::endl;
        }
}


void turnRightAndLeft(char direction, int angleIn){
        FILE *fp;
        fp = fopen("/dev/servoblaster", "w");
	int ang = round(angleIn*11.8);
	if(direction == 'a'){			//turn left by angle
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");
		fflush(fp);
		delay(ang);
		fprintf(fp,  "1=152\n");
		fprintf(fp, "0=152\n");
		fflush(fp);
		delay(1);
		Compass();
	}
        else if(direction == 'd'){		//turn right by angle 
                fprintf(fp,  "1=182\n");
                fprintf(fp, "0=182\n");
                fflush(fp);
                delay(ang);
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n");
                fflush(fp);
                delay(1);
                Compass();
        }
//	std::cout << "Angle is " << angle << std::endl;
}

void goToAngle(int turn){
	double a = angle;
	int difference = abs(round(turn-a));
	while(difference>5){
		a = angle;
		if(difference <=7){
			break;
		}
		if(difference>30){
			turnRightAndLeft('a',15);
		}
		else if(difference > 15){
			turnRightAndLeft('a', 7);
		}
		else if(difference >7){
			turnRightAndLeft('a', 3);
		}
		std::cout<<"Difference: "<<difference<<std::endl;
		difference = abs(round(turn-a));
	}
}


int main(void)
{
	FILE *fp;
        fp = fopen("/dev/servoblaster", "w");
        if (fp == NULL) {
                printf("Error opening file\n");
                exit(0);
        }
	setup();
	int KP = 0;
	int error, prev_error = 0, sum=0;
	int P, I, D = 0;
//        system("/home/pi/enableMotor.sh");
        if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
                fprintf(stderr, "Failed to open i2c bus\n");
        }
        char *foo = "HMC5883L";
        selectDevice(fd, HMC5883L_I2C_ADDR, foo);
        writeToDevice(fd, 0x01, 32);
        writeToDevice(fd, 0x02, 0);
	while(1)
	{
	int   readingL,  readingR,  readingB, readingF = 0;
	int  max = 0;
	int dist = 0;
	char ctrl;
	double  toturn;
		Compass();
		std::cout<<"Enter w, a, s, d or g(enter angle)"<<std::endl;
//			std::cout<<"Enter the angle you would like to turn to"<<std::endl;
			std::cin>>ctrl;
			if(ctrl == 'w'){
				std::cout<<"How many inches forward?"<<std::endl;
				std::cin>>dist;
				moveForwardAndBackward('w', dist);
			}
			else if( ctrl == 's'){
                                std::cout<<"How many inches backward?"<<std::endl;
				std::cin>>dist;
				moveForwardAndBackward('s', dist);
			}
			else if( ctrl == 'a' || ctrl == 'd'){
				if(ctrl == 'a'){
					std::cout <<"Enter the angle to turn left"<<std::endl;
					std::cin >>toturn;
					if(abs(toturn - angle) > 3){//8){
						turnRightAndLeft('a', toturn);
					}
				}
				else if(ctrl == 'd'){
					std::cout <<"Enter the angle to turn right"<<std::endl;
					std::cin>>toturn;
					if(abs(toturn - angle)>3){//8){
						turnRightAndLeft('d', toturn);

					}
				}
			}
			else if(ctrl == 'g'){
				std::cout<<"Enter angle to go to"<<std::endl;
				int an =0;
				std::cin>>an;
				goToAngle(an);
			}
			else{
				std::cout<<"Invalid input"<<std::endl;
			}
			std::cout <<"_______________________________________"<<std::endl;
	}
}
*/
