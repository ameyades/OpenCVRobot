#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "wiringPi.h"
#include  <iostream> 
#include <cmath>
#include <sstream>
#include <cstring>
//////////////////
#include <fcntl.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/ioctl.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <linux/i2c-dev.h> 
#include <math.h> 

#define TRIG 5

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
    int dista = 0;
	d = distance * 167;
        if(direction == 'w'){           //moving forwards
  //              std::cout << "Going forward"  << std::endl;
            for(int i =0; i <distance; i++){
                int dista = 0;
                dista = getCM();
                if(dista <20){
                    break;
                }
                fprintf(fp,  "1=182\n");
                fprintf(fp, "0=100\n");
                fflush(fp);
                delay(167);
                fprintf(fp, "0=152\n");
                fprintf(fp, "1=152\n");
                fflush(fp);
                delay(500);
            }
/*                fprintf(fp,  "1=182\n");
                fprintf(fp, "0=100\n");
                fflush(fp);
                delay(d);
                fprintf(fp, "0=152\n");
                fprintf(fp, "1=152\n");
                fflush(fp);
                delay(500);*/
        }
        else if(direction ==  's'){     //moving backwards
                std::cout << "Going forward"  << std::endl;
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







int moveSonar(){            //function to check any nearby objects 180˚ around front of robot and
    FILE *fp;               //determine where this smallest distance lies with respect to front
    fp = fopen("/dev/servoblaster", "w");
    int dista = 0, smallestDistance=0;
    int left = 220;
    int j = 1;
    int direction = 0;
    int right = 60;
    smallestDistance = getCM();
    for(int i = left; i > right; i = i-19){
        std::cout<<"HERE and j is "<<j<<std::endl;
        fprintf(fp, "2=%d\n", i);
        fflush(fp);
        delay(500);
        dista = getCM();
        if(dista<smallestDistance){
            smallestDistance = dista;
            direction = j;
        }
        j++;
    }
    fprintf(fp, "2=145\n");
    fflush(fp);
    delay(500);
    std::cout <<"Smallest distance is "<<smallestDistance;
    if(direction == 5){
        std::cout<<", right in front"<<std::endl;
    }
    std::cout <<" and came from ";
    if(direction == 1){ //left
        std::cout <<" 90˚ to the left."<<std::endl;
    }
    else if(direction == 9){ //left
        std::cout <<" 90˚ to the right."<<std::endl;
    }
    else if(direction< 5){
        std::cout <<(90-(direction*23))<<"˚ to the left."<<std::endl;
    }
    else if(direction > 5){
        std::cout <<((direction-5)*23)<<"˚ to the right."<<std::endl;
    }
    return smallestDistance;
}


/*
void moveSonarLeftRight(char direction){        //function to move servomotor of sonar left and right
    FILE *fp;                                   //a for left, d for right
    fp = fopen("/dev/servoblaster", "w");
    int dista = 0;
    if(direction == 'a'){           //checking left
        fprintf(fp,  "2=220\n");
        fflush(fp);
        delay(800);
        dista = getCM();
        std::cout <<"Distance to the left is "<<dista<<std::endl;
        fprintf(fp, "2=145\n");
        fflush(fp);
        delay(500);
    }
    else if(direction ==  'd'){     //checking right
        fprintf(fp,  "2=70\n");
        fflush(fp);
        delay(800);
        dista = getCM();
        std::cout <<"Distance to the right is "<<dista<<std::endl;
        fprintf(fp, "2=145\n");
        fflush(fp);
        delay(500);
    }
}
*/



void turnRightAndLeft(char direction, int angleIn){
        FILE *fp;
        fp = fopen("/dev/servoblaster", "w");
	int ang = round(angleIn*8.8);
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
	/* //OLD PID stuff
	int KP = 0;
    int dista = 0;
	int error, prev_error = 0, sum=0;
	int P, I, D = 0;
	*/
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
	//	std::cout << "Enter the angle you would like to  turn to" << std::endl;
	//	std::cin>>ctrl;
	//	std::cin >> toturn;
	//	Compass();
			/* std::cout << "Enter the angle you would like to  turn to" << std::endl;
	       		  std::cin >> toturn;*/
		Compass();
		std::cout<<"Enter w, a, s, d or g(enter angle) or c to check sonar distance"<<std::endl;
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
/*						fprintf(fp,  "1=100\n");
						fprintf(fp, "0=100\n");
						fflush(fp);
						delay(150);
						fprintf(fp,  "1=152\n");
						fprintf(fp, "0=152\n");
						fflush(fp);
						delay(1);
						Compass();
						std::cout << "Angle is " << angle << std::endl;*/
					}
				}
				else if(ctrl == 'd'){
					std::cout <<"Enter the angle to turn right"<<std::endl;
					std::cin>>toturn;
					if(abs(toturn - angle)>3){//8){
						turnRightAndLeft('d', toturn);
/*	                                        fprintf(fp,  "1=182\n");
        	                                fprintf(fp, "0=182\n");
                	                        fflush(fp);
                        	                delay(150);
						fprintf(fp,  "0=152\n");
						fprintf(fp, "1=152\n");
						fflush(fp);
						delay(1);
						Compass();
						std::cout <<"Angle is "<<angle<<std::endl;*/
					}
				}
			}
			else if(ctrl == 'g'){
				std::cout<<"Enter angle to go to"<<std::endl;
				int an =0;
				std::cin>>an;
				goToAngle(an);
			}
            else if(ctrl == 'c'){

                
                dist = moveSonar();

                //Begin PID
                int error, Integral_Sum,  prev_error = 0, sum=0;
				int reading, IntThresh=5; 
				float IntThresj = 0.01;
				int P, I, D = 0;
				float KP = 0.1, KI= .005, KD = .01 ; //KP = 200, KI= 300, KD = 10 ;

				//An increase in KP, increases response time
               
                /*
                while (1) {
					//int reading = getCM();
                    dist = moveSonar();
			        printf("Distance: %dcm\n", dist);
					delay(100);
					
					//setvalue 10
					
					//Manual test trials
					//std::cout << "Enter reading: "<<std::endl;
					//std::cin >>reading;
					
					reading = dist;
					Error = reading-256; //Error = setpoint-actual
					
					if (std::abs(Error) < IntThresh) //In case of error too small, then stop integration
					{
						Integral_Sum = Integral_Sum + Error;
					}
					else
						Integral_Sum=0;
                    }
					P = Error * KP;
					I = Integral_Sum * KI;
					D =  (Error - prev_error) * KD;
					
					
					std::stringstream ss;
		       		std::stringstream  s2;
		
		       		int Wheel1 = 148 +(P+I+D);
		       		int Wheel2 = 155 + (P+I+D);
			     
				  	ss << Wheel1;
			       	s2 << Wheel2;
				    std::string W1 = ss.str();
				    std::string W2 = s2.str();
			     
			     	//Screen Output
		     		std::cout<<"PID: " << P+I+D <<std::endl;
			        std::cout<<"Wheel1: " << Wheel1 << " , Wheel2: "<< Wheel2 <<std::endl;
			        
			        //Wheel Flush
					fprintf(fp,("0=" + W1  + "\n").c_str()) ;
			      	fprintf(fp, ("1=" +  W2 + "\n").c_str());
			        fprintf(fp,"2=250\n");//Is this even needed?
			      	fflush(fp);
					fprintf(fp,"2=150\n");//Is this even needed?
			        fflush(fp);
					fprintf(fp,"2=0\n");//Is this even needed?
			        fflush(fp);
					//delay(1000);	
				
					prev_error = Error;
				        
		    }
        */
            }
			else{
				std::cout<<"Invalid input"<<std::endl;
			}
//            dista = getCM();
//            std::cout <<"Distance in front is "<<dista<<std::endl;
			std::cout <<"_______________________________________"<<std::endl;
	}
    
}


/*	std::cin >> ctrl;
	Compass();
	if(ctrl == 'w')
	{
		delay(50);	
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
		delay(500);
		std::cout<<"Turning back " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
		delay(500);
		std::cout<<"Turning right " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
	}
	else if(ctrl == 's')
	{
		delay(50);	
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
		delay(500);
		std::cout<<"Turning back " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
		delay(500);

	}
	else if(ctrl == 'a')
	{
		delay(50);	
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(975);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
	}
	else if(ctrl == 'd')
	{
		std::cout<<"Turning right " << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=182\n");	
        	fflush(fp);
        	delay(975);
        	fprintf(fp,  "1=152\n");
        	fprintf(fp, "0=152\n"); 
        	fflush(fp);
		delay(100);
	}
        Compass();
*/
//	std::cout <<"Angle is "<<angle<<"."<<std::endl;


/*
std::cout<<"Turning left " << std::endl;
//	std::cout<<"Turning left " << std::endl;

	readingF = getCM();	
		delay(50);	
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
        readingL = getCM();       
		delay(500);
		std::cout<<"Turning back " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
        readingB = getCM();       
		delay(500);
		std::cout<<"Turning right " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
        readingR = getCM();       
		delay(500);
		std::cout<<"Turning forward " << std::endl;
	///	delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
        fflush(fp);
        delay(750);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
		 fflush(fp);
               
		delay(500);
		if(readingF <300){
			 max = readingF;
		}
		std::cout <<  readingF <<  " forward " <<  readingL <<  " left " <<  readingR <<  " right " <<  readingB <<  " back" << std::endl; 
		if(readingL > max && readingL < 300)
		{
			
		  max = readingL;
		  // std::cout << " max " + max << std::endl;	
		}
		if(readingR > max && readingR < 300)
		{
			max = readingR;
		}
		if(readingB > max && readingB < 300)
		{
			max = readingB;
		}
		
		std::cout << " max " <<  max << std::endl;

		if(readingL == max)
		{
			fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
                delay(500);
		}
		else if(readingB == max)
		{
			fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
                delay(50);
                fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
                delay(500);
		}
		else if(readingR == max)
		{
			fprintf(fp,  "1=182\n");
		fprintf(fp, "0=182\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
               delay(50);
             /*   fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
                delay(50);
                  fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
                delay(500); */
	/*	}
		if(abs(readingL-readingR)<=13){
			if(139>=readingF && readingF>=126)
				while(1){
			                fprintf(fp,  "1=152\n");
			                fprintf(fp, "0=152\n"); 
				}
		}
		 std::cout<<"Going forward!" << std::endl;
                fprintf(fp, "1=182\n");
                fprintf(fp, "0=100\n");
                fflush(fp);
                delay(500);
}
*/

/*        while(1) {

	std::cout<<"Turning left " << std::endl;
//	std::cout<<"Turning left " << std::endl;
			
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
        int readingL = getCM();       
		delay(1000);
		std::cout<<"Turning back " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(750);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
        int readingB = getCM();       
		delay(1000);
		std::cout<<"Turning right " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(750);
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
        int readingR = getCM();       
		delay(1000);
		std::cout<<"Turning forward " << std::endl;
	///	delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
        fflush(fp);
        delay(750);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n"); 
		 fflush(fp);
        int readingF = getCM();       
		delay(1000);

	}


/*	std::cout<<"Turning left " << std::endl;
//	std::cout<<"Turning left " << std::endl;
			
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(100);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp);
        int readingL = getCM();       
		delay(1000);
		std::cout<<"Turning back " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(100);
		fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp)
        int readingB = getCM();       
		delay(1000);
		std::cout<<"Turning right " << std::endl;
		// delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                delay(100);
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n"); 
                fflush(fp)
        int readingR = getCM();       
		delay(1000);
		std::cout<<"Turning forward " << std::endl;
	///	delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                
        int readingF = getCM();       
		delay(1000);
		
		delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                
        int readingL = getCM();       
		delay(1000);
		std::cout<<"Turning back " << std::endl;
		delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                
        int readingB = getCM();       
		delay(1000);
		std::cout<<"Turning right " << std::endl;
		delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
                
        int readingR = getCM();       
		delay(1000);

		std::cout<<"Turning forward " << std::endl;
		delay(500);
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);                
        int readingF = getCM();       
		delay(1000);
		
	}


	while (1) {
		int reading = getCM();
        	printf("Distance: %dcm\n", getCM());
		delay(100);
		
		//setvalue 10
		error = reading - 256;
		
		P = KP  * (error);
		D =  (error - prev_error);
		
		if ( std::abs(error) <= 5)
		{
			sum = sum + error;
		}
		else
		{
			sum = 0;
		}
		I =  sum;
		
		std::stringstream ss;
       		std::stringstream  s2;

       		int Wheel1 = 148 +(P+I+D);
       		int Wheel2 = 155 + (P+I+D);
        	ss << Wheel1;
        	s2 << Wheel2;
	        std::string W1 = ss.str();
	        std::string W2 = s2.str();
                
	        std::cout<<"Wheel1: " << Wheel1 << " , Wheel2: "<< Wheel2 <<std::endl;
	       // fprintf(fp,("0=" + W1  + "\n").c_str()) ;
      		// fprintf(fp, ("1=" +  W2 + "\n").c_str());
                fprintf(fp,"2=250\n");
      		fflush(fp);
		fprintf(fp,"2=150\n");
                fflush(fp);
		fprintf(fp,"2=0\n");
                fflush(fp);
		//delay(1000);	
		prev_error = error;
	}*/
	









/* #include <stdio.h>
#include <stdlib.h>
#include "wiringPi.h"
#include  <iostream> 
int main(void)
{
 FILE *fp;
 fp = fopen("/dev/servoblaster", "w");
 if (fp == NULL) {
 printf("Error opening file\n");
 exit(0);
 }
 while (1) {
       std::cout<< "Hello!" << std::endl; 
	char input;
	std::cin >> input;

	if(input == 'D')
	{
		std::cout<<"Turning right" << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=182\n");
		fflush(fp);
		delay(1000);
	}
        else if(input == 'A')
	{
		std::cout<<"Turning left " << std::endl;
		
		fprintf(fp,  "1=100\n");
		fprintf(fp, "0=100\n");	
                fflush(fp);
		delay(1000);
	} 
	else if(input == 'S')
	{
		std::cout<<"Going backward!" << std::endl;
		fprintf(fp, "0=182\n");
		fprintf(fp, "1=100\n");
		fflush(fp);
		delay(1000);
	}
	else if(input == 'W')
	{
		std::cout<<"Going forward!" << std::endl;
		fprintf(fp, "1=182\n");
		fprintf(fp, "0=100\n");
		fflush(fp);
		delay(1000);
	}
	fprintf(fp, "0=152\n");
	fprintf(fp, "1=152\n");	
	fflush(fp);
      /*  fprintf(fp, "0=182\n"); //Servo#0, Counter Clockwise
       fprintf(fp, "1=100\n"); //Servo#1, Counter Clockwise
       delay(1000);
       fflush(fp);
       fprintf(fp, "0=152\n"); //Stop
       fprintf(fp, "1=152\n"); //Stop
       delay(1000);
       fflush(fp);
       fprintf(fp, "0=100\n"); //Clockwise
       fprintf(fp, "1=182\n"); //Clockwise
       delay(1000);
       fflush(fp);
       fprintf(fp, "0=152\n"); //Stop
       fprintf(fp, "1=152\n"); //Stop
       delay(1000);
       fflush(fp);  THERE USED TO BE A END COMMENT HERE
 }
 fclose(fp); 
 return 0;
}
*/
