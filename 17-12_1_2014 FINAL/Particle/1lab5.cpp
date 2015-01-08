#include <stdio.h>
#include <string>
#include "wiringPi.h"
#include "lab5.h"
#include  <iostream> 
#include <cmath>
#include <sstream>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <queue>
#include <cstdlib>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Particle.h"
#include "Util_Map.h"
#include "Util.h"



#include <fcntl.h>
#include <unistd.h>
//#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <math.h>



#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv/cv.h>

#define TRIG 5

//test


using namespace cv;
//using namespace std;

const int HMC5883L_I2C_ADDR = 0x1E;
double angle;
int fd; 
unsigned char buf[16];

const int n=11; // horizontal size of the map
const int m=13; // vertical size size of the map
static int lmap[n][m];
static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static int dir_map[n][m]; // map of directions

bool redDelivered = false;
bool blueDelivered = false; 

char orientation = 's'; //s for down, a left, d right, w up
bool justTurned = false;


const int dir = 4; // number of possible directions to go at any position

int dx[dir]={1, 0, -1, 0}; //direction 
int dy[dir]={0, 1, 0, -1}; // 0-right, 1-up, 2-left, 3-down

void setup() {
    wiringPiSetup();
}


class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority
    
    public:
        node(int xp, int yp, int d, int p)
        {xPos=xp; yPos=yp; level=d; priority=p;}
    
        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}
    
    void updatePriority(const int & xDest, const int & yDest)
    {
        priority=level+estimate(xDest, yDest)*10; //A*
    }
    
    // give better priority to going strait instead of diagonally
    void nextLevel(const int & i) // i: direction
    {
        level+=(dir==8?(i%2==0?10:14):10);
    }
    
    // Estimation function for the remaining distance to the goal; returnd the distance
    const int & estimate(const int & xDest, const int & yDest) const
    {
        static int xd, yd, d;
        xd=xDest-xPos;
        yd=yDest-yPos;
        
        // Euclidian Distance
        //d=static_cast<int>(sqrt(xd*xd+yd*yd));
        
         //Manhattan distance
         d=abs(xd)+abs(yd);
        
        // Chebyshev distance
        //d=max(abs(xd), abs(yd));
        
        return(d);
    }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
    return a.getPriority() > b.getPriority();
}


// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish )
{
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;
    
    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }
    
    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    
    
    pq[pqi].push(*n0);


    // A* search
    while(!pq[pqi].empty())
    {
//         std::cout << "not empty" <<std::endl;
        // get the current node w/ the highest priority
        // from the list of open nodes
        
//        cout << "pq[pqi].top().getLevel() = " << pq[pqi].top().getLevel() << endl;
//        cout << "pq[pqi].top().getPriority() = " << pq[pqi].top().getPriority() << endl;

        
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                    pq[pqi].top().getLevel(), pq[pqi].top().getPriority());
        
        x=n0->getxPos(); y=n0->getyPos();
        
        pq[pqi].pop(); // remove the node from the open list

        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;
        
//         std::cout << "x = " << x << "; y = " << y << std::endl;
//         std::cout << "closed_nodes_map[x][y] = " << closed_nodes_map[x][y] << endl << std::endl;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                //cout << "dir_map[x][y] = " << dir_map[x][y]  << endl;

                j=dir_map[x][y];
                //cout << "j = " << j << endl;

                c='0'+(j+dir/2)%dir;
//                 std::cout << "c = " << c << endl << std::endl;

                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }
            
            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }
        
        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            
            xdx=x+dx[i]; ydy=y+dy[i];
//             std::cout << "i = " << i << std::endl;
//             std::cout << "xdx = " << xdx << "; ydy = " << ydy << std::endl;
//             std::cout << "map[xdx][ydy] = " << lmap[xdx][ydy] << "; closed_nodes_map[xdx][ydy] = " << closed_nodes_map[xdx][ydy] << std::endl;

            
            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || lmap[xdx][ydy]==1
                 || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                            n0->getPriority());
//                 std::cout << "TRAVELED m0->getLevel() = " << m0->getLevel() << "; m0->getPriority() = " << m0->getPriority() << std::endl;
                
                
                
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);
//                 std::cout << "NEW m0->getPriority() = " << m0->getPriority() << std::endl;

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
//                     std::cout << "lala" << std::endl;
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
//                     std::cout << "pq[pqi].top().getxPos() = " << pq[pqi].top().getxPos()<< std::endl;
//                     std::cout << "pq[pqi].top().getyPos() = " << pq[pqi].top().getyPos()<< std::endl;

                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

//                     std::cout << "dir_map[xdx][ydy] = " << dir_map[xdx][ydy] << endl << std::endl;

                
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
//                     std::cout << "NEW NEW m0->getPriority() = " << m0->getPriority() << std::endl;
//                     std::cout << "open_nodes_map[xdx][ydy] = " << open_nodes_map[xdx][ydy] << std::endl;
//                     std::cout << "in if: " << "xdx = " << xdx << "; ydy = " << ydy << std::endl;
   
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                    
                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                            pq[pqi].top().getyPos()==ydy))
                    {
//                         std::cout << "pq shows up" << std::endl;
//                         std::cout << "pq[pqi].top() = " << pq[pqi].top().getPriority() << std::endl;

                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
//                     std::cout << "pq[pqi].top().getPriority() = " << pq[pqi].top().getPriority() << std::endl;;
//                     std::cout << std::endl;
                    pq[pqi].pop(); // remove the wanted node
//                     std::cout << "After POP: pq[pqi].top().getPriority() = " << pq[pqi].top().getPriority() << std::endl;;

                    // empty the larger size pq to the smaller one
   //                  std::cout << "pq[pqi].size() = " << pq[pqi].size() << std::endl;
//                     std::cout << "pq[1 - pqi].size() = " << pq[1 - pqi].size() << std::endl;

                    
                   // if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
//                     std::cout << "pqi = " << pqi << std::endl;
                    
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
//                         std::cout << "NEW pq[pqi].top() = " << pq[pqi].top().getPriority() << std::endl;

                        pq[pqi].pop();
                    }
//                     std::cout << "Before pqi = " << pqi << std::endl;

//                     std::cout << "pq[" << pqi << "].size() = " << pq[pqi].size() << std::endl;
//                     std::cout << "pq[" << 1 - pqi << "].size() = " << pq[1 - pqi].size() << std::endl;
                    pqi=1-pqi;
//                     std::cout << "After (1- pqi) pqi = " << pqi << std::endl;
//                     std::cout << "pq[" << pqi << "].size() = " << pq[pqi].size() << std::endl;
//                     std::cout << "pq[" << 1 - pqi << "].size() = " << pq[1 - pqi].size() << std::endl;
//                     std::cout << "m0->getPriority() = " << m0->getPriority() << endl << std::endl;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

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





int getCM() {
    pinMode(TRIG, OUTPUT);
    digitalWrite(TRIG, LOW);
    delay(30);
    digitalWrite(TRIG, HIGH);
    delay(50);
    digitalWrite(TRIG, LOW);
    pinMode(TRIG, INPUT);
        while(digitalRead(TRIG) == LOW);

        long startTime = micros();
        while(digitalRead(TRIG) == HIGH);
        long travelTime = micros() - startTime;

        int distance = travelTime/50;

        return distance;




}



void WheelFlush(String W1, String W2, int delaynum)
{
    FILE *fp;
    fp = fopen("/dev/servoblaster", "w");
    if (fp == NULL) {
        printf("Error opening file\n");
        exit(0);
    }
    
   // std::cout<<"Wheel1: " << Wheel1 << " , Wheel2: "<< Wheel2 <<std::endl;
    fprintf(fp,("0=" + W1  + "\n").c_str()) ;
    fprintf(fp, ("1=" +  W2 + "\n").c_str());
    fflush(fp);
    delay(delaynum); 
    return;
}

void  Compass(){ 
//  int fd; 
//  unsigned char buf[16];
//  system("/home/pi/enableMotor.sh");
//  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) { 
//      fprintf(stderr, "Failed to open i2c bus\n"); 
//  } 
//       char *foo = "HMC5883L";
//  selectDevice(fd, HMC5883L_I2C_ADDR, foo); 
//  writeToDevice(fd, 0x01, 32); 
//  writeToDevice(fd, 0x02, 0); 
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
///         return angle;
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
            }
        }
        else if(direction ==  's'){     //moving backwards
                fprintf(fp,  "0=182\n");
                fprintf(fp, "1=100\n");
                fflush(fp);
                delay(d);
        }
        else{
              std::cout<<"To move forward or backwards, enter w or s"<<std::endl;
        }
        fprintf(fp, "0=152\n");
        fprintf(fp, "1=152\n");
        fflush(fp);
        delay(300);
}

void turnRightAndLeft(char direction, int angleIn){
        FILE *fp;
        fp = fopen("/dev/servoblaster", "w");
    int ang = round(angleIn*6);
    if(direction == 'a'){           //turn left by angle
        fprintf(fp,  "1=100\n");
        fprintf(fp, "0=100\n");
        fflush(fp);
        delay(ang);
        fprintf(fp,  "1=152\n");
        fprintf(fp, "0=152\n");
        fflush(fp);
        delay(1);
//         Compass();
    }
        else if(direction == 'd'){      //turn right by angle 
                fprintf(fp,  "1=182\n");
                fprintf(fp, "0=182\n");
                fflush(fp);
                delay(ang);
                fprintf(fp,  "1=152\n");
                fprintf(fp, "0=152\n");
                fflush(fp);
                delay(1);
//                 Compass();
        }
//  std::cout << "Angle is " << angle << std::endl;
}

void goToAngle(int turn){
    double a = angle;
    int difference = abs(round(turn-a));
    while(difference>5){
        a = angle;
        if(difference <=7){
            std::cout << "Breaking" << std::endl;
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


void PID(int smallDist, char errorDirect, int angle){
    int x = 6;
    double xcopy = float(x);
    int y = round(xcopy*(cos(float(90-angle)*0.0174533)));//-2);
//    std::cout<<"y is "<<y<<std::endl;
//    std::cout<<"INSIDE THE PID"<<std::endl;
    if (errorDirect=='a')//error on the left....MOVE RIGHT
    {
//        std::cout<<"Adjusting for the right"<<std::endl;
        moveForwardAndBackward('s', x); //move back x amount of inches
        turnRightAndLeft('d', 90-angle); //turn right by an angle
        moveForwardAndBackward('w', y); //move forward y amount of inches
    }
    else if (errorDirect =='d')//error on the right....MOVE LEFT
    {
//        std::cout<<"Adjusting for the left"<<std::endl;
        moveForwardAndBackward('s', x); //move back x amount of inches
        turnRightAndLeft('a', 90-angle); //turn left by an angle
        moveForwardAndBackward('w', y); //forward back y amount of inches
    }
    else if (errorDirect =='w')//error in front....MOVE BACK or STOP
    {
        moveForwardAndBackward('s', y); //move back x amount of inches
    }
}


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




int moveSonar(){            //function to check any nearby objects 180˚ around front of robot and
    FILE *fp;               //determine where this smallest distance lies with respect to front
    fp = fopen("/dev/servoblaster", "w");
    int dista = 0, smallestDistance=0;
    int left = 220;
    int j = 1;
    int direction = 0;
    int right = 60;
    int degrees;
    char poles;
    smallestDistance = getCM();
    for(int i = left; i > right; i = i-19){
//        std::cout<<"HERE and j is "<<j<<std::endl;
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
        moveForwardAndBackward('s', 3);
        std::cout<<", right in front"<<std::endl;
        degrees = 1000; //'1000' signifying straingt ahead
        poles = 'n';
    }
//    std::cout <<" and came from ";
    else if(direction == 1){ //left
        std::cout <<" and came from 90˚ to the left."<<std::endl;
        degrees = 90;
        poles = 'a';
    }
    else if(direction == 9){ //right
        std::cout <<" and came from 90˚ to the right."<<std::endl;
        degrees = 90;
        poles = 'd';
    }
    else if(direction< 5){
        std::cout <<" and came from "<<(90-(direction*23))<<"˚ to the left."<<std::endl;
        degrees = 90-(direction*23);
        poles = 'a';
    }
    else if(direction > 5){
        std::cout <<" and came from "<<((direction-5)*23)<<"˚ to the right."<<std::endl;
        degrees = (direction-5)*23;
        poles = 'd';
    }
    if (smallestDistance <=25 && direction != 5){
//        std::cout <<"CAlling PID"<<std::endl;
        if(degrees == 90){
            moveForwardAndBackward('s', 10);
            if(poles == 'a'){
                turnRightAndLeft('a', 10);
            }
            else if(poles == 'd'){
                turnRightAndLeft('d', 10);
            }
            moveSonar();
        }
        else{
            PID(smallestDistance, poles, degrees);
        }
    }
    return smallestDistance;
}


char getColorFromCamera(){
    std::cout<<"In Camera"<<std::endl;

//    setup();

    char value;


    VideoCapture cap(0); //capture the video from webcam
    std::cout<<"Video captured"<<std::endl;
    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

cap.set(CV_CAP_PROP_FRAME_WIDTH, 160);
cap.set(CV_CAP_PROP_FRAME_HEIGHT, 120);

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  int iLowH = 95; //original 0
 int iHighH = 115; //original 10

  int iLowS = 205;   //original 0
 int iHighS = 235;  //original 10

  int iLowV = 95;   //original 240
 int iHighV = 165;   //original 255

  //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 255);

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
 
    int iter = 0;
    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             std::cout << "Cannot read a frame from video stream" << std::endl;
             break;
        }

   Mat imgHSV;
     cv::Rect roi;
    roi.x = 40;
    roi.y = 30;
    roi.width = 60;
    roi.height = 40;
    cv::Mat Image1 =  imgOriginal(roi);
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    cvtColor(Image1, Image1, COLOR_BGR2HSV);     
  Mat imgThresholded;
  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
  Moments oMoments = moments(imgThresholded);

   double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;

   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
 /* if (dArea > 10000)
  {*/
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;  
   int sumH = 0;
   int sumS = 0;
   int sumV = 0;
   int avgH = 0;
   int avgS = 0;
   int avgV = 0;     
   char junk;     
 
     cv::MatIterator_<cv::Vec3b> it = Image1.begin<cv::Vec3b>(),
                        it_end = Image1.end<cv::Vec3b>();

   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
        line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(230,1,.5), 2); //blue : 230, 1, .5 //red
        if(dArea > 300){
            value = 'b';
            std::cout <<"BLUE"<<std::endl;
            return value;
        }


    delay(200);
   }

    iLastX = posX;
    iLastY = posY;
    imshow("Thresholded Image", imgThresholded); //show the thresholded image

    imgOriginal = imgOriginal + imgLines;
    imshow("Original", imgOriginal); //show the original image
    if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
        std::cout << "esc key is pressed by user" << std::endl;
        break; 
    }

//   return 0;
    }

}

bool getColor(char color){
    bool final = false;
	std::cout<<"In Camera"<<std::endl;
//    setup();
    char value;
    VideoCapture cap(0); //capture the video from webcam
    std::cout<<"Video captured"<<std::endl;
    if ( !cap.isOpened() )  // if not success, exit program
    {
		cout << "Cannot open the web cam" << endl;
        return false;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 160);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 120);    
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    int iLowH; //original 0
 	int iHighH; //original 10

	int iLowS;   //original 0
	int iHighS;  //original 10

	int iLowV;   //original 135
	int iHighV;   //original 145

	if(color == 'b')
	{
		iLowH = 105; //original 0
		iHighH = 130; //original 10

		iLowS = 215;   //original 0
		iHighS = 225;  //original 10

		iLowV = 100;   //original 135
		iHighV = 155;
	}
	else if(color == 'r')
	{
		iLowH = 170; //original 0
		iHighH = 179; //original 10

		iLowS = 150;   //original 0
		iHighS = 255;  //original 10

		iLowV = 60;   //original 240
		iHighV = 255;
	}
	createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 255);

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
	Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );

	int iter = 0;
	while (true)
    {
		Mat imgOriginal;
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
			std::cout << "Cannot read a frame from video stream" << std::endl;
			break;
        }
		Mat imgHSV;
    	cv::Rect roi;
    	roi.x = 40;
   		roi.y = 30;
   		roi.width = 60;
    	roi.height = 40;
	    cv::Mat Image1 =  imgOriginal(roi);
    	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	    cvtColor(Image1, Image1, COLOR_BGR2HSV);    
        Mat imgThresholded;
    	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    	Moments oMoments = moments(imgThresholded);
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;  
		int sumH = 0;
		int sumS = 0;
		int sumV = 0;
		int avgH = 0;
		int avgS = 0;
		int avgV = 0;     
		char junk;
		cv::MatIterator_<cv::Vec3b> it = Image1.begin<cv::Vec3b>(),
                        it_end = Image1.end<cv::Vec3b>();
// 			if(dArea > 2000){
		if(color == 'b'){
			if(dArea > 1000){
				value = 'b';
				std::cout <<"BLUE"<<std::endl;
				final = true;			
				return final;
			}
		}
		else if (color == 'r'){
			if(dArea > 2000){
				value = 'r';
				std::cout <<"RED"<<std::endl;
				final = true;
				return final;
			}
	     }
	    if(iter > 150)
	    {
		    final = false;
       		return final;
	    }    
	    iLastX = posX;
	    iLastY = posY;
	    imshow("Thresholded Image", imgThresholded); //show the thresholded image

	    imgOriginal = imgOriginal + imgLines;
	    imshow("Original", imgOriginal); //show the original image
	    if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	    {
	        std::cout << "esc key is pressed by user" << std::endl;
	        break; 
	    }
	    iter++;
    }
// 	cvDestroyWindow(namedWindow);
// 	cvReleaseImage(imgOriginal);
// 	cvReleaseImage(Image1);
// 	cvReleaseImage(imgHSV);
// 	cvReleaseImage(imgThresholded);
    std::cout << "STATUS OF FINAL: " << final << std::endl;
    return final;


}



void adjusting(char sideToTrace, int distance){
	//std::cout<<"-----------------------"<<std::endl;
	FILE *fp;
	fp = fopen("/dev/servoblaster", "w");
	int y = getCM();
	int distanceInFront = y;
	bool tooClose = false;
	delay(10);
	if(y < 19 && justTurned){
		tooClose = true;
		if(sideToTrace == 'd'){//just turned left, so turn right, go forward for 4, and turn left again
			turnRightAndLeft('d', 90);
			moveForwardAndBackward('w', 4);
			turnRightAndLeft('a', 90);
		}
		else if(sideToTrace =='a'){//just turned right, so turn left, go forward for 4, and turn right again
			turnRightAndLeft('a', 90);
			moveForwardAndBackward('w', 4);
			turnRightAndLeft('d', 90);
		}
	}
	if(!tooClose){
		while(y<22){
			moveForwardAndBackward('s', 1);
			y = getCM();
			delay(10);
		}
	}
	if(y < 30){
		if(sideToTrace == 'a'){
			turnRightAndLeft('a', 30);
		}
		else if(sideToTrace == 'd'){
			turnRightAndLeft('d', 30);
		}
	}
	delay(10);
    int dista = 0;
    char tracingSide;
    int count = 0;
	if(sideToTrace == 'd'){ //Tracing right
		//tracing right side
		fprintf(fp,  "2=70\n");
		fflush(fp);
		delay(1000);
		dista = getCM();
		delay(10);
		tracingSide = 'd';
		if(dista > 50){
			std::cout << "DISTA OVER 35, SWITCHING TO A	" << std::endl;
			//trace right side
			fprintf(fp,  "2=220\n");
			fflush(fp);
			delay(1000);
			dista = getCM();
			delay(10);
			tracingSide = 'a';
		}
		//TRACE
		for(int i =0; i < (distance); i++){		
			if(dista < 27){
				//Move left wheel more
				if(tracingSide == 'a'){
// 					std::cout << "TRACING A, HEADING RIGHT" << std::endl;
					count ++;
					turnRightAndLeft('d', 2);
				}
				else if(tracingSide == 'd'){
// 					std::cout << "TRACING D, HEADING LEFT" << std::endl;
					count ++;
					turnRightAndLeft('a', 2);
				}
			}
			else if(dista > 33&& dista < 50){
				
				//Move left wheel more
				if(tracingSide == 'a'){
// 				      std::cout << "MOVING CLOSER TO WALL A " << std::endl;
					count++;
					turnRightAndLeft('a', 1);
				}
				else if(tracingSide == 'd'){
// 				    std::cout << "MOVING CLOSER TO WALL D " << std::endl;	
					count ++;
					turnRightAndLeft('d', 1);
				}
			}
            
			fprintf(fp,  "1=182\n");
			fprintf(fp, "0=100\n");
			fflush(fp);
			delay(27);
			dista = getCM();			
			delay(10);
			distanceInFront = distanceInFront -5;
			if(distanceInFront <10){
				break;
			}
		}
		fprintf(fp, "0=152\n");
		fprintf(fp, "1=152\n");
		fflush(fp);
		delay(300);
	}
	else if(sideToTrace == 'a'){	//Tracing left side
		//tracing left side
		fprintf(fp,  "2=220\n");
		fflush(fp);
		delay(1000);
		dista = getCM();
		delay(10);
		tracingSide = 'a';
		if(dista > 50){
			std::cout << "DISTA OVER 35, SWITCHING TO D	" << std::endl;
			//tracing right side
			fprintf(fp,  "2=70\n");
			fflush(fp);
			delay(1000);
			dista = getCM();
			delay(10);
			tracingSide = 'd';
		}
		//TRACE
		for(int i =0; i <(distance); i++){
			if(dista < 27){
				//Move left wheel more
				if(tracingSide == 'a'){
// 					std::cout << "TRACING A, HEADING RIGHT" << std::endl;
					count++;
					turnRightAndLeft('d', 2);
				}
				else if(tracingSide == 'd'){
// 					std::cout << "TRACING D, HEADING LEFT" << std::endl;
					count ++;
					turnRightAndLeft('a', 2);
				}
			}
			else if(dista > 33&& dista < 50){
				//Move left wheel more
				if(tracingSide == 'a'){
// 				     std::cout << "MOVING CLOSER TO WALL A " << std::endl;
					count++;
					turnRightAndLeft('a', 1);
				}
				else if(tracingSide == 'd'){
// 				std::cout << "MOVING CLOSER TO WALL D " << std::endl;	
					count ++;
					turnRightAndLeft('d', 1);
				}
			}
            
			fprintf(fp,  "1=182\n");
			fprintf(fp, "0=100\n");
			fflush(fp);
			delay(27);
			dista = getCM();
			delay(10);
			distanceInFront = distanceInFront -5;
			if(distanceInFront <10){
				break;
			}
		}
		fprintf(fp, "0=152\n");
		fprintf(fp, "1=152\n");
		fflush(fp);
		delay(300);
	} 
	fprintf(fp, "2=145\n");
	fflush(fp);
	delay(500);
 	if(tracingSide == 'a'){ //what does this do?
 		int i = getCM();
 		delay(10);
 		while(i < 15){
 			std::cout << "In line 688 loop" << std::endl;
 			turnRightAndLeft('d', 1);
			moveForwardAndBackward('s', 1);
 			if(i > 15){
 				std::cout << "Breaking" << std::endl;
 				break;
 			}
 			i = getCM();
 			delay(10);
 		}
 	}
 	else if(tracingSide == 'd'){	
 		int i = getCM();
 		delay(10);
 		while(i < 15){
 			std::cout << "In line 702 loop" << std::endl;
 			turnRightAndLeft('a', 3);
            moveForwardAndBackward('s', 1);
 			if(i > 15){
 				std::cout << "Breaking" << std::endl;
 				break;
 			}
 			i = getCM();
 			delay(10);
 		}
 	}
}

int main( int argc, char** argv )
{
	char lastTurn = 'a';
    cout << "hello--beginning00" << endl;
    FILE *fp;
        fp = fopen("/dev/servoblaster", "w");
        if (fp == NULL) {
                printf("Error opening file\n");
                exit(0);
        }
    setup();
	bool redBool = false;
	bool blueBool = false;
    if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
                fprintf(stderr, "Failed to open i2c bus\n");
        }
        char *foo = "HMC5883L";
        selectDevice(fd, HMC5883L_I2C_ADDR, foo);
        writeToDevice(fd, 0x01, 32);
        writeToDevice(fd, 0x02, 0);
    char image_window[] = "Lab7";
    std::cout << "Sonar: " << getCM() << std::endl;
 

     namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"


    namedWindow("result");
    Mat original = Mat::zeros( 300, 300, CV_8UC3 );
    Mat image = Mat::zeros( 300, 300, CV_8UC3 );
        Mat image2;
    bool trueblue = false;
    bool truered = false;    
    //Mat kanye = imread("/Volumes/Home\ Directory/ServoMotor/kanye.jpeg", 1);   // Read the file
    int numberOfParticles = 25;
    double variance = 2.0;
    vector<Particle> parts(numberOfParticles);
    int x=0,z=0;
    for(int i = 0;i<numberOfParticles;i++){
                x = (rand()%280)+10;
                z = (rand()%80)+5;
                cout <<"x: "<<x<<", z: "<<z<<endl;
        parts[i].setWeight(0.04);
                parts[i].moveParticle(z, x, 0.8);
    }

    parts[9].setPosition(cv::Point(5, 1));
    if(! image.data )                              // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    std::vector<Point> edge;
    std::vector<Point> rock;
    std::vector<Point> dock;
    Point p;
    Particle mainpt(cv::Point(5,5));
    vector<Point> shootPoints(25);
    mainpt.setWeight(4.0);
    edge.push_back(Point(0,0));
    edge.push_back(Point(0,300));
    edge.push_back(Point(300,0));
    edge.push_back(Point(300,300));
    edge.push_back(Point(0,0));
    edge.push_back(Point(300,0));
    edge.push_back(Point(0,300));
    edge.push_back(Point(300,300));
    //cout<<"The edges work"<<endl;
    
    
    char ctrl;
    char ctrl2 = 'w';
    int show=0;
    int show1=0;
    //imshow("result", original);
    image = Scalar::all(0);
    image = original.clone();
    //drawMap(image, edge, rock, dock, p);
    waitKey(500);
    
     //cv::Rect roi(cv::Point(5,5),kanye.size());
   // cv::Rect roi(mainpt.getPosition(),kanye.size());
   image2 = Scalar::all(0);
    
    
    
    //Mat image2 = image(cv::Rect(5, 5, kanye.cols, kanye.rows));
   // kanye.copyTo(image2);
    
     /////////////////////
    drawParts(parts, image, mainpt.getPosition(), 0);
    Mat image3 = image.clone();
    drawGuy(image3, 15, 15, 0);
    //image2.copyTo(image3);
    imshow("result", image3);
    image3 = original.clone();
    drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), 90);
    drawParts(parts, image3, mainpt.getPosition(), angle);
    drawMap(image3, edge, rock, dock, p);
    imshow("result", image3);
    show = getOrientation(1, mainpt.getPosition());
    for(int i = 0; i<numberOfParticles;i++){
    show1 = getOrientation(1, parts[i].getPosition());
    //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
        shootPoints[i] = Point(250, parts[i].getYPosition());//mainpt.getXPosition(),show);
    }
            //            cout <<show<<endl;
    
    waitKey(1000);
   
    std::cout <<"enter key to begin"<<std::endl;
    char junk2;
    std::cin >> junk2;
    double angle = 0.0;
    double movement = 0.0; 
    int numberOfMoves = 2;
  //while(true){
        int  max = 0;
        int dist = 0;
        double  toturn;
        updateProbability(parts, shootPoints, show);
//         Compass();
        srand(time(NULL));
    
    int xA, yA, xB=1, yB=1;
    int plan =1, visit =0; 
    bool end=false; 
    char color = 't';
    char junk = 'j';
    int j2;  
//    char orientation = 's'; //s for down, a left, d right, w up

	bool detect=false;

    while (end==false)
    {
    
        updateProbability(parts, shootPoints, show);

        // create empty map, initializes values to 0 
        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++) lmap[x][y]=0;
        }
        
    
        /*      DRAW WALLS map[horizontal-n][vert-m]       */
    
        int jCount = m-1;
        int kCount = n-1;
        for (int xW = 0; xW<n; xW++) //Top-Bottom borders
        {
            lmap[xW][0] = 1;
            lmap[xW][jCount] = 1;
        }
        for (int yW = 0; yW<m; yW++) //Left-Right borders
        {
            lmap[0][yW] = 1;
            lmap[kCount][yW] = 1;
        }
        
        //Column 1
            lmap[1][6] = 1;
            lmap[1][8] = 1;
    
        //Column 2
            lmap[2][1] = 1;
            lmap[2][2] = 1;
            lmap[2][4] = 1;
            lmap[2][6] = 1;
            lmap[2][8] = 1;
            lmap[2][9] = 1;
            lmap[2][10] = 1;
    
        //Column 3
            lmap[3][4] = 1;
    
        //Column 4
            lmap[4][2] = 1;
            lmap[4][4] = 1;
            lmap[4][5] = 1;
            lmap[4][6] = 1;
            lmap[4][7] = 1;
            lmap[4][8] = 1;  
            lmap[4][10] = 1;
            lmap[4][11] = 1;
    
        //Column 5
            lmap[5][2] = 1;
            lmap[5][6] = 1;
    
        //Column 6
            lmap[6][1] = 1;
            lmap[6][2] = 1;
            lmap[6][4] = 1;
            lmap[6][6] = 1;
            lmap[6][7] = 1;
            lmap[6][8] = 1;
            lmap[6][10] = 1;
    
        //Column 7
            lmap[7][2] = 1;
            lmap[7][4] = 1;
            lmap[7][6] = 1;
            lmap[7][10] = 1;
    
        //Column 8
            lmap[8][2] = 1;
            lmap[8][4] = 1;
            lmap[8][6] = 1;
            lmap[8][8] = 1;
            lmap[8][9] = 1;
            lmap[8][10] = 1;
    
        //Column 9
            lmap[9][4] = 1;
            lmap[9][8] = 1;
    
        /*WALLS Finshed*/
        
        
        //Select start and finish locations
        
        xA = xB; //Start Location 
        yA = yB;
        
        if (color == 'r') //Go TO Lab A, for 'red' 
        {
        	detect = true;
            xB = 1; 
            yB = 9;
        }   
        else if (color == 'b') //go to Lab B, for 'Blue' 
        {
            detect =true;
            xB =9; yB = 9;
        }   
        else //search at next next virus location 
        {
        	visit++;
            if (detect == true)
        	{
        		xB=5; 
        		yB=7;
        	}       	
            else if (visit == 1)  //X1
            {
                xB =5; yB = 1;
            }
            else if (visit == 2) //X2
            {
                xB=7; yB=1;
            }   
            else if (visit==3) //X3
            {
                xB=5; yB=7;
            }
        
        }
        
        
            

     //   cout<<"Map Size (X,Y): "<<n<<","<<m<<endl;
     //   cout<<"Start: "<<xA<<","<<yA<<endl;
     //   cout<<"Finish: "<<xB<<","<<yB<<endl;
        // get the route
        clock_t start = clock();
        string route=pathFind(xA, yA, xB, yB);
        if(route=="") cout<<"An empty route generated!"<<endl;
        clock_t end = clock();
        double time_elapsed = double(end - start);
     //   cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
     //   cout<<"Route:"<<endl;
     //   cout<<route<<endl<<endl;
//        char orientation = 's'; //s for down, a left, d right, w up
        //redefining here. dx and dy was not correctly being accessed before
    //    int dx[4]={1, 0, -1, 0}; //direction 
      //  int dy[4]={0, 1, 0, -1}; // 0-right, 1-up, 2-left, 3-down
        
        // follow the route on the map and display it
        if(route.length()>0)
        {
            int j; char c;
            int x=xA;
            int y=yA;
            lmap[x][y]=2;
            for(int i=0;i<route.length();i++)
            {
                            
                c =route.at(i);
                j=atoi(&c);
                x=x+dx[j];
                y=y+dy[j];
                lmap[x][y]=3;
            }
            lmap[x][y]=4;
            
            // display the map with the route
            for(int y=0;y<m;y++)
            {
                for(int x=0;x<n;x++)
                    if(lmap[x][y]==0)
                        std::cout<<". ";
                    else if(lmap[x][y]==1)
                        std::cout<<"O "; //obstacle
                    else if(lmap[x][y]==2)
                        std::cout<<"S "; //start
                    else if(lmap[x][y]==3)
                        std::cout<<"R "; //route
                    else if(lmap[x][y]==4)
                        std::cout<<"F "; //finish
                std::cout<<endl;
            }
            for(int r=0;r<route.length();r++) //Loop to execute robot wheel path
            {
                //LATER--->enter wheel rotations 
                c =route.at(r);
                j=atoi(&c);
                if (j == 0){
                    std::cout <<"Orientation before is "<<orientation<<std::endl;
                    std::cout << "Moving RIGHT"<<std::endl;
                    if(orientation != 'd'){
                        if(orientation == 's'){
                            turnRightAndLeft('a', 90);
                            lastTurn = 'a';
                            justTurned = true;
							adjusting(lastTurn, 12);
//                            std::cout <<"turning left"<<std::endl;
                        }
                        else if(orientation == 'a'){
                            turnRightAndLeft('a', 190);
							lastTurn = 'a';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                        else if(orientation == 'w'){
//                            std::cout <<"turning right"<<std::endl;
                            turnRightAndLeft('d', 90);
                            lastTurn = 'd';
                            justTurned = true;
							adjusting(lastTurn, 10);
                        }
                    }
                    else{
						adjusting(lastTurn, 12);
                        justTurned = false;
                    }
//                    orientation = 'd';
                   for(int i = 0;i<parts.size();i++){
             		     parts[i].moveParticle(0, 25, variance);
                     }
                    mainpt.setPosition(cv::Point(mainpt.getXPosition() + 25, mainpt.getYPosition()));
                    updateProbability(parts, shootPoints, show); 
                    image3 = original.clone();
                    drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), 0);
                    drawParts(parts, image3, mainpt.getPosition(), 0);
                    drawMap(image3, edge, rock, dock, p);
                    imshow("result", image3);
                    show = getOrientation(1, mainpt.getPosition());
                    for(int i = 0; i<numberOfParticles;i++){
                    	show1 = getOrientation(1, parts[i].getPosition());
                    //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                   		shootPoints[i] = Point(250, parts[i].getYPosition());//mainpt.getXPosition(),show);
                    }
                    //            cout <<show<<endl;
                    waitKey(1000);
                    orientation = 'd';
               //     std::cout <<"Orientation after is "<<orientation<<std::endl;
                //    std::cin >> junk;

                }
                else if (j==1){
                   std::cout <<"Orientation before is "<<orientation<<std::endl;
                    std::cout << "Moving DOWN"<<std::endl;
                    if(orientation != 's'){
                        if(orientation == 'a'){
                            turnRightAndLeft('a', 90);
							lastTurn = 'a';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                        else if(orientation == 'w'){
                            turnRightAndLeft('a', 190);
                            lastTurn = 'a';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                        else if(orientation == 'd'){
                            turnRightAndLeft('d', 90);
                            lastTurn = 'd';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                    }
                    else{
						adjusting(lastTurn, 12);
						justTurned = false;
                     }
//                    orientation = 's';
                    for(int i = 0;i<parts.size();i++){
                        parts[i].moveParticle(90, 25, variance);
                    }
                    updateProbability(parts, shootPoints, show); 
                    mainpt.setPosition(cv::Point(mainpt.getXPosition(), mainpt.getYPosition() + 25));
                     image3 = original.clone();
                     drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), 90);
                     drawParts(parts, image3, mainpt.getPosition(), 90);
                     drawMap(image3, edge, rock, dock, p);
                     imshow("result", image3);
                     show = getOrientation(1, mainpt.getPosition());
                     for(int i = 0; i<numberOfParticles;i++){
                     show1 = getOrientation(1, parts[i].getPosition());
                     //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                     shootPoints[i] = Point(250, parts[i].getYPosition());//mainpt.getXPosition(),show);
                     }
                        //            cout <<show<<endl;
    
                    waitKey(1000);
                    orientation = 's';
                
                }
                else if (j==2){
                    std::cout <<"Orientation before is "<<orientation<<std::endl;
                    std::cout << "Moving LEFT"<<std::endl;
                    if(orientation != 'a'){
                        if(orientation == 'w'){
                            turnRightAndLeft('a', 90);
							lastTurn = 'a';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                        else if(orientation == 'd'){
                            turnRightAndLeft('a', 190);
                            lastTurn = 'd';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                        else if(orientation == 's'){
                            turnRightAndLeft('d', 90);
                            lastTurn = 'a';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                    }
                    else{
						adjusting(lastTurn, 12);
						justTurned = false;
                    }
//                    orientation = 'a';
                    for(int i = 0;i<parts.size();i++){
                        parts[i].moveParticle(180, 25, variance);
                    }
                    mainpt.setPosition(cv::Point(mainpt.getXPosition() - 25, mainpt.getYPosition()));
                    updateProbability(parts, shootPoints, show);
                     image3 = original.clone();
                     drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), 180);
                     drawParts(parts, image3, mainpt.getPosition(), 180);
                     drawMap(image3, edge, rock, dock, p);
                     imshow("result", image3);
                     show = getOrientation(1, mainpt.getPosition());
                     for(int i = 0; i<numberOfParticles;i++){
                     show1 = getOrientation(1, parts[i].getPosition());
                     //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                     shootPoints[i] = Point(250, parts[i].getYPosition());//mainpt.getXPosition(),show);
                     }
                        //            cout <<show<<endl;
    
                    waitKey(1000);
                    orientation = 'a';
                //    std::cout <<"Orientation after is "<<orientation<<std::endl;
                 //   std::cin >> junk;
                }
                else if (j==3){
                    std::cout <<"Orientation before is "<<orientation<<std::endl;
                   std::cout << "Moving UP"<<std::endl;
                    if(orientation != 'w'){
                        if(orientation == 'd'){
                            turnRightAndLeft('a', 90);
                            lastTurn = 'a';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                        else if(orientation == 's'){
                            turnRightAndLeft('a', 190);
                            lastTurn = 'a';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                        else if(orientation == 'a'){
                            turnRightAndLeft('d', 90);
                            lastTurn = 'd';
                            justTurned = true;
							adjusting(lastTurn, 12);
                        }
                    }
                    else{
						adjusting(lastTurn, 11);
						justTurned = false;
                    }
//                    orientation = 'w';
                    for(int i = 0;i<parts.size();i++){
                        parts[i].moveParticle(270, 25, variance);
                    }
                    mainpt.setPosition(cv::Point(mainpt.getXPosition(), mainpt.getYPosition() - 25));
                    updateProbability(parts, shootPoints, show);  
                     image3 = original.clone();
                     drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), 270);
                     drawParts(parts, image3, mainpt.getPosition(), 270);
                     drawMap(image3, edge, rock, dock, p);
                     imshow("result", image3);
                     show = getOrientation(1, mainpt.getPosition());
                     for(int i = 0; i<numberOfParticles;i++){
                     show1 = getOrientation(1, parts[i].getPosition());
                     //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                     shootPoints[i] = Point(250, parts[i].getYPosition());//mainpt.getXPosition(),show);
                     }
                        //            cout <<show<<endl;
    
                    waitKey(1000);
                    orientation = 'w';
                }
                updateProbability(parts, shootPoints, show);
                parts =  resampleParticles(parts);
            }
            j2 = j;            
            parts = resampleParticles(parts);
        }
        //adjust for color recognition
        int y = getCM();
        delay(10);
		while(y<26){
			moveForwardAndBackward('s', 1);
    	    y = getCM();
	        delay(10);
        }
		trueblue = getColor('b');
        if(!trueblue)
        {
            std::cout << "BLUE IS FALSE" << std::endl;
			truered = getColor('r');
//             truered = getRedFromCamera();
        }
        if(truered)
        {   
            std::cout << "RED IS TRUE" << std::endl;
			redBool = true;
            color = 'r';
        } 
        else if(trueblue)
        {
            std::cout << "BLUE IS TRUE" << std::endl;
			blueBool = true;
            color = 'b';
        }
        else
        {
            color = 't';
        }
        
        trueblue = false;
        truered = false;
        std::cout << "Color is " << color << std::endl;
        
   
//        std::cin >> junk;
         /*if (color == 'x')
         {
            return 0;
        }
		if(blueBool and redBool){
			break;
		}*/
		      //getchar(); // wait for a (Enter) keypress
        
        
        /*std::cout<<"Enter w, a, s, d or g(enter angle)"<< std::endl;
        std::cin >> ctrl;
        
        if(ctrl == 'w'){
            //Up
            //   waitKey(0);
            //            cv::Rect roi(mainpt.getPosition(),kanye.size());
            
            if(ctrl == ctrl2){
                //                cout <<"Moving"<<endl;
                
                std::cout<<"How many inches forward?"<< std::endl;
                cin>>dist;
                mainpt.moveParticle(270.0,  2 * dist, 0);
                moveForwardAndBackward('w', dist);

            }
            if(ctrl2 == 'a'){
                //turns left by 90˚ or right by 270˚
                angle = 180.0;
                movement = 0.0;
                turnRightAndLeft('d', 90);
            }
            else if(ctrl2 == 's'){
                //turns around by 180˚. Turns left 180˚ or right 180˚
                angle = 90.0;
                movement = 0.0;
                turnRightAndLeft('a', 90);
                turnRightAndLeft('a', 90);
            }
            else if(ctrl2 == 'd'){
                //turns right by 90˚ or left by 270˚
                angle = 0.0;
                movement = 0.0;
                turnRightAndLeft('a', 90);
            }
            else{
                //                cout <<"Moving"<<endl;
                angle = 270.0;
                movement = 15.0;
                //goToAngle(270);
                //                mainpt.moveParticle(270.0, 15, 0);
            }
           // cv::Rect roi(mainpt.getPosition(),kanye.size());
           // Mat r = cv::getRotationMatrix2D(Point(25.0,25.0), 90, 1.0);
           // Mat rotatedKanye;
           // warpAffine(kanye, rotatedKanye, r, kanye.size());
           // image2 = image.clone();
           // rotatedKanye.copyTo(image2);
            //            cout <<"showing"<<endl;
            for(int i = 0;i<parts.size();i++){
                parts[i].moveParticle(angle, movement, variance);
            }
            image3 = original.clone();
            drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), angle);
            drawParts(parts, image3, mainpt.getPosition(), angle);
            imshow("result", image3);
            show = getOrientation(0, mainpt.getPosition());
            for(int i = 0; i<numberOfParticles;i++){
                show1 = getOrientation(0, parts[i].getPosition());
                //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                shootPoints[i] = Point(parts[i].getXPosition(),0);//mainpt.getYPosition());
            }
            //            cout <<show<<endl;.
            waitKey(1000);
            ctrl2=ctrl;
            parts = resampleParticles(parts);
        }
        else if(ctrl == 'a'){
            //Left
            //   waitKey(0);
            //            cv::Rect roi(mainpt.getPosition(),kanye.size());
            if(ctrl == ctrl2){
                //                cout <<"Moving"<<endl;
                
                std::cout<<"How many inches forward?"<< std::endl;
                std::cin>>dist;
                mainpt.moveParticle(180.0,  2 * dist, 0);
                moveForwardAndBackward('w', dist);
            }
            if(ctrl2 == 'w'){
                //turns left by 90˚ or right by 270˚
                angle = 270.0;
                movement = 0.0;
                turnRightAndLeft('a', 90);
            }
            else if(ctrl2 == 's'){
                //turns around by 180˚. Turns left 180˚ or right 180˚
                angle = 90.0;
                movement = 0.0;
                turnRightAndLeft('d', 90);
            }
            else if(ctrl2 == 'd'){
                //turns right by 90˚ or left by 270˚
                angle = 0.0;
                movement = 0.0;
                turnRightAndLeft('a', 90);
                turnRightAndLeft('a', 90);
            }
            else{
                angle = 180.0;
                movement = 15.0;
            }
            //cv::Rect roi(mainpt.getPosition(),kanye.size());
            //Mat r = cv::getRotationMatrix2D(Point(25.0,25.0), 180, 1.0);
            //Mat rotatedKanye;
            //warpAffine(kanye, rotatedKanye, r, kanye.size());
            //image2 = image.clone();
            //rotatedKanye.copyTo(image2);
            //            cout <<"showing"<<endl;
            for(int i = 0;i<parts.size();i++){
                parts[i].moveParticle(angle, movement, variance);
            }
            image3 = original.clone();
            drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), angle);
            drawParts(parts, image3, mainpt.getPosition(), angle);
            imshow("result", image3);
            show = getOrientation(3, mainpt.getPosition());
            for(int i = 0; i<numberOfParticles;i++){
                show1 = getOrientation(3, parts[i].getPosition());
                //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                shootPoints[i] = Point(0,parts[i].getYPosition());//mainpt.getXPosition(),show);
            }
            //            cout <<show<<endl;
            waitKey(1000);
            ctrl2=ctrl;
            parts = resampleParticles(parts);
        }
        else if(ctrl == 's'){
            //Down
            //   waitKey(0);
            //            cv::Rect roi(mainpt.getPosition(),kanye.size());
            if(ctrl == ctrl2){
                //                cout <<"Moving"<<endl;
                 
                 std::cout<<"How many inches forward?"<< std::endl;
                 std::cin>>dist;
                 mainpt.moveParticle(90.0,  2 * dist, 0);
                 moveForwardAndBackward('w', dist);
            }
            if(ctrl2 == 'a'){
                //turns left by 90˚ or right by 270˚
                angle = 180.0;
                movement = 0.0;
                turnRightAndLeft('a', 90);
            }
            else if(ctrl2 == 'w'){
                //turns around by 180˚. Turns left 180˚ or right 180˚
                angle = 270.0;
                movement = 0.0;
                turnRightAndLeft('a', 90);
                turnRightAndLeft('a', 90);
            }
            else if(ctrl2 == 'd'){
                //turns right by 90˚ or left by 270˚
                angle = 0.0;
                movement = 0.0;
                turnRightAndLeft('d', 90);
            }
            else{
                angle = 90.0;
                movement = 15.0;
            }
           // cv::Rect roi(mainpt.getPosition(),kanye.size());
           // Mat r = cv::getRotationMatrix2D(Point(25.0,25.0), 270, 1.0);
           // Mat rotatedKanye;
           // warpAffine(kanye, rotatedKanye, r, kanye.size());
           // image2 = image.clone();
           // rotatedKanye.copyTo(image2);
            //            cout <<"showing"<<endl;
            for(int i = 0;i<parts.size();i++){
                parts[i].moveParticle(angle, movement, variance);
            }
            image3 = original.clone();
            drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), angle);
            drawParts(parts, image3, mainpt.getPosition(), angle);
            imshow("result", image3);
            show = getOrientation(2, mainpt.getPosition());
            for(int i = 0; i<numberOfParticles;i++){
                show1 = getOrientation(2, parts[i].getPosition());
                //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                shootPoints[i] = Point(parts[i].getXPosition(),250);//mainpt.getYPosition());
            }
            //            cout <<show<<endl;
            waitKey(1000);
            ctrl2=ctrl;
            parts = resampleParticles(parts);
        }
        else if(ctrl == 'd'){
            //Right
            //   waitKey(0);
            //            cv::Rect roi(mainpt.getPosition(),kanye.size());
            if(ctrl == ctrl2){
                //                cout <<"Moving"<<endl;
                
                 std::cout<<"How many inches forward?"<<std::endl;
                 std::cin>>dist;
                 mainpt.moveParticle(0, 2 * dist, 0);
                 moveForwardAndBackward('w', dist);
                
            }
            if(ctrl2 == 'a'){
                //turns left by 90˚ or right by 270˚
                angle = 180.0;
                movement = 0.0;
                turnRightAndLeft('a', 90);
                turnRightAndLeft('a', 90);
            }
            else if(ctrl2 == 's'){
                //turns around by 180˚. Turns left 180˚ or right 180˚
                angle = 90.0;
                movement = 0.0;
                turnRightAndLeft('a', 90);
            }
            else if(ctrl2 == 'w'){
                //turns right by 90˚ or left by 270˚
                angle = 0.0;
                movement = 0;
                turnRightAndLeft('d', 90);
            }
            else{
                angle = 0.0;
                movement = 15.0;
            }
           // cv::Rect roi(mainpt.getPosition(),kanye.size());
           // Mat r = cv::getRotationMatrix2D(Point(25.0,25.0), 0, 1.0);
           // Mat rotatedKanye;
           // warpAffine(kanye, rotatedKanye, r, kanye.size());
           // image2 = image.clone();
           // rotatedKanye.copyTo(image2);
            //            cout <<"showing"<<endl;
            for(int i = 0;i<parts.size();i++){
                parts[i].moveParticle(angle, movement, variance);
            }
            image3 = original.clone();
            drawGuy(image3, mainpt.getXPosition(), mainpt.getYPosition(), angle);
            drawParts(parts, image3, mainpt.getPosition(), angle);
            imshow("result", image3);
            show = getOrientation(1, mainpt.getPosition());
            for(int i = 0; i<numberOfParticles;i++){
                show1 = getOrientation(1, parts[i].getPosition());
                //                cout <<"distance to wall from particle "<<i<<": "<<show1<<endl;
                shootPoints[i] = Point(250, parts[i].getYPosition());//mainpt.getXPosition(),show);
            }
            //            cout <<show<<endl;
    
            waitKey(1000);
            ctrl2=ctrl;
            parts = resampleParticles(parts);
            
        }
        else
        {
            std::cout << "Invalid input" << std::endl;
        }
        */
        //        updateProbability(parts, shootPoints, show);
    }
// }
    return 0;
}
