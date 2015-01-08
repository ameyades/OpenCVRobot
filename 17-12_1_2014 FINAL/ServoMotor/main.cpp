#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>
#include <cstdlib>

#include "Util.h"
#include "Util_Map.h"
#include "Particle.h"

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
    
    Mat kanye = imread("/Users/juanluisvasquez/Documents/School/USC/2014-2015/Fall 2014/CSCI 445/Labs/Lab 7-Particle Filter Simulation/team7TestOpenCV/team7TestOpenCV/kanye.jpeg", 1);   // Read the file
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






