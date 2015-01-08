//
//  Util_Map.h
//  testOpenCV
//
//  Created by Ameya Deshpande on 10/20/14.
//  Copyright (c) 2014 Ameya Deshpande. All rights reserved.
//

#ifndef testOpenCV_Util_Map_h
#define testOpenCV_Util_Map_h

#include <iostream>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include "Util.h"
#include "Particle.h"
//#include <array>

using namespace std;
using namespace cv;

void parseFile(
               std::vector<Point> &edges,
               std::vector<Point> &virus,
               std::vector<Point> &labA,
               Point &labB
               ) {
    std::ifstream myfile;
    //myfile.open ("map.txt");
    myfile.open ("map_origin.txt");
    
    if (myfile.is_open()){
        while (!myfile.eof()){
            Point p1;
            Point p2;
            int type;// 0 is wall,1 is rocket, 2 is docking
            if (myfile >> p1.x >> p1.y >> p2.x >> p2.y >> type ) {
                p1.x = p1.x*25;//convert ft to cm
                p1.y = p1.y*25;//convert ft to cm
                p2.x = p2.x*25;//convert ft to cm
                p2.y = p2.y*25;//convert ft to cm
                if(type == 0) {
                    //wall
                    edges.push_back(p1);
                    edges.push_back(p2);
                    //cout << "edges pushed" << endl;
                }
                else if(type == 1){
                    virus.push_back(p1);
                    virus.push_back(p2);
                }
                else if(type ==2){
                    labA.push_back(p1);
                    labA.push_back(p2);
                    
                }else if(type == 3){
                    labB.x = p1.x;
                    labB.y = p1.y;
                    
                }
            }
        }
    }
    myfile.close();
}

void drawMap(Mat& img,
             std::vector<Point> &edges,
               std::vector<Point> &virus,
               std::vector<Point> &labA,
               Point &labB
             ) {
    parseFile(edges,virus,labA,labB);
    
    int thickness = 6; //6
    int lineType = 8; //8
    
    //Draw Wall
    for(int i = 0;i < edges.size()-1; i+=2){

        Point p1 = edges[i];
        Point p2 = edges[i+1];
        line (img, p1, p2, Scalar(255, 0, 0), thickness, lineType);
    }

    //Draw Virus Location
    for(int i = 0;i < virus.size()-1; i+=2) {
        Point p1 = virus[i];
        Point p2 = virus[i+1];
        
        Point topLeft1, topRight1, bottomLeft1, bottomRight1;
        topLeft1.x = int (p1.x - p2.x/2);
        topLeft1.y = int (p1.y - p2.y/2);
        topRight1.x = int (p1.x + p2.x/2);
        topRight1.y = int (p1.y - p2.y/2);
        bottomLeft1.x = int (p1.x - p2.x/2);
        bottomLeft1.y = int (p1.y + p2.y/2);
        bottomRight1.x = int (p1.x + p2.x/2);
        bottomRight1.y = int (p1.y + p2.y/2);
        
        line(img,topLeft1, bottomRight1, CV_RGB(0, 150, 150));
        line(img,topRight1, bottomLeft1, CV_RGB(0, 150, 150));
    }
    
    //Draw LabA Location
    for(int i = 0;i < labA.size()-1; i+=2){
        Point p1 = labA[i];
        Point p2 = labA[i+1];
        
        Point topLeft, topRight, bottomLeft, bottomRight;
        topLeft.x = int (p1.x - p2.x/2);
        topLeft.y = int (p1.y - p2.y/2);
        topRight.x = int (p1.x + p2.x/2);
        topRight.y = int (p1.y - p2.y/2);
        bottomLeft.x = int (p1.x - p2.x/2);
        bottomLeft.y = int (p1.y + p2.y/2);
        bottomRight.x = int (p1.x + p2.x/2);
        bottomRight.y = int (p1.y + p2.y/2);
        rectangle(img, topLeft, bottomRight, cvScalar(204, 0, 204, 0),1,8,0);
        
        line(img,topLeft, bottomRight, CV_RGB(204,0,204));
        line(img,topRight, bottomLeft, CV_RGB(204,0,204));
    }
    //Draw LabB Location
    circle(img, labB, 10, CV_RGB(255,0,0), 5, CV_AA, 0);

}

#endif
