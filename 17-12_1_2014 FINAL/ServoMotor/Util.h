//
//  Util.h
//  testOpenCV
//
//  Created by Ameya Deshpande on 10/20/14.
//  Copyright (c) 2014 Ameya Deshpande. All rights reserved.
//

#ifndef testOpenCV_Util_h
#define testOpenCV_Util_h

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <opencv/highgui.h>
#include <opencv/cv.h>
//#include <array>
#include "Particle.h"

using namespace std;
/*

double getProbability(double u, double s, double x){
    double coefficient = 1.0/sqrt(2.0 * M_PI * s*s);    //s = Standard Deviation, s*s = Variance
    return coefficient * exp(-(x-u)*(x-u)/(2.0 * s*s));
}

double distToEdge(cv::Point pt1, cv::Point pt2) {
    double x = pt1.x - pt2.x;
    double y = pt1.y - pt2.y;
    
    return sqrt(pow(x, 2) + pow(y, 2));
}

void updateProbability(std::vector<Particle> &particles, std::vector<cv::Point> &particlesShoot, double distance) {
    
    float total_probabilities = 0.0;
    float new_weight = 0.0;
    float old_probabilities = 0.0;
    float new_probability = 0.0;
    double map_distance = 0.0;
    double sonar_variance = 10.0;
    
    // update all the particle probabilities
    for (int i=0; i<particles.size(); i++){
        cv::Point pos = particles[i].getPosition();
        
        //use heading to calculate the map distance from particle to wall.
        //map_distance =  distToEdge(direction,cv::Point(pos));
        map_distance = distToEdge(particles[i].getPosition(), particlesShoot[i]);
        
        // printf("Sonar: %f, Map_Distance: %f, direction: %d, Point(%d,%d), Weight: %f\n", distance, map_distance, direction, pos.i, pos.j, particles[i].getWeight());
        // Compute new probability using measured distance , map distance and sonar variance
        new_probability =  getProbability(distance, sonar_variance, map_distance); //distance by sonar report, sonar variance, given loaction
        
        
        
        // update each probability and compute total probabilities
        old_probabilities = particles[i].getWeight(); //P(robot@location)
        new_weight = old_probabilities * new_probability; //P(Sensor Reading| Robot @ Location) * P(robot@location)
        particles[i].setWeight(new_weight);
//        cout <<"THIS SECOND WEIGHT: "<<new_weight<<endl;
        total_probabilities += new_weight; //Ex: 0.25 + 0.25 + 0.3 = 0.8, so N = 1/0.8, here total_probabilities = 1/N
        //cout << "total_probabilities = " << total_probabilities << endl;
    }
    
    
    // Normalize all probabilities
    for (int i=0; i<particles.size(); i++){
        //normalized probability = probability / total probabilities
        particles[i].setWeight(particles[i].getWeight()/total_probabilities); //0.25/0.8 + 0.25/0.8 + 0.3/0.8 = 1
    }
    
}


std::vector<Particle> resampleParticles(std::vector<Particle>& oldParticles) {
    
    std::vector<Particle> newParticles;
    
    //Calculate a Cumulative Distribution Function for our particle weights
    std::vector<double> CDF;
    CDF.resize(oldParticles.size());
    CDF[0] = ((Particle)oldParticles[0]).getWeight();
    
    for(int i=1; i<CDF.size(); i++)
        CDF[i] = CDF[i-1] + oldParticles[i].getWeight();
    //Loop through our particles as if spinning a roulette wheel.
    //The random u that we start with determines the initial offset
    //and each particle will have a probability of getting landed on and
    //saved proportional to its posterior probability. If a particle has a very large
    //posterior, then it may get landed on many times. If a particle has a very low
    //posterior, then it may not get landed on at all. By incrementing by
    // 1/(numParticles) we ensure that we don't change the number of particles in our
    // returned set.
    
    int i = 0;
    double u = randomDouble()* 1.0/double(oldParticles.size());
    //double u = 1.0/double(oldParticles.size());
    
    
    for(int j=0; j < oldParticles.size(); j++){
        while(u > CDF[i]) //if i particle is very small, we don't want to let it in newparticle, so i++
            i++;
        
        Particle p = oldParticles[i]; //leave possible particles
        p.setWeight(1.0/double(oldParticles.size()));
        newParticles.push_back(p);
        
        u += 1.0/double(oldParticles.size());
    }
    
    
    return newParticles;
}









void drawParts(std::vector<Particle> &particles, cv::Mat img, cv::Point k){
  //  std::vector<Particle> newParticles = resampleParticles(particles);
//    int kx = k.x;
//    int ky = k.y;
    double weight = particles[0].getWeight();
    int distance = 0, DD = 0, j=0;
    int smallX = 0, smallY = 0;
//    cout <<"Position of Kanye: ("<<k.x+25<<", "<<k.y+25<<")"<<endl;
    smallX = particles[0].getXPosition() + 25;
    smallY = particles[0].getYPosition() + 25;
    distance = distToEdge(cv::Point (k.x+25, k.y+25), cv::Point (smallX,smallY));
    DD = distance;
    cv::Point pos;
    for(int i = 0; i<particles.size(); i++){
//        particles[i].moveParticle(0, 25, 20);
//        cout <<"Weight of particle "<<i+1<<": "<<particles[i].getWeight()<<endl;
//        cv::Point pos = particles[i].getPosition();
        smallX = particles[i].getXPosition() + 25;
        smallY = particles[i].getYPosition() + 25;
        distance = distToEdge(cv::Point (k.x+25, k.y+25), cv::Point (smallX,smallY));
        pos.x = particles[i].getXPosition()+25;
        pos.y = particles[i].getYPosition()+25;
//        cout <<"Particle "<<i+1<<"'s position: ("<<pos.x<<","<<pos.y<<")"<<endl;
//        if(distance < DD){
//            j = i;
//        }
        if(weight<particles[i].getWeight()){
            j = i;
            weight = particles[i].getWeight();
        }
//        int w = pos.x;
//        int q = pos.y;
//       cout<< "x:" << w <<", y:"<<q<<endl;
        circle(img, pos, 4.0, cvScalar(0, 0, 255), -1, 8);
    }
    pos.x = particles[j].getXPosition()+25;
    pos.y = particles[j].getYPosition()+25;
//    cout <<"Heaviest Particle is number "<<j+1<<" with weight "<<weight<<endl;
    circle(img, pos, 4.0, cvScalar(255, 255, 0), -1, 8);
}



//returns 0,1,2,3 based on which way it's facing
//0 is north
//1 is east
//2 is south
//3 is west
int getOrientation(int facing, cv::Point loc){
    int distanceToWall = 0;
    int x,y;
    x = loc.x;
    y = loc.y;
    if(facing == 0 || facing == 2){     //north or south
        if(facing == 0){    //north
            distanceToWall = y;
        }
        else{               //south
            distanceToWall = 300 - y-50;
        }
    }
    else if(facing == 1 || facing == 3){     //east or west
        if(facing == 1){    //east
            distanceToWall = 300 - x-50;
        }
        else{               //west
            distanceToWall = x;
        }
    }
    return distanceToWall;
}

*/




#endif
