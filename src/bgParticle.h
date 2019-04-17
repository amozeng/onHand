#pragma once

#include "ofMain.h"
#include "stdio.h"

class bgParticle
{
public:
    bgParticle();
    ~bgParticle();
    void update();
    void draw();
    
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 acceleration;
    
    
    //------- size & range
    float cubeX = 2000;
    float cubeY = 1500;
    float cubeZ = 100;
    float randomSphereSize;
    
    //------- color opcaity
    float randomOpacity;
    float opacityMin, opacityMax;
    float opaDecrSpeed = 0.5;
    bool opacityDecrease = false;

};

