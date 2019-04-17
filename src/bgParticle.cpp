#include "bgParticle.h"

bgParticle::bgParticle()
{
    opacityMin = 150;
    opacityMax = 255;
    randomOpacity = ofRandom(opacityMin, opacityMax);
    
    
    position = glm::vec3();
    velocity = glm::vec3();
    acceleration = glm::vec3(0,0,0);

}

bgParticle::~bgParticle()
{
}


void bgParticle::update() {
    
    // color opacity changes
    if(opacityDecrease){
        if(opacityMin > 0) {
            opacityMin -= opaDecrSpeed;
        }
        if(opacityMax > 0) {
            opacityMax -= opaDecrSpeed;
        }
    }
    
    velocity += acceleration;
    position += velocity;
    acceleration = glm::vec3(0, 0, 0);
    
    // bounce when it reach the cube
    if(position.x > cubeX/2) {
        velocity.x *= -1;
        position.x = cubeX/2;
    }
    if(position.y > cubeY/2) {
        velocity.y *= -1;
        position.y = cubeY/2;
    }
    if(position.z > cubeZ/2) {
        velocity.z *= -1;
        position.z = cubeZ/2;
    }
    
    if(position.x < -cubeX/2) {
        velocity.x *= -1;
        position.x = -cubeX/2;
    }
    if(position.y < -cubeY/2) {
        velocity.y *= -1;
        position.y = -cubeY/2;
    }
    if(position.z < -cubeZ/2) {
        velocity.z *= -1;
        position.z = -cubeZ/2;
    }
    
    
}

void bgParticle::draw() {
    randomOpacity = ofRandom(opacityMin, opacityMax);
    ofSetColor(255, randomOpacity);
    ofFill();
    ofDrawSphere(position, randomSphereSize);
}




