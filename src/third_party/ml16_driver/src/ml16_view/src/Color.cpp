// 2018-11-07
// Set the color of ML16 Viewer

#include "ml16_view/TWColor.h"
#include<ros/ros.h>

using namespace std;

TWColor::TWColor(){};

TWColor::~TWColor(){};


uint32_t TWColor::ConstColor(){
	uint32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(125) << 8 | static_cast<uint32_t>(155));	
	return rgb;
}

uint32_t TWColor::IndoorColor(float distance){
 	if(distance<10){
	int g = (distance<8)?distance*10*3:255;
	int b = (distance>8)? 255-distance*3: 255;
	uint32_t rgb = (static_cast<uint32_t>(0) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	return rgb;		
	}
	else if (distance>=10 && distance<12){	
	int r = (distance*10-100)*12.75;
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(180));
	return rgb;
	}
	else if (distance>=12 && distance<20){
	int g =255-(distance*10-120)*3;
	uint32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(0));
	return rgb;
	}
	else {uint32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(88) << 8 | static_cast<uint32_t>(0));
	return rgb;}
}

uint32_t TWColor::OutdoorColor(float distance){
	if(distance<25){
	int g = (distance<20)?distance*10:200;
	int b = (distance>20)? 255-(distance-20)*30: 255;
	uint32_t rgb = (static_cast<uint32_t>(0) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	return rgb;
	}
	else if (distance>=25 && distance<30){	
	int r = (distance*10-250)*4;
	int b = (distance<28)?(100-distance*3):10;
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(220) << 8 | static_cast<uint32_t>(b));
	return rgb;
	}
	else if (distance>=30 && distance<50){
	int r = (distance>45)?200+(distance-30)*2.7 :255;
	int g =(distance<45)?220-(distance*10-300)*0.6:100;
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(0));
	return rgb;
	}
	else {uint32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(10) << 8 | static_cast<uint32_t>(0));return rgb;}
	
};
