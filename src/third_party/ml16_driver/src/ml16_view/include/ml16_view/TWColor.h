/*
 * TWColor.h
 *
 *  Created on: 11-07-2018
 *  Author: Elodie Shan
*/


#ifndef TWCOLOR_H_
#define TWCOLOR_H_
#include <stdint.h>

using namespace std;

class TWColor
{
public:
  TWColor();
  
  virtual ~TWColor();

  uint32_t ConstColor();

  uint32_t IndoorColor(float distance);
  
  uint32_t OutdoorColor(float distance);

};


#endif
