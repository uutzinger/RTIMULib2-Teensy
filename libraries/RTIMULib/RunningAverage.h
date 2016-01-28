//
//    FILE: RunningAverage.h
//  AUTHOR: Rob dot Tillaart at gmail dot com
// VERSION: 0.2.08
//    DATE: 2015-apr-10
// PURPOSE: RunningAverage library for Arduino
//     URL: http://arduino.cc/playground/Main/RunningAverage
// HISTORY: See RunningAverage.cpp
//
// Released to the public domain
//
// backwards compatibility
// clr()   clear()
// add(x)  addValue(x)
// avg()   getAverage()

#ifndef RunningAverage_h
#define RunningAverage_h

#define RUNNINGAVERAGE_LIB_VERSION "0.2.08"

#include <string.h>
#include <math.h>
#include <stdint.h>

class RunningAverage
{
public:
    RunningAverage(void);
    RunningAverage(uint16_t);
    ~RunningAverage();

    void clear();
    void addValue(float);
    void fillValue(float, uint16_t);

    float getAverage();
    // returns lowest value added to the data-set since last clear
    float getMin() { return _min; };
    // returns highest value added to the data-set since last clear
    float getMax() { return _max; };

    float getElement(uint16_t idx);
    uint8_t getSize() { return _size; }
    uint8_t getCount() { return _cnt; }

protected:
    uint16_t _size;
    uint16_t _cnt;
    uint16_t _idx;
    float _sum;
    float * _ar;
    float _min;
    float _max;
};

#endif
// END OF FILE