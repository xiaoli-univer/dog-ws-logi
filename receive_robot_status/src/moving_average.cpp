#include "moving_average/moving_average.h"

MovingAverage::MovingAverage(unsigned short filterLength):
    filter_length(filterLength),
    filled(false),
    index(-1),
    sum(0), average(0) {
    data.resize(filter_length, 0);
}

MovingAverage::~MovingAverage() { }

void MovingAverage::in(double x) {
	index = (index + 1) % filter_length;
	sum -= data[index];
	data[index] = x;
	sum += x;
	if (!filled && index == filter_length - 1) {
		filled = true;
	}
	if (filled) {
		average = sum / filter_length;
	} else {
		average = sum / (index + 1);
	}
}

double MovingAverage::out() {
	return average;
}