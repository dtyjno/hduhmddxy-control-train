#ifndef PSO_H
#define PSO_H
#include "PID.h"
#include "FuzzyPID.h"

double fitnessFunction(FuzzyPID &fuzzypidcontroller, PID &controller, const double &measurement, const double &setpoint, double dt);
#endif