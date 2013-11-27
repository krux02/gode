package ode

/*
#cgo linux LDFLAGS: -lode -lstdc++
#include "ode/timer.h"
*/
import "C"

type Stopwatch C.dStopwatch

func (this *Stopwatch) Reset() {
	C.dStopwatchReset((*C.dStopwatch)(this))
}

func (this *Stopwatch) Start() {
	C.dStopwatchStart((*C.dStopwatch)(this))
}

func (this *Stopwatch) Stop() {
	C.dStopwatchStop((*C.dStopwatch)(this))
}

func (this *Stopwatch) Time() float64 {
	return float64(C.dStopwatchTime((*C.dStopwatch)(this)))
}
