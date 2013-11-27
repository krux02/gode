package ode

/*
#cgo linux LDFLAGS: -lode -lstdc++
#include "ode/contact.h"
*/
import "C"

type ContactType uint

const (
	Contact_Mu2       ContactType = C.dContactMu2
	Contact_FDir1     ContactType = C.dContactFDir1
	Contact_Bounce    ContactType = C.dContactBounce
	Contact_SoftERP   ContactType = C.dContactSoftERP
	Contact_SoftCFM   ContactType = C.dContactSoftCFM
	Contact_Motion1   ContactType = C.dContactMotion1
	Contact_Motion2   ContactType = C.dContactMotion2
	Contact_MotionN   ContactType = C.dContactMotionN
	Contact_Slip1     ContactType = C.dContactSlip1
	Contact_Slip2     ContactType = C.dContactSlip2
	Contact_Approx0   ContactType = C.dContactApprox0
	Contact_Approx1_1 ContactType = C.dContactApprox1_1
	Contact_Approx1_2 ContactType = C.dContactApprox1_2
	Contact_Approx1   ContactType = C.dContactApprox1
)

type SurfaceParameters C.dSurfaceParameters

type Contact C.dContact
