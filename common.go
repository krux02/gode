package ode

/*
#define dDOUBLE
#include "ode/common.h"
*/
import "C"

import (
	"unsafe"
)

/*
type Real C.dReal

type Vector3 C.dVector3
type Vector4 C.dVector4
type Matrix3 C.dMatrix3
type Matrix4 C.dMatrix4
type Matrix6 C.dMatrix6
type Quaternion C.dQuaternion
*/

type World C.struct_dxWorld
type Space C.struct_dxSpace
type Body C.struct_dxBody
type Geom C.struct_dxGeom
type Joint C.struct_dxJoint
type JointGroup C.struct_dxJointGroup

// cgo -godefs=true typedefs.go
type Real float64
type Vector3 [4]float64
type Vector4 [4]float64
type Matrix3 [12]float64
type Matrix4 [16]float64
type Matrix6 [48]float64
type Quaternion [4]float64

func toBool(i C.int) bool {
	return i != 0
}

func toCint(b bool) C.int {
	if b {
		return 1
	} else {
		return 0
	}
}

func (this *World) CID() C.dWorldID {
	return (C.dWorldID)((*C.struct_dxWorld)(this))
}

func WorldPtr(id C.dWorldID) *World {
	return (*World)((*C.struct_dxWorld)(id))
}

func (this *Space) CID() C.dSpaceID {
	return (C.dSpaceID)((*C.struct_dxSpace)(this))
}

func SpacePtr(id C.dSpaceID) *Space {
	return (*Space)((*C.struct_dxSpace)(id))
}

func (this *Body) CID() C.dBodyID {
	return (C.dBodyID)((*C.struct_dxBody)(this))
}

func BodyPtr(id C.dBodyID) *Body {
	return (*Body)((*C.struct_dxBody)(id))
}

func (this *Geom) CID() C.dGeomID {
	return (C.dGeomID)((*C.struct_dxGeom)(this))
}

func GeomPtr(id C.dGeomID) *Geom {
	return (*Geom)((*C.struct_dxGeom)(id))
}

func (this *Joint) CID() C.dJointID {
	return (C.dJointID)((*C.struct_dxJoint)(this))
}

func JointPtr(id C.dJointID) *Joint {
	return (*Joint)((*C.struct_dxJoint)(id))
}

func (this *JointGroup) CID() C.dJointGroupID {
	return (C.dJointGroupID)((*C.struct_dxJointGroup)(this))
}

func JointGroupPtr(id C.dJointGroupID) *JointGroup {
	return (*JointGroup)((*C.struct_dxJointGroup)(id))
}

type Error int

const (
	Err_Unknown Error = C.d_ERR_UNKNOWN /* unknown error */
	Err_IAssert Error = C.d_ERR_IASSERT /* internal assertion failed */
	Err_UAssert Error = C.d_ERR_UASSERT /* user assertion failed */
	Err_LCP     Error = C.d_ERR_LCP     /* user assertion failed */
)

type JointType C.dJointType

const (
	JointType_None      JointType = C.dJointTypeNone /* or "unknown" */
	JointType_Ball      JointType = C.dJointTypeBall
	JointType_Hinge     JointType = C.dJointTypeHinge
	JointType_Slider    JointType = C.dJointTypeSlider
	JointType_Contact   JointType = C.dJointTypeContact
	JointType_Universal JointType = C.dJointTypeUniversal
	JointType_Hinge2    JointType = C.dJointTypeHinge2
	JointType_Fixed     JointType = C.dJointTypeFixed
	JointType_Null      JointType = C.dJointTypeNull
	JointType_AMotor    JointType = C.dJointTypeAMotor
	JointType_LMotor    JointType = C.dJointTypeLMotor
	JointType_Plane2D   JointType = C.dJointTypePlane2D
	JointType_PR        JointType = C.dJointTypePR
	JointType_PU        JointType = C.dJointTypePU
	JointType_Piston    JointType = C.dJointTypePiston
)

const (
	AMotorUser  = C.dAMotorUser
	AMotorEuler = C.dAMotorEuler
)

type JointFeedback struct {
	F1 Vector3 /* force applied to body 1 */
	T1 Vector3 /* torque applied to body 1 */
	F2 Vector3 /* force applied to body 2 */
	T2 Vector3 /* torque applied to body 2 */
}

/* private functions that must be implemented by the collision library:
 * (1) indicate that a geom has moved, (2) get the next geom in a body list.
 * these functions are called whenever the position of geoms connected to a
 * body have changed, e.g. with dBodySetPosition(), dBodySetRotation(), or
 * when the ODE step function updates the body state.
 */
func (this *Geom) GeomMoved() {
	C.dGeomMoved(C.dGeomID((*C.struct_dxWorld)(this)))
}

func (this *Geom) GetNextBody() (next *Geom) {
	return (*Geom)((*C.struct_dxWorld)(C.dGeomGetBodyNext(C.dGeomID((*C.struct_dxWorld)(this)))))
}

func GetConfiguration() string {
	return C.GoString(C.dGetConfiguration())
}

func CheckConfiguration(token string) bool {
	t := C.CString(token)
	defer C.free(unsafe.Pointer(t))
	if C.dCheckConfiguration(t) == 0 {
		return false
	} else {
		return true
	}
}
