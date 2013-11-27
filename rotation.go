package ode

/*
#cgo linux LDFLAGS: -lode -lstdc++
#include "ode/rotation.h"
*/
import "C"

func (R *Matrix3) SetIdentity() {
	C.dRSetIdentity((*C.dReal)(&R[0]))
}

func (R *Matrix3) FromAxisAndAngle(ax, ay, az, angle Real) {
	C.dRFromAxisAndAngle((*C.dReal)(&R[0]), C.dReal(ax), C.dReal(ay), C.dReal(az), C.dReal(angle))
}

func (R *Matrix3) FromEulerAngles(phi, theta, psi Real) {
	C.dRFromEulerAngles((*C.dReal)(&R[0]), C.dReal(phi), C.dReal(theta), C.dReal(psi))
}

func (R *Matrix3) From2Axes(ax, ay, az, bx, by, bz Real) {
	C.dRFrom2Axes((*C.dReal)(&R[0]), C.dReal(ax), C.dReal(ay), C.dReal(az), C.dReal(bx), C.dReal(by), C.dReal(bz))
}

func (R *Matrix3) FromZAxis(ax, ay, az Real) {
	C.dRFromZAxis((*C.dReal)(&R[0]), C.dReal(ax), C.dReal(ay), C.dReal(az))
}

func (q *Quaternion) SetIdentity() {
	C.dQSetIdentity((*C.dReal)(&q[0]))
}

func (q *Quaternion) FromAxisAndAngle(ax, ay, az, angle Real) {
	C.dQFromAxisAndAngle((*C.dReal)(&q[0]), C.dReal(ax), C.dReal(ay), C.dReal(az), C.dReal(angle))
}

func (qa *Quaternion) Multiply0(qb, qc *Quaternion) {
	C.dQMultiply0((*C.dReal)(&qa[0]), (*C.dReal)(&qb[0]), (*C.dReal)(&qc[0]))
}

func (qa *Quaternion) Multiply1(qb, qc *Quaternion) {
	C.dQMultiply1((*C.dReal)(&qa[0]), (*C.dReal)(&qb[0]), (*C.dReal)(&qc[0]))
}

func (qa *Quaternion) Multiply2(qb, qc *Quaternion) {
	C.dQMultiply2((*C.dReal)(&qa[0]), (*C.dReal)(&qb[0]), (*C.dReal)(&qc[0]))
}

func (qa *Quaternion) Multiply3(qb, qc *Quaternion) {
	C.dQMultiply3((*C.dReal)(&qa[0]), (*C.dReal)(&qb[0]), (*C.dReal)(&qc[0]))
}

func (R *Matrix3) FromQuat(Q *Quaternion) {
	C.dRfromQ((*C.dReal)(&R[0]), (*C.dReal)(&Q[0]))
}

func (Q *Quaternion) FromMat(M *Matrix3) {
	C.dQfromR((*C.dReal)(&Q[0]), (*C.dReal)(&M[0]))
}

func DQfromW(dq *[4]Real, w Vector3, q Quaternion) {
	C.dDQfromW((*C.dReal)(&dq[0]), (*C.dReal)(&w[0]), (*C.dReal)(&q[0]))
}
