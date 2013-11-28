package ode

/*
#cgo linux LDFLAGS: -lode -lstdc++
#define dDOUBLE
#include "ode/misc.h"
*/
import "C"

func (v *Vector3) Random(absmax Real) {
	C.dMakeRandomVector((*C.dReal)(&v[0]), 3, C.dReal(absmax))
}

func (v *Vector4) Random(absmax Real) {
	C.dMakeRandomVector((*C.dReal)(&v[0]), 4, C.dReal(absmax))
}

func (m *Matrix3) Random(absmax Real) {
	C.dMakeRandomMatrix((*C.dReal)(&m[0]), 3, 3, C.dReal(absmax))
}

func (m *Matrix4) Random(absmax Real) {
	C.dMakeRandomMatrix((*C.dReal)(&m[0]), 4, 4, C.dReal(absmax))
}

func (m *Matrix6) Random(absmax Real) {
	C.dMakeRandomMatrix((*C.dReal)(&m[0]), 6, 6, C.dReal(absmax))
}

func (m *Matrix3) ClearUpperTriangle() {
	C.dClearUpperTriangle((*C.dReal)(&m[0]), 3)
}

func (m *Matrix4) ClearUpperTriangle(absmax Real) {
	C.dClearUpperTriangle((*C.dReal)(&m[0]), 4)
}

func (m *Matrix6) ClearUpperTriangle(absmax Real) {
	C.dClearUpperTriangle((*C.dReal)(&m[0]), 6)
}

func (A *Matrix3) MaxDifference(B *Matrix3) Real {
	return Real(C.dMaxDifference((*C.dReal)(&A[0]), (*C.dReal)(&B[0]), 3, 3))
}

func (A *Matrix4) MaxDifference(B *Matrix4) Real {
	return Real(C.dMaxDifference((*C.dReal)(&A[0]), (*C.dReal)(&B[0]), 4, 4))
}

func (A *Matrix6) MaxDifference(B *Matrix6) Real {
	return Real(C.dMaxDifference((*C.dReal)(&A[0]), (*C.dReal)(&B[0]), 6, 6))
}

func (A *Matrix3) MaxDifferenceLowerTriangle(B *Matrix3) Real {
	return Real(C.dMaxDifferenceLowerTriangle((*C.dReal)(&A[0]), (*C.dReal)(&B[0]), 3))
}

func (A *Matrix4) MaxDifferenceLowerTriangle(B *Matrix4) Real {
	return Real(C.dMaxDifferenceLowerTriangle((*C.dReal)(&A[0]), (*C.dReal)(&B[0]), 4))
}

func (A *Matrix6) MaxDifferenceLowerTriangle(B *Matrix6) Real {
	return Real(C.dMaxDifferenceLowerTriangle((*C.dReal)(&A[0]), (*C.dReal)(&B[0]), 6))
}
