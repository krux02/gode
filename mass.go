package ode

/*
#cgo linux LDFLAGS: -lode -lstdc++
#define dDOUBLE
#include "ode/mass.h"
*/
import "C"

type Mass C.dMass

func (m *Mass) Check() bool {
	if C.dMassCheck((*C.dMass)(m)) == 0 {
		return false
	} else {
		return true
	}
}

func (m *Mass) SetZero() {
	C.dMassSetZero((*C.dMass)(m))
}

func (m *Mass) SetParameters(themass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23 Real) {
	C.dMassSetParameters((*C.dMass)(m), C.dReal(themass), C.dReal(cgx), C.dReal(cgy), C.dReal(cgz), C.dReal(I11), C.dReal(I22), C.dReal(I33), C.dReal(I12), C.dReal(I13), C.dReal(I23))
}

func (m *Mass) SetSphere(density, radius Real) {
	C.dMassSetSphere((*C.dMass)(m), C.dReal(density), C.dReal(radius))
}

func (m *Mass) SetSphereTotal(total_mass, radius Real) {
	C.dMassSetSphereTotal((*C.dMass)(m), C.dReal(total_mass), C.dReal(radius))
}

func (m *Mass) SetCapsule(density Real, direction int, radius, length Real) {
	C.dMassSetCapsule((*C.dMass)(m), C.dReal(density), C.int(direction), C.dReal(radius), C.dReal(length))
}

func (m *Mass) SetCapsuleTotal(total_mass Real, direction int, radius, length Real) {
	C.dMassSetCapsuleTotal((*C.dMass)(m), C.dReal(total_mass), C.int(direction), C.dReal(radius), C.dReal(length))
}

func (m *Mass) SetCylinder(density Real, direction int, radius, length Real) {
	C.dMassSetCylinder((*C.dMass)(m), C.dReal(density), C.int(direction), C.dReal(radius), C.dReal(length))
}

func (m *Mass) SetCylinderTotal(total_mass Real, direction int, radius, length Real) {
	C.dMassSetCylinderTotal((*C.dMass)(m), C.dReal(total_mass), C.int(direction), C.dReal(radius), C.dReal(length))
}

func (m *Mass) SetBox(density, lx, ly, lz Real) {
	C.dMassSetBox((*C.dMass)(m), C.dReal(density), C.dReal(lx), C.dReal(ly), C.dReal(lz))
}

func (m *Mass) SetBoxTotal(total_mass, lx, ly, lz Real) {
	C.dMassSetBoxTotal((*C.dMass)(m), C.dReal(total_mass), C.dReal(lx), C.dReal(ly), C.dReal(lz))
}

func (m *Mass) SetTrimesh(density Real, g *Geom) {
	C.dMassSetTrimesh((*C.dMass)(m), C.dReal(density), (C.dGeomID)((*C.struct_dxGeom)(g)))
}

func (m *Mass) SetTrimeshTotal(total_mass Real, g *Geom) {
	C.dMassSetTrimeshTotal((*C.dMass)(m), C.dReal(total_mass), (C.dGeomID)((*C.struct_dxGeom)(g)))
}

func (m *Mass) Adjust(newmass Real) {
	C.dMassAdjust((*C.dMass)(m), C.dReal(newmass))
}

func (m *Mass) Translate(x, y, z Real) {
	C.dMassTranslate((*C.dMass)(m), C.dReal(x), C.dReal(y), C.dReal(z))
}

func (m *Mass) Rotate(R *Matrix3) {
	C.dMassRotate((*C.dMass)(m), (*C.dReal)(&R[0]))
}

func (m *Mass) Add(b *Mass) {
	C.dMassAdd((*C.dMass)(m), (*C.dMass)(b))
}
