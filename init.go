package ode

/*
#define dDOUBLE
#include "ode/odeinit.h"
*/
import "C"

type InitFlags C.enum_dInitODEFlags

const InitFlag_ManualThreadCleanup InitFlags = C.dInitFlagManualThreadCleanup

type AllocateDataFlags C.enum_dAllocateODEDataFlags

const (
	AllocateFlag_BasicData     AllocateDataFlags = C.dAllocateFlagBasicData
	AllocateFlag_CollisionData AllocateDataFlags = C.dAllocateFlagCollisionData
	AllocateFlag_MaskAll       AllocateDataFlags = C.dAllocateMaskAll
)

func Init() {
	C.dInitODE()
}

func Init2(initFlags InitFlags) bool {
	if C.dInitODE2(C.uint(initFlags)) == 0 {
		return false
	} else {
		return true
	}
}

func AllocateDataForThread(flags AllocateDataFlags) {
	C.dAllocateODEDataForThread(C.uint(flags))
}

func CleanupAllDataForThread() {
	C.dCleanupODEAllDataForThread()
}

func Close() {
	C.dCloseODE()
}
