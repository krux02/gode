package ode

/*
#include "ode/error.h"
*/
import "C"

import "fmt"

/* all user defined error functions have this type. error and debug functions
 * should not return.
 */
type MessageFunction func(errnum int, msg string, ap ...interface{})

// TODO implement callback backend

func SetErrorHandler(fn MessageFunction)   { fmt.Println("SetErrorHandler not wrapped yet") }
func SetDebugHandler(fn MessageFunction)   { fmt.Println("SetDebugHandler not wrapped yet") }
func SetMessageHandler(fn MessageFunction) { fmt.Println("SetMessageHandler not wrapped yet") }

func GetErrorHandler() MessageFunction   { fmt.Println("GetErrorHandler not wrapped yet"); return nil }
func GetDebugHandler() MessageFunction   { fmt.Println("GetDebugHandler not wrapped yet"); return nil }
func GetMessageHandler() MessageFunction { fmt.Println("GetMessageHandler not wrapped yet"); return nil }

// /* generate a fatal error, debug trap or a message. */
// ODE_API void dError (int num, const char *msg, ...);
// ODE_API void dDebug (int num, const char *msg, ...);
// ODE_API void dMessage (int num, const char *msg, ...);
