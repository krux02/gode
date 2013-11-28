package ode

/*
#cgo linux LDFLAGS: -lode -lstdc++
#include "ode/objects.h"
*/
import "C"

/**
 * @defgroup world World
 *
 * The world object is a container for rigid bodies and joints. Objects in
 * different worlds can not interact, for example rigid bodies from two
 * different worlds can not collide.
 *
 * All the objects in a world exist at the same point in time, thus one
 * reason to use separate worlds is to simulate systems at different rates.
 * Most applications will only need one world.
 */


/**
 * @brief Create a new, empty world and return its ID number.
 * @return an identifier
 * @ingroup world
 */
func CreateWorld() *World {
	return (*World)((*C.struct_dxWorld)(C.dWorldCreate()))
}

/**
 * @brief Destroy a world and everything in it.
 *
 * This includes all bodies, and all joints that are not part of a joint
 * group. Joints that are part of a joint group will be deactivated, and
 * can be destroyed by calling, for example, dJointGroupEmpty().
 * @ingroup world
 * @param world the identifier for the world the be destroyed.
 */
func (w *World) Destroy() {
	C.dWorldDestroy(w.CID())
}

/**
 * @brief Set the world's global gravity vector.
 *
 * The units are m/s^2, so Earth's gravity vector would be (0,0,-9.81),
 * assuming that +z is up. The default is no gravity, i.e. (0,0,0).
 *
 * @ingroup world
 */
func (w *World) SetGravity(x, y, z Real) {
	C.dWorldSetGravity(w.CID(), C.dReal(x), C.dReal(y), C.dReal(z))
}

/**
 * @brief Get the gravity vector for a given world.
 * @ingroup world
 */
func (w *World) GetGravity() (gravity Vector3) {
	C.dWorldGetGravity(w.CID(), (*C.dReal)(&gravity[0]))
	return
}

/**
 * @brief Set the global ERP value, that controls how much error
 * correction is performed in each time step.
 * @ingroup world
 * @param dWorldID the identifier of the world.
 * @param erp Typical values are in the range 0.1--0.8. The default is 0.2.
 */
func (w *World) SetErp(erp Real) {
	C.dWorldSetERP(w.CID(), C.dReal(erp))
}

/**
 * @brief Get the error reduction parameter.
 * @ingroup world
 * @return ERP value
 */
func (w *World) GetErp() (erp Real) {
	return Real(C.dWorldGetERP(w.CID()))
}

/**
 * @brief Set the global CFM (constraint force mixing) value.
 * @ingroup world
 * @param cfm Typical values are in the range @m{10^{-9}} -- 1.
 * The default is 10^-5 if single precision is being used, or 10^-10
 * if double precision is being used.
 */
func (w *World) SetCfm(cfm Real)    { C.dWorldSetCFM(w.CID(), C.dReal(cfm)) }

/**
 * @brief Get the constraint force mixing value.
 * @ingroup world
 * @return CFM value
 */
func (w *World) GetCfm() (cfm Real) { return Real(C.dWorldGetCFM(w.CID())) }

/**
 * @brief Step the world.
 *
 * This uses a "big matrix" method that takes time on the order of m^3
 * and memory on the order of m^2, where m is the total number of constraint
 * rows. For large systems this will use a lot of memory and can be very slow,
 * but this is currently the most accurate method.
 * @ingroup world
 * @param stepsize The number of seconds that the simulation has to advance.
 */
func (w *World) Step(stepsize Real) { C.dWorldStep(w.CID(), C.dReal(stepsize)) }

/**
 * @brief Converts an impulse to a force.
 * @ingroup world
 * @remarks
 * If you want to apply a linear or angular impulse to a rigid body,
 * instead of a force or a torque, then you can use this function to convert
 * the desired impulse into a force/torque vector before calling the
 * BodyAdd... function.
 * The current algorithm simply scales the impulse by 1/stepsize,
 * where stepsize is the step size for the next step that will be taken.
 * This function is given a dWorldID because, in the future, the force
 * computation may depend on integrator parameters that are set as
 * properties of the world.
 */
func (w *World) ImpulseToForce(stepsize Real, ix, iy, iz Real) (force Vector3) {
	C.dWorldImpulseToForce(w.CID(), C.dReal(stepsize), C.dReal(ix), C.dReal(iy), C.dReal(iz), (*C.dReal)(&force[0]))
	return
}

/**
 * @brief Step the world.
 * @ingroup world
 * @remarks
 * This uses an iterative method that takes time on the order of m*N
 * and memory on the order of m, where m is the total number of constraint
 * rows N is the number of iterations.
 * For large systems this is a lot faster than dWorldStep(),
 * but it is less accurate.
 * @remarks
 * QuickStep is great for stacks of objects especially when the
 * auto-disable feature is used as well.
 * However, it has poor accuracy for near-singular systems.
 * Near-singular systems can occur when using high-friction contacts, motors,
 * or certain articulated structures. For example, a robot with multiple legs
 * sitting on the ground may be near-singular.
 * @remarks
 * There are ways to help overcome QuickStep's inaccuracy problems:
 * \li Increase CFM.
 * \li Reduce the number of contacts in your system (e.g. use the minimum
 *     number of contacts for the feet of a robot or creature).
 * \li Don't use excessive friction in the contacts.
 * \li Use contact slip if appropriate
 * \li Avoid kinematic loops (however, kinematic loops are inevitable in
 *     legged creatures).
 * \li Don't use excessive motor strength.
 * \liUse force-based motors instead of velocity-based motors.
 *
 * Increasing the number of QuickStep iterations may help a little bit, but
 * it is not going to help much if your system is really near singular.
 */
func (w *World) QuickStep(stepsize Real) { C.dWorldQuickStep(w.CID(), C.dReal(stepsize)) }

/**
 * @brief Set the number of iterations that the QuickStep method performs per
 *        step.
 * @ingroup world
 * @remarks
 * More iterations will give a more accurate solution, but will take
 * longer to compute.
 * @param num The default is 20 iterations.
 */
func (w *World) SetQuickStepNumIterations(num int) {
	C.dWorldSetQuickStepNumIterations(w.CID(), C.int(num))
}

/**
 * @brief Get the number of iterations that the QuickStep method performs per
 *        step.
 * @ingroup world
 * @return nr of iterations
 */
func (w *World) GetQuickStepNumIterations() (num int) {
	return int(C.dWorldGetQuickStepNumIterations(w.CID()))
}

/**
 * @brief Set the SOR over-relaxation parameter
 * @ingroup world
 * @param over_relaxation value to use by SOR
 */
func (w *World) SetQuickStepW(over_relaxation Real) {
	C.dWorldSetQuickStepW(w.CID(), C.dReal(over_relaxation))
}

/**
 * @brief Get the SOR over-relaxation parameter
 * @ingroup world
 * @returns the over-relaxation setting
 */
func (w *World) GetQuickStepW() Real {
	return Real(C.dWorldGetQuickStepW(w.CID()))
}

/* World contact parameter functions */

/**
 * @brief Set the maximum correcting velocity that contacts are allowed
 * to generate.
 * @ingroup world
 * @param vel The default value is infinity (i.e. no limit).
 * @remarks
 * Reducing this value can help prevent "popping" of deeply embedded objects.
 */
func (w *World) SetContactMaxCorrectingVel(vel Real) {
	C.dWorldSetContactMaxCorrectingVel(w.CID(), C.dReal(vel))
}

/**
 * @brief Get the maximum correcting velocity that contacts are allowed
 * to generated.
 * @ingroup world
 */
func (w *World) GetContactMaxCorrectingVel() Real {
	return Real(C.dWorldGetContactMaxCorrectingVel(w.CID()))
}

/**
 * @brief Set the depth of the surface layer around all geometry objects.
 * @ingroup world
 * @remarks
 * Contacts are allowed to sink into the surface layer up to the given
 * depth before coming to rest.
 * @param depth The default value is zero.
 * @remarks
 * Increasing this to some small value (e.g. 0.001) can help prevent
 * jittering problems due to contacts being repeatedly made and broken.
 */
func (w *World) SetContactSurfaceLayer(depth Real) {
	C.dWorldSetContactSurfaceLayer(w.CID(), C.dReal(depth))
}

/**
 * @brief Get the depth of the surface layer around all geometry objects.
 * @ingroup world
 * @returns the depth
 */
func (w *World) GetContactSurfaceLayer() Real {
	return Real(C.dWorldGetContactSurfaceLayer(w.CID()))
}

/* StepFast1 functions */

/**
 * @brief Step the world using the StepFast1 algorithm.
 * @param stepsize the nr of seconds to advance the simulation.
 * @param maxiterations The number of iterations to perform.
 * @ingroup world
 */
func (w *World) StepFast1(stepsize Real, maxiterations int) {
	C.dWorldStepFast1(w.CID(), C.dReal(stepsize), C.int(maxiterations))
}


/**
 * @defgroup disable Automatic Enabling and Disabling
 * @ingroup world bodies
 *
 * Every body can be enabled or disabled. Enabled bodies participate in the
 * simulation, while disabled bodies are turned off and do not get updated
 * during a simulation step. New bodies are always created in the enabled state.
 *
 * A disabled body that is connected through a joint to an enabled body will be
 * automatically re-enabled at the next simulation step.
 *
 * Disabled bodies do not consume CPU time, therefore to speed up the simulation
 * bodies should be disabled when they come to rest. This can be done automatically
 * with the auto-disable feature.
 *
 * If a body has its auto-disable flag turned on, it will automatically disable
 * itself when
 *   @li It has been idle for a given number of simulation steps.
 *   @li It has also been idle for a given amount of simulation time.
 *
 * A body is considered to be idle when the magnitudes of both its
 * linear average velocity and angular average velocity are below given thresholds.
 * The sample size for the average defaults to one and can be disabled by setting
 * to zero with 
 *
 * Thus, every body has six auto-disable parameters: an enabled flag, a idle step
 * count, an idle time, linear/angular average velocity thresholds, and the
 * average samples count.
 *
 * Newly created bodies get these parameters from world.
 */

/**
 * @brief Set the AutoEnableDepth parameter used by the StepFast1 algorithm.
 * @ingroup disable
 */
func (w *World) SetAutoEnableDepthSF1(autoEnableDepth bool) {
	C.dWorldSetAutoEnableDepthSF1(w.CID(), toCint(autoEnableDepth))
}

/**
 * @brief Get the AutoEnableDepth parameter used by the StepFast1 algorithm.
 * @ingroup disable
 */
func (w *World) GetAutoEnableDepthSF1() bool {
	return toBool(C.dWorldGetAutoEnableDepthSF1(w.CID()))
}

/**
 * @brief Get auto disable linear threshold for newly created bodies.
 * @ingroup disable
 * @return the threshold
 */
func (w *World) GetAutoDisableLinearThreshold() Real {
	return Real(C.dWorldGetAutoDisableLinearThreshold(w.CID()))
}

/**
 * @brief Set auto disable linear threshold for newly created bodies.
 * @param linear_threshold default is 0.01
 * @ingroup disable
 */

func (w *World) SetAutoDisableLinearThreshold(linear_theshold Real) {
	C.dWorldSetAutoDisableLinearThreshold(  w.CID(), C.dReal(linear_theshold) )
}

/**
 * @brief Get auto disable angular threshold for newly created bodies.
 * @ingroup disable
 * @return the threshold
 */

func (w *World) GetAutoDisableAngularThreshold() Real {
	return Real(C.dWorldGetAutoDisableAngularThreshold(w.CID()))
}

/**
 * @brief Set auto disable angular threshold for newly created bodies.
 * @param linear_threshold default is 0.01
 * @ingroup disable
 */

func (w *World) SetAutoDisableAngularThreshold(angular_threshold Real) {
	C.dWorldSetAutoDisableAngularThreshold( w.CID(), C.dReal(angular_threshold))
}

/**
 * @brief Get auto disable linear average threshold for newly created bodies.
 * @ingroup disable
 * @return the threshold
 */

func (w *World) GetAutoDisableLinearAverageThreshold() Real {
	return Real( C.dWorldGetAutoDisableLinearAverageThreshold(w.CID()) )
}

/**
 * @brief Set auto disable linear average threshold for newly created bodies.
 * @param linear_average_threshold default is 0.01
 * @ingroup disable
 */

func (w *World) SetAutoDisableLinearAverageThreshold(linear_average_threshold Real) {
	C.dWorldSetAutoDisableLinearAverageThreshold(w.CID, C.dReal(linear_average_threshold))
}

/**
 * @brief Get auto disable angular average threshold for newly created bodies.
 * @ingroup disable
 * @return the threshold
 */

func (w *World) GetAutoDisableAngularAverageThreshold() Real {
	return Real(C.dWorldGetAutoDisableAngularAverageThreshold(w.CID()))
}

/**
 * @brief Set auto disable angular average threshold for newly created bodies.
 * @param linear_average_threshold default is 0.01
 * @ingroup disable
 */
func (w *World) SetAutoDisableAngularAverageThreshold(angular_average_threshold Real) {
	C.dWorldSetAutoDisableAngularAverageThreshold(w.CID(), C.dReal(angular_average_threshold))
}

/**
 * @brief Get auto disable sample count for newly created bodies.
 * @ingroup disable
 * @return number of samples used
 */
func (w *World) GetAutoDisableAverageSamplesCount() int {
	return int(C.dWorldGetAutoDisableAverageSamplesCount(w.CID()))
}

/**
 * @brief Set auto disable average sample count for newly created bodies.
 * @ingroup disable
 * @param average_samples_count Default is 1, meaning only instantaneous velocity is used.
 * Set to zero to disable sampling and thus prevent any body from auto-disabling.
 */
func (w *World) SetAutoDisableAverageSamplesCount(average_samples_count int) {
	C.dWorldSetAutoDisableAverageSamplesCount(w.CID(), C.uint(average_samples_count))
}

/**
 * @brief Get auto disable steps for newly created bodies.
 * @ingroup disable
 * @return nr of steps
 */
func (w *World) GetAutoDisableSteps() bool {
	return toBool(C.dWorldGetAutoDisableSteps(w.CID()))
}

/**
 * @brief Set auto disable steps for newly created bodies.
 * @ingroup disable
 * @param steps default is 10
 */
func (w *World) SetAutoDisableSteps(steps int) {
	dWorldSetAutoDisableSteps(w.CID(), C.int(steps))
}

/**
 * @brief Get auto disable time for newly created bodies.
 * @ingroup disable
 * @return nr of seconds
 */
func (w *World) GetAutoDisableTime() Real {
	return Real(C.dWorldGetAutoDisableTime(w.CID()))
}

/**
 * @brief Set auto disable time for newly created bodies.
 * @ingroup disable
 * @param time default is 0 seconds
 */
func (w *World) SetAutoDisableTime(time Real) {
	C.dWorldSetAutoDisableTime(w.CID(), C.dReal(time))
}

/**
 * @brief Get auto disable flag for newly created bodies.
 * @ingroup disable
 * @return 0 or 1
 */
func (w *World) GetAutoDisableFlag() bool {
	return toBool( GetAutoDisableFlag(w.CID()) )
}

/**
 * @brief Set auto disable flag for newly created bodies.
 * @ingroup disable
 * @param do_auto_disable default is false.
 */
func (w *World) SetAutoDisableFlag(do_auto_disable bool) {
	C.dWorldSetAutoDisableFlag(w.CID(),toCint(do_auto_disable))
}

/**
 * @defgroup damping Damping
 * @ingroup bodies world
 *
 * Damping serves two purposes: reduce simulation instability, and to allow
 * the bodies to come to rest (and possibly auto-disabling them).
 *
 * Bodies are constructed using the world's current damping parameters. Setting
 * the scales to 0 disables the damping.
 *
 * Here is how it is done: after every time step linear and angular
 * velocities are tested against the corresponding thresholds. If they
 * are above, they are multiplied by (1 - scale). So a negative scale value
 * will actually increase the speed, and values greater than one will
 * make the object oscillate every step; both can make the simulation unstable.
 *
 * To disable damping just set the damping scale to zero.
 *
 * You can also limit the maximum angular velocity. In contrast to the damping
 * functions, the angular velocity is affected before the body is moved.
 * This means that it will introduce errors in joints that are forcing the body
 * to rotate too fast. Some bodies have naturally high angular velocities
 * (like cars' wheels), so you may want to give them a very high (like the default,
 * dInfinity) limit.
 *
 * @note The velocities are damped after the stepper function has moved the
 * object. Otherwise the damping could introduce errors in joints. First the
 * joint constraints are processed by the stepper (moving the body), then
 * the damping is applied.
 *
 * @note The damping happens right after the moved callback is called; this way 
 * it still possible use the exact velocities the body has acquired during the
 * step. You can even use the callback to create your own customized damping.
 */

/**
 * @brief Get the world's linear damping threshold.
 * @ingroup damping
 */
func (w *World) GetLinearDampingThreshold() Real {
	return Real(C.dWorldGetLinearDampingThreshold(w.CID()))
}

/**
 * @brief Set the world's linear damping threshold.
 * @param threshold The damping won't be applied if the linear speed is
 *        below this threshold. Default is 0.01.
 * @ingroup damping
 */
func (w *World) SetLinearDampingThreshold(threshold Real) {
	C.dWorldSetLinearDampingThreshold(w.CID(), C.dReal(threshold))
}

/**
 * @brief Get the world's angular damping threshold.
 * @ingroup damping
 */
func (w *World) GetAngularDampingThreshold() Real {
	return Real(dWorldGetAngularDampingThreshold(w.CID()))
}

/**
 * @brief Set the world's angular damping threshold.
 * @param threshold The damping won't be applied if the angular speed is
 *        below this threshold. Default is 0.01.
 * @ingroup damping
 */
func (w *World) SetAngularDampingThreshold(threshold Real) {
	C.dWorldSetAngularDampingThreshold(w.CID(), C.dReal(threshold))
}

/**
 * @brief Get the world's linear damping scale.
 * @ingroup damping
 */
func (w *World) GetLinearDamping() Real {
	return Real(C.dWorldGetLinearDamping(w.CID()))
}

/**
 * @brief Set the world's linear damping scale.
 * @param scale The linear damping scale that is to be applied to bodies.
 * Default is 0 (no damping). Should be in the interval [0, 1].
 * @ingroup damping
 */
func (w *World) SetLinearDamping(scale Real) {
	C.dWorldSetLinearDamping(w.CID(), C.dReal(scale))
}

/**
 * @brief Get the world's angular damping scale.
 * @ingroup damping
 */
func (w *World) GetAngularDamping() Real {
	return Real(C.dWorldGetAngularDamping(w.CID()))
}

/**
 * @brief Set the world's angular damping scale.
 * @param scale The angular damping scale that is to be applied to bodies.
 * Default is 0 (no damping). Should be in the interval [0, 1].
 * @ingroup damping
 */
func (w *World) SetAngularDamping(scale Real) {
	C.dWorldSetAngularDamping(w.CID(), C.dReal(scale))
}

/**
 * @brief Convenience function to set body linear and angular scales.
 * @param linear_scale The linear damping scale that is to be applied to bodies.
 * @param angular_scale The angular damping scale that is to be applied to bodies.
 * @ingroup damping
 */
 func (w *World) SetDamping( linear_scale, angular_scale Real) {
 	C.dWorldSetDamping(w.CID(), C.dReal(linear_scale), C.dReal(angular_scale))
 }

/**
 * @brief Get the default maximum angular speed.
 * @ingroup damping
 * @sa dBodyGetMaxAngularSpeed()
 */
func (w *World) GetMaxAngularSpeed() Real {
	return Real(C.dWorldGetMaxAngularSpeed(w.CID()))
}

/**
 * @brief Set the default maximum angular speed for new bodies.
 * @ingroup damping
 * @sa dBodySetMaxAngularSpeed()
 */
func (w *World) SetMaxAngularSpeed(max_speed Real) {
	C.dWorldSetMaxAngularSpeed(w.CID(), C.dReal(max_speed))
}

/**
 * @defgroup bodies Rigid Bodies
 *
 * A rigid body has various properties from the point of view of the
 * simulation. Some properties change over time:
 *
 *  @li Position vector (x,y,z) of the body's point of reference.
 *      Currently the point of reference must correspond to the body's center of mass.
 *  @li Linear velocity of the point of reference, a vector (vx,vy,vz).
 *  @li Orientation of a body, represented by a quaternion (qs,qx,qy,qz) or
 *      a 3x3 rotation matrix.
 *  @li Angular velocity vector (wx,wy,wz) which describes how the orientation
 *      changes over time.
 *
 * Other body properties are usually constant over time:
 *
 *  @li Mass of the body.
 *  @li Position of the center of mass with respect to the point of reference.
 *      In the current implementation the center of mass and the point of
 *      reference must coincide.
 *  @li Inertia matrix. This is a 3x3 matrix that describes how the body's mass
 *      is distributed around the center of mass. Conceptually each body has an
 *      x-y-z coordinate frame embedded in it that moves and rotates with the body.
 *
 * The origin of this coordinate frame is the body's point of reference. Some values
 * in ODE (vectors, matrices etc) are relative to the body coordinate frame, and others
 * are relative to the global coordinate frame.
 *
 * Note that the shape of a rigid body is not a dynamical property (except insofar as
 * it influences the various mass properties). It is only collision detection that cares
 * about the detailed shape of the body.
 */


/**
 * @brief Get auto disable linear average threshold.
 * @ingroup bodies disable
 * @return the threshold
 */

func (b *Body) GetAutoDisableLinearThreshold() Real {
	return Real(C.dBodyGetAutoDisableLinearThreshold(b.CID()))
}

/**
 * @brief Set auto disable linear average threshold.
 * @ingroup bodies disable
 * @return the threshold
 */
func (b *Body) SetAutoDisableLinearThreshold(linear_average_threshold Real) {
	C.dBodySetAutoDisableLinearThreshold( b.CID(), C.dReal(linear_average_threshold))
}

/**
 * @brief Get auto disable angular average threshold.
 * @ingroup bodies disable
 * @return the threshold
 */
func (b *Body) GetAutoDisableAngularThreshold() Real {
	return Real(C.dBodyGetAutoDisableAngularThreshold(b.CID()))
}

/**
 * @brief Set auto disable angular average threshold.
 * @ingroup bodies disable
 * @return the threshold
 */
func (b *Body) SetAutoDisableAngularThreshold(angular_average_threshold Real) {
	C.dBodySetAutoDisableAngularThreshold(b.CID(), C.dReal(angular_average_threshold))
}

/**
 * @brief Get auto disable average size (samples count).
 * @ingroup bodies disable
 * @return the nr of steps/size.
 */
func (b *Body) GetAutoDisableAverageSamplesCount() int {
	return int( C.dBodyGetAutoDisableAverageSamplesCount(b.CID()))
}

/**
 * @brief Set auto disable average buffer size (average steps).
 * @ingroup bodies disable
 * @param average_samples_count the nr of samples to review.
 */
func (b *Body) SetAutoDisableAverageSamplesCount( average_samples_count int) {
	C.dBodySetAutoDisableAverageSamplesCount(b.CID(), C.int(average_samples_count))
}

/**
 * @brief Get auto steps a body must be thought of as idle to disable
 * @ingroup bodies disable
 * @return the nr of steps
 */
func (b *Body) GetAutoDisableSteps() int {
	return int(C.dBodyGetAutoDisableSteps(b.CID()))
}

/**
 * @brief Set auto disable steps.
 * @ingroup bodies disable
 * @param steps the nr of steps.
 */
func (b *Body) SetAutoDisableSteps(steps int) {
	C.dBodySetAutoDisableSteps(b.CID(), C.int(steps))
}

/**
 * @brief Get auto disable time.
 * @ingroup bodies disable
 * @return nr of seconds
 */
func (b *Body) GetAutoDisableTime() Real {
	return Real(C.dBodyGetAutoDisableTime(b.CID()))
}

/**
 * @brief Set auto disable time.
 * @ingroup bodies disable
 * @param time nr of seconds.
 */
func (b *Body) SetAutoDisableTime(time Real) {
	C.SetAutoDisableTime(b.CID(), C.dReal(time))
}

/**
 * @brief Get auto disable flag.
 * @ingroup bodies disable
 * @return 0 or 1
 */
func (b *Body) GetAutoDisableFlag() bool {
	return toBool(C.dBodyGetAutoDisableFlag(b.CID()))
}

/**
 * @brief Set auto disable flag.
 * @ingroup bodies disable
 * @param do_auto_disable 0 or 1
 */
func (b *Body) SetAutoDisableFlag(do_auto_disable bool) {
	C.dBodySetAutoDisableFlag(b.CID(), toCint(do_auto_disable))
}

/**
 * @brief Set auto disable defaults.
 * @remarks
 * Set the values for the body to those set as default for the world.
 * @ingroup bodies disable
 */
func (b *Body) SetAutoDisableDefaults() {
	C.dBodySetAutoDisableDefaults(b.CID())
}


/**
 * @brief Retrieves the world attached to te given body.
 * @remarks
 * 
 * @ingroup bodies
 */
func (b *Body) GetWorld() *World {
	return (*World)((*C.struct_dxWorld)(C.dBodyGetWorld(b.CID())))
}

/**
 * @brief Create a body in given world.
 * @remarks
 * Default mass parameters are at position (0,0,0).
 * @ingroup bodies
 */
func (w *World) CreateBody() *Body {
	return (*Body)((*C.struct_dxBody)(C.dBodyCreate(w.CID())))
}

/**
 * @brief Destroy a body.
 * @remarks
 * All joints that are attached to this body will be put into limbo:
 * i.e. unattached and not affecting the simulation, but they will NOT be
 * deleted.
 * @ingroup bodies
 */
func (b *Body) Destroy() {
	C.dBodyDestroy(b.CID())
}

/**
 * @brief Set the body's user-data pointer.
 * @ingroup bodies
 * @param data arbitraty pointer
 */
func (b *Body) SetData(data unsafe.Pointer) {
	C.dBodySetData(b.CID(), data)
}

/**
 * @brief Get the body's user-data pointer.
 * @ingroup bodies
 * @return a pointer to the user's data.
 */
func (b *Body) GetData() unsafe.Pointer {
	return C.dBodyGetData(b.CID());
}

/**
 * @brief Set position of a body.
 * @remarks
 * After setting, the outcome of the simulation is undefined
 * if the new configuration is inconsistent with the joints/constraints
 * that are present.
 * @ingroup bodies
 */
func (b *Body) SetPosition(x,y,z Real) {
	C.dBodySetPosition(b.CID(), C.dReal(x),C.dReal(y),C.dReal(z))
}

/**
 * @brief Set the orientation of a body.
 * @ingroup bodies
 * @remarks
 * After setting, the outcome of the simulation is undefined
 * if the new configuration is inconsistent with the joints/constraints
 * that are present.
 */
func (b *Body) SetRotation(R *Matrix) {
	C.dBodySetRotation(b.CID(), (*C.dReal)(R[0]))
}

/**
 * @brief Set the orientation of a body.
 * @ingroup bodies
 * @remarks
 * After setting, the outcome of the simulation is undefined
 * if the new configuration is inconsistent with the joints/constraints
 * that are present.
 */
func (b *Body) SetQuaternion(q Quaternion) {
	C.dBodySetQuaternion(b.CID(), (*C.dReal)(q[0]));
}

/**
 * @brief Set the linear velocity of a body.
 * @ingroup bodies
 */
func (b *Body) SetLinearVel(x,y,z Real) {
	C.dBodySetLinearVel(b.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief Set the angular velocity of a body.
 * @ingroup bodies
 */
func (b *Body) SetAngularVel(x,y,z Real) {
	C.dBodySetAngularVel(b.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief Get the position of a body.
 * @ingroup bodies
 * @remarks
 * When getting, the returned values are pointers to internal data structures,
 * so the vectors are valid until any changes are made to the rigid body
 * system structure.
 * @sa dBodyCopyPosition
 */
func (b *Body)  GetPositionPointer() *Vector3 {
	ptr := (*Real)(C.dBodyGetPosition(b.CID()));
	return (*Vector3)((unsafe.Pointer)(ptr))
}

/**
 * @brief Copy the position of a body into a vector.
 * @ingroup bodies
 * @param body  the body to query
 * @param pos   a copy of the body position
 * @sa dBodyGetPosition
 */
func (b *Body) CopyPosition() (pos Vector3) {
	C.dBodyCopyPosition(b.CID(), (*C.dReal)(&pos[0]));
	return;
}

/**
 * @brief Get the rotation of a body.
 * @ingroup bodies
 * @return pointer to a 4x3 rotation matrix.
 */
func (b *Body)  GetRotationPointer() *Vector3 {
	ptr := (*Real)(C.dBodyGetRotation(b.CID()));
	return (*Vector3)((unsafe.Pointer)(ptr))
}


/**
 * @brief Copy the rotation of a body.
 * @ingroup bodies
 * @param body   the body to query
 * @param R      a copy of the rotation matrix
 * @sa dBodyGetRotation
 */
func (b *Body) CopyRotation() (R Matrix3) {
	C.dBodyCopyRotation(b.CID(), (*C.dReal)(&R[0]));
	return
}

/**
 * @brief Get the rotation of a body.
 * @ingroup bodies
 * @return pointer to 4 scalars that represent the quaternion.
 */
func (b *Body)  GetQuaternionPointer() *Quaternion {
	ptr := (*Real)(C.dBodyGetQuaternion(b.CID()));
	return (*Quaternion)((unsafe.Pointer)(ptr))
}

/**
 * @brief Copy the orientation of a body into a quaternion.
 * @ingroup bodies
 * @param body  the body to query
 * @param quat  a copy of the orientation quaternion
 * @sa dBodyGetQuaternion
 */
func (b *Body) CopyQuaternion() (quat Quaternion) {
	C.dBodyCopyQuaternion(b.CID(), (*C.dReal)(&quat[0]));
	return
}

/**
 * @brief Get the linear velocity of a body.
 * @ingroup bodies
 */
func (b *Body)  GetLinearVelPointer() *Vector3 {
	ptr := (*Real)(C.dBodyGetLinearVel(b.CID()));
	return (*Vector3)((unsafe.Pointer)(ptr))
}

/**
 * @brief Get the angular velocity of a body.
 * @ingroup bodies
 */
func (b *Body)  GetAngularVelPointer() *Vector3 {
	ptr := (*Real)(C.dBodyGetAngularVel(b.CID()));
	return (*Vector3)((unsafe.Pointer)(ptr))
}

/**
 * @brief Set the mass of a body.
 * @ingroup bodies
 */
func (b *Body) SetMass(mass *Mass) {
	C.dBodySetMass(b.CID(), (*C.dMass)(mass));
}


/**
 * @brief Get the mass of a body.
 * @ingroup bodies
 */
func (b *Body) GetMass(mass *Mass) {
	C.dBodyGetMass(b.CID(), (*C.dMass)(mass));
}

/**
 * @brief Add force at centre of mass of body in absolute coordinates.
 * @ingroup bodies
 */
func (b *Body) AddForce(x,y,z Real) {
	C.dBodyAddForce(b.CID(), C.dReal(fx), C.dReal(fy), C.dReal(fz))
}

/**
 * @brief Add torque at centre of mass of body in absolute coordinates.
 * @ingroup bodies
 */
func (b *Body) AddTorque(x,y,z Real) {
	C.dBodyAddTorque(b.CID(), C.dReal(fx), C.dReal(fy), C.dReal(fz))
}

/**
 * @brief Add force at centre of mass of body in coordinates relative to body.
 * @ingroup bodies
 */
func (b *Body) AddRelForce(x,y,z Real) {
	C.dBodyAddRelForce(b.CID(), C.dReal(fx), C.dReal(fy), C.dReal(fz))
}

/**
 * @brief Add torque at centre of mass of body in coordinates relative to body.
 * @ingroup bodies
 */
func (b *Body) AddRelTorque(x,y,z Real) {
	C.dBodyAddRelTorque(b.CID(), C.dReal(fx), C.dReal(fy), C.dReal(fz))
}

/**
 * @brief Add force at specified point in body in global coordinates.
 * @ingroup bodies
 */
func (b *Body) AddForceAtPos(fx,fy,fz,px,py,pz Real) {
	C.dBodyAddForceAtPos(b.CID(), C.dReal(fx), C.dReal(fy), C.dReal(fz),
			                C.dReal(px), C.dReal(py), C.dReal(pz))
}
/**
 * @brief Add force at specified point in body in local coordinates.
 * @ingroup bodies
 */
func (b *Body) AddForceAtRelPos(fx,fy,fz,px,py,pz Real) {
	C.dBodyAddForceAtRelPos(b.CID(), C.dReal(fx), C.dReal(fy), C.dReal(fz),
			                C.dReal(px), C.dReal(py), C.dReal(pz))
}
/**
 * @brief Add force at specified point in body in global coordinates.
 * @ingroup bodies
 */
func (b *Body) AddRelForceAtPos(fx,fy,fz,px,py,pz Real) {
	C.dBodyAddRelForceAtPos(b.CID(), C.dReal(fx), C.dReal(fy), C.dReal(fz),
			                C.dReal(px), C.dReal(py), C.dReal(pz))
}
/**
 * @brief Add force at specified point in body in local coordinates.
 * @ingroup bodies
 */
func (b *Body) AddRelForceAtRelPos(fx,fy,fz,px,py,pz Real) {
	C.dBodyAddRelForceAtRelPos(b.CID(), C.dReal(fx), C.dReal(fy), C.dReal(fz),
			                C.dReal(px), C.dReal(py), C.dReal(pz))
}

/**
 * @brief Return the current accumulated force vector.
 * @return points to an array of 3 reals.
 * @remarks
 * The returned values are pointers to internal data structures, so
 * the vectors are only valid until any changes are made to the rigid
 * body system.
 * @ingroup bodies
 */
func (b *Body) GetForcePointer() *[3]Real {
	 return (*[3]Real)(C.dBodyGetForce(b.CID()));
}

/**
 * @brief Return the current accumulated torque vector.
 * @return points to an array of 3 reals.
 * @remarks
 * The returned values are pointers to internal data structures, so
 * the vectors are only valid until any changes are made to the rigid
 * body system.
 * @ingroup bodies
 */
func (b *Body) GetTorque() Real {
	return Real(C.dBodyGetTorque(b.CID()))
}

/**
 * @brief Set the body force accumulation vector.
 * @remarks
 * This is mostly useful to zero the force and torque for deactivated bodies
 * before they are reactivated, in the case where the force-adding functions
 * were called on them while they were deactivated.
 * @ingroup bodies
 */
func (b *Body) SetForce(x,y,z Real) {
	C.dBodySetForce(b.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief Set the body torque accumulation vector.
 * @remarks
 * This is mostly useful to zero the force and torque for deactivated bodies
 * before they are reactivated, in the case where the force-adding functions
 * were called on them while they were deactivated.
 * @ingroup bodies
 */
func (b *Body) SetTorque(x,y,z Real) {
	C.dBodySetTorque (b.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief Get world position of a relative point on body.
 * @ingroup bodies
 * @param result will contain the result.
 */
func (b *Body) GetRelPointPos(px,py,pz Real) (result Vector3) {
	C.dBodyGetRelPointPos(b.CID(), C.dReal(px), C.dReal(py), C.dReal(pz), (*C.dVector3)(&result[0]))
}

/**
 * @brief Get velocity vector in global coords of a relative point on body.
 * @ingroup bodies
 * @param result will contain the result.
 */
func (b *Body) GetRelPointVel(px,py,pz Real) (result Vector3) {
	C.dBodyGetRelPointVel(b.CID(), C.dReal(px), C.dReal(py), C.dReal(pz), (*C.dVector3)(&result[0]))
}

/**
 * @brief Get velocity vector in global coords of a globally
 * specified point on a body.
 * @ingroup bodies
 * @param result will contain the result.
 */
func (b *Body) GetPointVel(px,py,pz Real) (result Vector3) {
	C.dBodyGetPointVel(b.CID(), C.dReal(px), C.dReal(py), C.dReal(pz), (*C.dVector3)(&result[0]))
}

/**
 * @brief takes a point in global coordinates and returns
 * the point's position in body-relative coordinates.
 * @remarks
 * This is the inverse of dBodyGetRelPointPos()
 * @ingroup bodies
 * @param result will contain the result.
 */
func (b *Body) GetPosRelPoint(px,py,pz Real) (result Vector3) {
	C.dBodyGetPosRelPoint(b.CID(), C.dReal(px), C.dReal(py), C.dReal(pz), (*C.dVector3)(&result[0]))
}

/**
 * @brief Convert from local to world coordinates.
 * @ingroup bodies
 * @param result will contain the result.
 */
func (b *Body) VectorToWorld(px,py,pz Real) (result Vector3) {
	C.dBodyVectorToWorld(b.CID(), C.dReal(px), C.dReal(py), C.dReal(pz), (*C.dVector3)(&result[0]))
}

/**
 * @brief Convert from world to local coordinates.
 * @ingroup bodies
 * @param result will contain the result.
 */
func (b *Body) VectorFromWorld(px, py, pz Real) (result Vector3) {
	C.dBodyVectorFromWorld(b.CID(), C.dReal(px), C.dReal(py), C.dReal(pz), (*C.dVector3)(&result[0]))
}

/**
 * @brief controls the way a body's orientation is updated at each timestep.
 * @ingroup bodies
 * @param mode can be 0 or 1:
 * \li 0: An ``infinitesimal'' orientation update is used.
 * This is fast to compute, but it can occasionally cause inaccuracies
 * for bodies that are rotating at high speed, especially when those
 * bodies are joined to other bodies.
 * This is the default for every new body that is created.
 * \li 1: A ``finite'' orientation update is used.
 * This is more costly to compute, but will be more accurate for high
 * speed rotations.
 * @remarks
 * Note however that high speed rotations can result in many types of
 * error in a simulation, and the finite mode will only fix one of those
 * sources of error.
 */
func (b *Body) SetFiniteRotationMode(mode int) {
	C.dBodySetFiniteRotationMode(b.dBodyID, C.int(mode))
}

/**
 * @brief sets the finite rotation axis for a body.
 * @ingroup bodies
 * @remarks
 * This is axis only has meaning when the finite rotation mode is set
 * If this axis is zero (0,0,0), full finite rotations are performed on
 * the body.
 * If this axis is nonzero, the body is rotated by performing a partial finite
 * rotation along the axis direction followed by an infinitesimal rotation
 * along an orthogonal direction.
 * @remarks
 * This can be useful to alleviate certain sources of error caused by quickly
 * spinning bodies. For example, if a car wheel is rotating at high speed
 * you can call this function with the wheel's hinge axis as the argument to
 * try and improve its behavior.
 */
func (b *Body) SetFiniteRotationAxis(x,y,z Real) {
	C.dBodySetFiniteRotationAxis(b.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief Get the way a body's orientation is updated each timestep.
 * @ingroup bodies
 * @return the mode 0 (infitesimal) or 1 (finite).
 */
func (b *Body) GetFiniteRotationMode() int {
	return int(C.dBodyGetFiniteRotationMode(b.CID()))
}

/**
 * @brief Get the finite rotation axis.
 * @param result will contain the axis.
 * @ingroup bodies
 */
func (b *Body) GetFiniteRotationAxis() (result Vector3) {
	C.dBodyGetFiniteRotationAxis(b.CID(), (*C.dReal)(&result[0]))
	return
}

/**
 * @brief Get the number of joints that are attached to this body.
 * @ingroup bodies
 * @return nr of joints
 */
func (b *Body) GetNumJoints() int {
	return int(C.dBodyGetNumJoints(b.CID()))
}

/**
 * @brief Return a joint attached to this body, given by index.
 * @ingroup bodies
 * @param index valid range is  0 to n-1 where n is the value returned by
 * dBodyGetNumJoints().
 */
func (b *Body) GetJoint(index int) *Joint {
	return JointPtr(C.dBodyGetJoint(b.CID(), C.int(index)))
}



/**
 * @brief Set rigid body to dynamic state (default).
 * @param dBodyID identification of body.
 * @ingroup bodies
 */
func (b *Body) SetDynamic() {
	C.dBodySetDynamic(b.CID())
}

/**
 * @brief Set rigid body to kinematic state.
 * When in kinematic state the body isn't simulated as a dynamic
 * body (it's "unstoppable", doesn't respond to forces),
 * but can still affect dynamic bodies (e.g. in joints).
 * Kinematic bodies can be controlled by position and velocity.
 * @note A kinematic body has infinite mass. If you set its mass
 * to something else, it loses the kinematic state and behaves
 * as a normal dynamic body.
 * @param dBodyID identification of body.
 * @ingroup bodies
 */
func (b *Body) SetKinematic() {
	C.dBodySetKinematic(b.CID());
}

/**
 * @brief Check wether a body is in kinematic state.
 * @ingroup bodies
 * @return 1 if a body is kinematic or 0 if it is dynamic.
 */
func (b *Body) IsKinematic() bool {
	return toBool(C.dBodyIsKinematic(dBodyID));
}

/**
 * @brief Manually enable a body.
 * @param dBodyID identification of body.
 * @ingroup bodies
 */
func (b *Body) Enable() {
	C.dBodyEnable(b.CID());
}

/**
 * @brief Manually disable a body.
 * @ingroup bodies
 * @remarks
 * A disabled body that is connected through a joint to an enabled body will
 * be automatically re-enabled at the next simulation step.
 */
func (b *Body) Disable() {
	C.dBodyDisable(b.CID())
}

/**
 * @brief Check wether a body is enabled.
 * @ingroup bodies
 * @return 1 if a body is currently enabled or 0 if it is disabled.
 */
func (b *Body) IsEnabled() bool {
	return toBool(C.dBodyIsEnabled(b.CID()))
}

/**
 * @brief Set whether the body is influenced by the world's gravity or not.
 * @ingroup bodies
 * @param mode when nonzero gravity affects this body.
 * @remarks
 * Newly created bodies are always influenced by the world's gravity.
 */
func (b *Body) SetGravityMode(mode bool) {
	C.dBodySetGravityMode(b.CID(), toCint(mode))
}

/**
 * @brief Get whether the body is influenced by the world's gravity or not.
 * @ingroup bodies
 * @return nonzero means gravity affects this body.
 */
func (b *Body) GetGravityMode() bool {
	return toBool(C.dBodyGetGravityMode(b.CID()))
}


/**
 * @brief Set the 'moved' callback of a body.
 *
 * Whenever a body has its position or rotation changed during the
 * timestep, the callback will be called (with body as the argument).
 * Use it to know which body may need an update in an external
 * structure (like a 3D engine).
 *
 * @param b the body that needs to be watched.
 * @param callback the callback to be invoked when the body moves. Set to zero
 * to disable.
 * @ingroup bodies
 */
func (b *Body) SetMovedCallback(callback func(*Body)) {
	movedCallbacks[b.CID()] = callback
	C.dBodySetMovedCallback(b.CID(), movedCallback)
}

var movedCallbacks [C.dBodyID]func(b Body)

//export moveCallback
func movedCallback(b C.dBodyID) {
	callbacks[b](b)
}


/**
 * @brief Return the first geom associated with the body.
 * 
 * You can traverse through the geoms by repeatedly calling
 * dBodyGetNextGeom().
 *
 * @return the first geom attached to this body, or 0.
 * @ingroup bodies
 */
func (b *Body) GetFirstGeom() *Geom {
	return (*Geom)((*C.struct_dxWorld)(C.dBodyGetFirstGeom(b.CID())))
}

/**
 * @brief returns the next geom associated with the same body.
 * @param g a geom attached to some body.
 * @return the next geom attached to the same body, or 0.
 * @sa dBodyGetFirstGeom
 * @ingroup bodies
 */
// ODE_API dGeomID dBodyGetNextGeom (dGeomID g);
// already implementey with dGeomGetBodyNext


/**
 * @brief Resets the damping settings to the current world's settings.
 * @ingroup bodies damping
 */
func (b *Body) SetDampingDefaults() {
	C.dBodySetDampingDefaults(b.CID())
}

/**
 * @brief Get the body's linear damping scale.
 * @ingroup bodies damping
 */
func (b *Body) GetLinearDamping() Real {
	return Real(C.dBodyGetLinearDamping(b.CID()))
}

/**
 * @brief Set the body's linear damping scale.
 * @param scale The linear damping scale. Should be in the interval [0, 1].
 * @ingroup bodies damping
 * @remarks From now on the body will not use the world's linear damping
 * scale until dBodySetDampingDefaults() is called.
 * @sa dBodySetDampingDefaults()
 */
func (b *Body) SetLinearDamping(scale Real) {
	C.dBodySetLinearDamping(b.CID(), C.dReal(scale))
}


/**
 * @brief Get the body's angular damping scale.
 * @ingroup bodies damping
 * @remarks If the body's angular damping scale was not set, this function
 * returns the world's angular damping scale.
 */
func (b *Body) GetAngularDamping() Real {
	return Real(C.dBodyGetAngularDamping(b.CID()))
}

/**
 * @brief Set the body's angular damping scale.
 * @param scale The angular damping scale. Should be in the interval [0, 1].
 * @ingroup bodies damping
 * @remarks From now on the body will not use the world's angular damping
 * scale until dBodyResetAngularDamping() is called.
 * @sa dBodyResetAngularDamping()
 */
func (b *Body) SetAngularDamping(scale Real) {
	 C.dBodySetAngularDamping(b.CID(), C.dReal(scale))
}

/**
 * @brief Convenience function to set linear and angular scales at once.
 * @param linear_scale The linear damping scale. Should be in the interval [0, 1].
 * @param angular_scale The angular damping scale. Should be in the interval [0, 1].
 * @ingroup bodies damping
 * @sa dBodySetLinearDamping() dBodySetAngularDamping()
 */
func (b *Body) SetDamping(linear_scale, angular_scale Real) {
	 C.dBodySetDamping(b.CID(), C.dReal(linear_scale), C.dReal(angular_scale))
}

/**
 * @brief Get the body's linear damping threshold.
 * @ingroup bodies damping
 */
func (b *Body) GetLinearDampingThreshold() Real {
	 return Real(C.dBodyGetLinearDampingThreshold(b.CID()));
}

/**
 * @brief Set the body's linear damping threshold.
 * @param threshold The linear threshold to be used. Damping
 *      is only applied if the linear speed is above this limit.
 * @ingroup bodies damping
 */
func (b *Body) SetLinearDampingThreshold() {
	 C.dBodySetLinearDampingThreshold(b.CID(), C.dReal(threshold))
}

/**
 * @brief Get the body's angular damping threshold.
 * @ingroup bodies damping
 */
func (b *Body) GetAngularDampingThreshold() Real {
	 return Real(C.dBodyGetAngularDampingThreshold(b.CID()))
}

/**
 * @brief Set the body's angular damping threshold.
 * @param threshold The angular threshold to be used. Damping is
 *      only used if the angular speed is above this limit.
 * @ingroup bodies damping
 */
func (b *Body) SetAngularDampingThreshold(threshold Real) {
	C.dBodySetAngularDampingThreshold(b.CID(), C.dReal(threshold))
}

/**
 * @brief Get the body's maximum angular speed.
 * @ingroup damping bodies
 * @sa dWorldGetMaxAngularSpeed()
 */
func (b *Body) GetMaxAngularSpeed() Real {
	return Real(C.dBodyGetMaxAngularSpeed(b.CID()));
}


/**
 * @brief Set the body's maximum angular speed.
 * @ingroup damping bodies
 * @sa dWorldSetMaxAngularSpeed() dBodyResetMaxAngularSpeed()
 * The default value is dInfinity, but it's a good idea to limit
 * it at less than 500 if the body has the gyroscopic term
 * enabled.
 */
func (b *Body) SetMaxAngularSpeed(max_speed Real) {
	C.dBodySetMaxAngularSpeed(C.dReal(max_speed))
}




/**
 * @brief Get the body's gyroscopic state.
 *
 * @return nonzero if gyroscopic term computation is enabled (default),
 * zero otherwise.
 * @ingroup bodies
 */
func (b *Body) GetGyroscopicMode() bool {
	return toInt(C.dBodyGetGyroscopicMode(b.CID()))
}

/**
 * @brief Enable/disable the body's gyroscopic term.
 *
 * Disabling the gyroscopic term of a body usually improves
 * stability. It also helps turning spining objects, like cars'
 * wheels.
 *
 * @param enabled   nonzero (default) to enable gyroscopic term, 0
 * to disable.
 * @ingroup bodies
 */
func (b *Body) SetGyroscopicMode(enabled bool) {
	C.dBodySetGyroscopicMode(b.CID(), toCint(enabled))
}





/**
 * @defgroup joints Joints
 *
 * In real life a joint is something like a hinge, that is used to connect two
 * objects.
 * In ODE a joint is very similar: It is a relationship that is enforced between
 * two bodies so that they can only have certain positions and orientations
 * relative to each other.
 * This relationship is called a constraint -- the words joint and
 * constraint are often used interchangeably.
 *
 * A joint has a set of parameters that can be set. These include:
 *
 *
 * \li  dParamLoStop Low stop angle or position. Setting this to
 *	-dInfinity (the default value) turns off the low stop.
 *	For rotational joints, this stop must be greater than -pi to be
 *	effective.
 * \li  dParamHiStop High stop angle or position. Setting this to
 *	dInfinity (the default value) turns off the high stop.
 *	For rotational joints, this stop must be less than pi to be
 *	effective.
 *	If the high stop is less than the low stop then both stops will
 *	be ineffective.
 * \li  dParamVel Desired motor velocity (this will be an angular or
 *	linear velocity).
 * \li  dParamFMax The maximum force or torque that the motor will use to
 *	achieve the desired velocity.
 *	This must always be greater than or equal to zero.
 *	Setting this to zero (the default value) turns off the motor.
 * \li  dParamFudgeFactor The current joint stop/motor implementation has
 *	a small problem:
 *	when the joint is at one stop and the motor is set to move it away
 *	from the stop, too much force may be applied for one time step,
 *	causing a ``jumping'' motion.
 *	This fudge factor is used to scale this excess force.
 *	It should have a value between zero and one (the default value).
 *	If the jumping motion is too visible in a joint, the value can be
 *	reduced.
 *	Making this value too small can prevent the motor from being able to
 *	move the joint away from a stop.
 * \li  dParamBounce The bouncyness of the stops.
 *	This is a restitution parameter in the range 0..1.
 *	0 means the stops are not bouncy at all, 1 means maximum bouncyness.
 * \li  dParamCFM The constraint force mixing (CFM) value used when not
 *	at a stop.
 * \li  dParamStopERP The error reduction parameter (ERP) used by the
 *	stops.
 * \li  dParamStopCFM The constraint force mixing (CFM) value used by the
 *	stops. Together with the ERP value this can be used to get spongy or
 *	soft stops.
 *	Note that this is intended for unpowered joints, it does not really
 *	work as expected when a powered joint reaches its limit.
 * \li  dParamSuspensionERP Suspension error reduction parameter (ERP).
 *	Currently this is only implemented on the hinge-2 joint.
 * \li  dParamSuspensionCFM Suspension constraint force mixing (CFM) value.
 *	Currently this is only implemented on the hinge-2 joint.
 *
 * If a particular parameter is not implemented by a given joint, setting it
 * will have no effect.
 * These parameter names can be optionally followed by a digit (2 or 3)
 * to indicate the second or third set of parameters, e.g. for the second axis
 * in a hinge-2 joint, or the third axis in an AMotor joint.
 */


/**
 * @brief Create a new joint of the ball type.
 * @ingroup joints
 * @remarks
 * The joint is initially in "limbo" (i.e. it has no effect on the simulation)
 * because it does not connect to any bodies.
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateBall(w *World) *Joint {
	return JointPtr(C.dJointCreateBall(w.CID, jg.CID()))
}

/**
 * @brief Create a new joint of the hinge type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateHinge(w *World) *Joint {
	return JointPtr(C.dJointCreateHinge(w.CID, jg.CID()))
}

/**
 * @brief Create a new joint of the slider type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateSlider(w *World) *Joint {
	return JointPtr(C.dJointCreateSlider(w.CID(), jg.CID()))
}

/**
 * @brief Create a new joint of the contact type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateContact(w *World, contact *Contact) *Joint {
	return JointPtr(C.dJointCreateContact(w.CID(), jg.CID, (*C.dContact)(contact)))
}

/**
 * @brief Create a new joint of the hinge2 type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateHinge2(w *World) *Joint {
	return JointPtr(C.dJointCreateHinge2(w.CID(), jg.CID()))
}

/**
 * @brief Create a new joint of the universal type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateUniversal(w *World) *Joint {
	return JointPtr(C.dJointCreateUniversal(w.CID(), jg.CID()))
}

/**
 * @brief Create a new joint of the PR (Prismatic and Rotoide) type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreatePR(w *World) *Joint {
	return JointPtr(C.dJointCreatePR(w.CID(), w.CID()))
}

  /**
   * @brief Create a new joint of the PU (Prismatic and Universal) type.
   * @ingroup joints
   * @param dJointGroupID set to 0 to allocate the joint normally.
   * If it is nonzero the joint is allocated in the given joint group.
   */
func (jg *JointGroup) CreatePU(w *World) *Joint {
 	return JointPtr(C.dJointCreatePU(w.CID(), jg.CID()))
}

  /**
   * @brief Create a new joint of the Piston type.
   * @ingroup joints
   * @param dJointGroupID set to 0 to allocate the joint normally.
   *                      If it is nonzero the joint is allocated in the given
   *                      joint group.
   */
func (jg *JointGroup) CreatePiston(w *World) *Joint {
 	return JointPtr(C.dJointCreatePiston(w.CID(), jg.CID()))
}

/**
 * @brief Create a new joint of the fixed type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateFixed(w *World) *Joint {
 	return JointPtr(C.dJointCreateFixed(w.CID(), jg.CID()))
}

func (jg *JointGroup) CreateNull(w *World) *Joint {
 	return JointPtr(C.dJointCreateNull(w.CID(), jg.CID()))
}

/**
 * @brief Create a new joint of the A-motor type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateAMotor(w *World) *Joint {
 	return JointPtr(C.dJointCreateAMotor(w.CID(), jg.CID()))
}

/**
 * @brief Create a new joint of the L-motor type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreateLMotor(w *World) *Joint {
 	return JointPtr(C.dJointCreateLMotor(w.CID(), jg.CID()))
}

/**
 * @brief Create a new joint of the plane-2d type.
 * @ingroup joints
 * @param dJointGroupID set to 0 to allocate the joint normally.
 * If it is nonzero the joint is allocated in the given joint group.
 */
func (jg *JointGroup) CreatePlane2D(w *World) *Joint {
 	return JointPtr(C.dJointCreatePlane2D(w.CID(), jg.CID()))
}

/**
 * @brief Destroy a joint.
 * @ingroup joints
 *
 * disconnects it from its attached bodies and removing it from the world.
 * However, if the joint is a member of a group then this function has no
 * effect - to destroy that joint the group must be emptied or destroyed.
 */
func (j *Jont) Destroy() {
	C.dJointDestroy(j.CID());
}

/**
 * @brief Create a joint group
 * @ingroup joints
 * @param max_size deprecated. Set to 0.
 */
func (jg *JointGroup) CreateJointGroup(max_size int) {
	return JointGroupPtr(C.dJointGroupCreate(C.int(max_size)))
}

/**
 * @brief Destroy a joint group.
 * @ingroup joints
 *
 * All joints in the joint group will be destroyed.
 */
func (jg *JointGroup) Destroy() {
	C.dJointGroupDestroy(jg.CID());
}

/**
 * @brief Empty a joint group.
 * @ingroup joints
 *
 * All joints in the joint group will be destroyed,
 * but the joint group itself will not be destroyed.
 */
func (jg *JointGroup) Empty() {
	C.dJointGroupEmpty(jg.CID());
}

/**
 * @brief Return the number of bodies attached to the joint
 * @ingroup joints
 */
func (j *Joint) GetNumBodies() int {
	return int(C.dJointGetNumBodies(j.CID()))
}

/**
 * @brief Attach the joint to some new bodies.
 * @ingroup joints
 *
 * If the joint is already attached, it will be detached from the old bodies
 * first.
 * To attach this joint to only one body, set body1 or body2 to zero - a zero
 * body refers to the static environment.
 * Setting both bodies to zero puts the joint into "limbo", i.e. it will
 * have no effect on the simulation.
 * @remarks
 * Some joints, like hinge-2 need to be attached to two bodies to work.
 */
func (j *Joint) Attach(body1 *Body, body2 *Body) {
	C.dJointAttach(j.CID(), body1.CID(), body2.CID());
}

/**
 * @brief Manually enable a joint.
 * @param dJointID identification of joint.
 * @ingroup joints
 */
func (j *Joint) Enable() {
	C.dJointEnable(j.CID());
}

/**
 * @brief Manually disable a joint.
 * @ingroup joints
 * @remarks
 * A disabled joint will not affect the simulation, but will maintain the anchors and
 * axes so it can be enabled later.
 */
func (j *Joint) Disable() {
	C.dJointDisable(j.CID())
}

/**
 * @brief Check wether a joint is enabled.
 * @ingroup joints
 * @return 1 if a joint is currently enabled or 0 if it is disabled.
 */
func (j *Joint) IsEnabled() {
	C.dJointIsEnabled(j.CID())
}

/**
 * @brief Set the user-data pointer
 * @ingroup joints
 */
func (j *Joint) SetData(data unsafe.Pointer) {
	C.dJointSetData(j.CID(),data)
}

/**
 * @brief Get the user-data pointer
 * @ingroup joints
 */
func (j *Joint) GetData() unsafe.Pointer {
	return C.dJointGetData(j.CID());
}

/**
 * @brief Get the type of the joint
 * @ingroup joints
 * @return the type, being one of these:
 * \li dJointTypeBall
 * \li dJointTypeHinge
 * \li dJointTypeSlider
 * \li dJointTypeContact
 * \li dJointTypeUniversal
 * \li dJointTypeHinge2
 * \li dJointTypeFixed
 * \li dJointTypeNull
 * \li dJointTypeAMotor
 * \li dJointTypeLMotor
 * \li dJointTypePlane2D
 * \li dJointTypePR
 * \li dJointTypePU
 * \li dJointTypePiston
 */
func (j *Joint) GetType() JointType {
	return JointType(C.dJointGetType(j.CID()))
}

/**
 * @brief Return the bodies that this joint connects.
 * @ingroup joints
 * @param index return the first (0) or second (1) body.
 * @remarks
 * If one of these returned body IDs is zero, the joint connects the other body
 * to the static environment.
 * If both body IDs are zero, the joint is in ``limbo'' and has no effect on
 * the simulation.
 */
func (j *Joint) GetBody(index int) *Body {
	return BodyPtr(C.dJointGetBody(j.CID(), C.int(index)))
}

/**
 * @brief Sets the datastructure that is to receive the feedback.
 *
 * The feedback can be used by the user, so that it is known how
 * much force an individual joint exerts.
 * @ingroup joints
 */
ODE_API void dJointSetFeedback (dJointID, dJointFeedback *);

/**
 * @brief Gets the datastructure that is to receive the feedback.
 * @ingroup joints
 */
ODE_API dJointFeedback *dJointGetFeedback (dJointID);

/**
 * @brief Set the joint anchor point.
 * @ingroup joints
 *
 * The joint will try to keep this point on each body
 * together. The input is specified in world coordinates.
 */
func (j *Joint) SetBallAnchor(x,y,z Real) {
	C.dJointSetBallAnchor(j.CID,C.dReal(x), C.dReal(y),C.dReal(z));
}

/**
 * @brief Set the joint anchor point.
 * @ingroup joints
 */
func (j *Joint) SetBallAnchor2(x,y,z Real) {
	C.dJointSetBallAnchor2(j.CID,C.dReal(x), C.dReal(y),C.dReal(z));
}

/**
 * @brief Param setting for Ball joints
 * @ingroup joints
 */
func (j *Joint) SetBallParam(parameter int, value Real) {
	C.dJointSetBallParam(j.CID(),C.int(parameter), C.dReal(value))
}

/**
 * @brief Set hinge anchor parameter.
 * @ingroup joints
 */
func (j *Joint) SetHingeAnchor(x,y,z Real) {
 	C.dJointSetHingeAnchor(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z))
 } 

func (j *Joint) SetHingeAnchorDelta(x,y,z,ax,ay,az Real) {
	C.dJointSetHingeAnchorDelta(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z), C.dReal(ax), C.dReal(ay), C.dReal(az));
}

/**
 * @brief Set hinge axis.
 * @ingroup joints
 */
func (j *Joint) SetHingeAxis(x,y,z Real) {
	C.dJointSetHingeAxis(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief Set the Hinge axis as if the 2 bodies were already at angle appart.
 * @ingroup joints
 *
 * This function initialize the Axis and the relative orientation of each body
 * as if body1 was rotated around the axis by the angle value. \br
 * Ex:
 * <PRE>
 * dJointSetHingeAxis(jId, 1, 0, 0);
 * // If you request the position you will have: dJointGetHingeAngle(jId) == 0
 * dJointSetHingeAxisDelta(jId, 1, 0, 0, 0.23);
 * // If you request the position you will have: dJointGetHingeAngle(jId) == 0.23
 * </PRE>

 * @param j The Hinge joint ID for which the axis will be set
 * @param x The X component of the axis in world frame
 * @param y The Y component of the axis in world frame
 * @param z The Z component of the axis in world frame
 * @param angle The angle for the offset of the relative orientation.
 *              As if body1 was rotated by angle when the Axis was set (see below).
 *              The rotation is around the new Hinge axis.
 *
 * @note Usually the function dJointSetHingeAxis set the current position of body1
 *       and body2 as the zero angle position. This function set the current position
 *       as the if the 2 bodies where \b angle appart.
 * @warning Calling dJointSetHingeAnchor or dJointSetHingeAxis will reset the "zero"
 *          angle position.
 */
func (j *Joint) SetHingeAxisOffset(x,y,z,angle Real) {
	C.dJointSetHingeAxisOffset(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z), C.dReal(angle));
}


/**
 * @brief set joint parameter
 * @ingroup joints
 */
func (j *Joint) SetHingeParam(parameter int, value Real) {
	C.dJointSetHingeParam(j.CID(), C.int(parameter), C.dReal(value));
}

/**
 * @brief Applies the torque about the hinge axis.
 *
 * That is, it applies a torque with specified magnitude in the direction
 * of the hinge axis, to body 1, and with the same magnitude but in opposite
 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
 * @ingroup joints
 */
func (j *Joint) AddHingeTorque(torque) {
	C.dJointAddHingeTorque(j.CID(), C.dReal(torque))
}


/**
 * @brief set the joint axis
 * @ingroup joints
 */
func (j *Joint) SetSliderAxis(x,y,z Real) {
	C.dJointSetSliderAxis(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}


/**
 * @ingroup joints
 */
func (j *Joint) SetSliderAxisDelta(x,y,z,ax,ay,az) {
	C.dJointSetSliderAxisDelta(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z), C.dReal(ax), C.dReal(ay), C.dReal(az));
}

/**
 * @brief set joint parameter
 * @ingroup joints
 */
func (j *Joint) SetSliderParam(parameter int, value Real) {
	C.dJointSetSliderParam(j.CID(), C.int(parameter), C.dReal(value))
}

/**
 * @brief Applies the given force in the slider's direction.
 *
 * That is, it applies a force with specified magnitude, in the direction of
 * slider's axis, to body1, and with the same magnitude but opposite
 * direction to body2.  This function is just a wrapper for dBodyAddForce().
 * @ingroup joints
 */
func (j *Joint) AddSliderFore(force Real) {
	C.dJointAddSliderForce(j.CID(), C.dReal(force))
}

/**
 * @brief set anchor
 * @ingroup joints
 */
func (j *Joint) SetHinge2Anchor(x,y,z Real) {
	C.dJointSetHinge2Anchor(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief set axis
 * @ingroup joints
 */
func (j *Joint) SetHinge2Axis1(x,y,z Real) {
	C.dJointSetHinge2Axis1(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z))
}

/**
 * @brief set axis
 * @ingroup joints
 */
func (j *Joint) SetHinge2Axis2(x,y,z Real) {
	C.dJointSetHinge2Axis2(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z))
}

/**
 * @brief set joint parameter
 * @ingroup joints
 */
func (j *Joint) SetHinge2Param(parameter int, value Real) {
	C.dJointSetHinge2Param(j.CID(), C.int(parameter), C.dReal(value))
} 

/**
 * @brief Applies torque1 about the hinge2's axis 1, torque2 about the
 * hinge2's axis 2.
 * @remarks  This function is just a wrapper for dBodyAddTorque().
 * @ingroup joints
 */
func (j *Joint) AddHinge2Torques() {
	C.dJointAddHinge2Torques(j.CID(),   C.dReal(torque1), C.dReal(torque2))
}

/**
 * @brief set anchor
 * @ingroup joints
 */
func (j *Joint) SetUniversalAnchor() {
	C.dJointSetUniversalAnchor(j.CID(),C.dReal(x), C.dReal(y), C.dReal(z))
}

/**
 * @brief set axis
 * @ingroup joints
 */
func (j *Joint) SetUniversalAxis1() {
	C.dJointSetUniversalAxis1(j.CID(),C.dReal(x), C.dReal(y), C.dReal(z))
}

/**
 * @brief Set the Universal axis1 as if the 2 bodies were already at 
 *        offset1 and offset2 appart with respect to axis1 and axis2.
 * @ingroup joints
 *
 * This function initialize the axis1 and the relative orientation of 
 * each body as if body1 was rotated around the new axis1 by the offset1 
 * value and as if body2 was rotated around the axis2 by offset2. \br
 * Ex:
* <PRE>
 * dJointSetHuniversalAxis1(jId, 1, 0, 0);
 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0
 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0
 * dJointSetHuniversalAxis1Offset(jId, 1, 0, 0, 0.2, 0.17);
 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0.2
 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0.17
 * </PRE>
 *
 * @param j The Hinge joint ID for which the axis will be set
 * @param x The X component of the axis in world frame
 * @param y The Y component of the axis in world frame
 * @param z The Z component of the axis in world frame
 * @param angle The angle for the offset of the relative orientation.
 *              As if body1 was rotated by angle when the Axis was set (see below).
 *              The rotation is around the new Hinge axis.
 *
 * @note Usually the function dJointSetHingeAxis set the current position of body1
 *       and body2 as the zero angle position. This function set the current position
 *       as the if the 2 bodies where \b offsets appart.
 *
 * @note Any previous offsets are erased.
 *
 * @warning Calling dJointSetUniversalAnchor, dJointSetUnivesalAxis1, 
 *          dJointSetUniversalAxis2, dJointSetUniversalAxis2Offset 
 *          will reset the "zero" angle position.
 */
func (j *Joint) SetUniversalAxis1Offset(x,y,z,offset1,offset2 Real) {
	C.dJointSetUniversalAxis1Offset(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z), C.dReal(offset1), C.dReal(offset2));
}

/**
 * @brief set axis
 * @ingroup joints
 */
func (j *Joint) SetUniversalAxis2(x,y,z Real) {
	C.dJointSetUniversalAxis2(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}



/**
 * @brief Set the Universal axis2 as if the 2 bodies were already at 
 *        offset1 and offset2 appart with respect to axis1 and axis2.
 * @ingroup joints
 *
 * This function initialize the axis2 and the relative orientation of 
 * each body as if body1 was rotated around the axis1 by the offset1 
 * value and as if body2 was rotated around the new axis2 by offset2. \br
 * Ex:
 * <PRE>
 * dJointSetHuniversalAxis2(jId, 0, 1, 0);
 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0
 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0
 * dJointSetHuniversalAxis2Offset(jId, 0, 1, 0, 0.2, 0.17);
 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0.2
 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0.17
 * </PRE>

 * @param j The Hinge joint ID for which the axis will be set
 * @param x The X component of the axis in world frame
 * @param y The Y component of the axis in world frame
 * @param z The Z component of the axis in world frame
 * @param angle The angle for the offset of the relative orientation.
 *              As if body1 was rotated by angle when the Axis was set (see below).
 *              The rotation is around the new Hinge axis.
 *
 * @note Usually the function dJointSetHingeAxis set the current position of body1
 *       and body2 as the zero angle position. This function set the current position
 *       as the if the 2 bodies where \b offsets appart.
 *
 * @note Any previous offsets are erased.
 *
 * @warning Calling dJointSetUniversalAnchor, dJointSetUnivesalAxis1, 
 *          dJointSetUniversalAxis2, dJointSetUniversalAxis2Offset 
 *          will reset the "zero" angle position.
 */


func (j *Joint) SetUniversalAxis2Offset(x,y,z, offset1, offset2 Real) {
	C.dJointSetUniversalAxis2Offset(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z), C.dReal(offset1), C.dReal(offset2))
}


/**
 * @brief set joint parameter
 * @ingroup joints
 */
func (j *Joint) SetUniversalParam(parameter int, value Real) {
	C.dJointSetUniversalParam(j.CID(), C.int(parameter), C.dReal(value))
}

/**
 * @brief Applies torque1 about the universal's axis 1, torque2 about the
 * universal's axis 2.
 * @remarks This function is just a wrapper for dBodyAddTorque().
 * @ingroup joints
 */
func (j *Joint) AddUniversalTorques(torque1, torque2 Real) {
	C.dJointAddUniversalTorques(j.CID(), C.dReal(torque1), C.dReal(torque2))
}

/**
 * @brief set anchor
 * @ingroup joints
 */
func (j *Joint) SetPRAnchor(x,y,z Real) {
	C.dJointSetPRAnchor(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief set the axis for the prismatic articulation
 * @ingroup joints
 */
func (j *Joint) SetPRAxis1(x,y,z Real) {
	C.dJointSetPRAxis1(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief set the axis for the rotoide articulation
 * @ingroup joints
 */
func (j *Joint) SetPRAxis2(x,y,z Real) {
	C.dJointSetPRAxis2(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

/**
 * @brief set joint parameter
 * @ingroup joints
 *
 * @note parameterX where X equal 2 refer to parameter for the rotoide articulation
 */
func (j *Joint) SetPRParam(parameter int, value Real) {
	C.dJointSetPRParam(j.CID(), C.int(parameter), C.dReal(value))
}

/**
 * @brief Applies the torque about the rotoide axis of the PR joint
 *
 * That is, it applies a torque with specified magnitude in the direction 
 * of the rotoide axis, to body 1, and with the same magnitude but in opposite
 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
 * @ingroup joints
 */
func (j *Joint) AddPRTorque(torque Real) {
	C.dJointAddPRTorque(j.CID(), C.dReal(torque));
}


  /**
  * @brief set anchor
  * @ingroup joints
  */
func (j *Joint) SetPUAnchor(x,y,z Real) {
	C.dJointSetPUAnchor(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z));
}

  /**
   * @brief Set the PU anchor as if the 2 bodies were already at [dx, dy, dz] appart.
   * @ingroup joints
   *
   * This function initialize the anchor and the relative position of each body
   * as if the position between body1 and body2 was already the projection of [dx, dy, dz]
   * along the Piston axis. (i.e as if the body1 was at its current position - [dx,dy,dy] when the
   * axis is set).
   * Ex:
   * <PRE>
   * dReal offset = 3;
   * dVector3 axis;
   * dJointGetPUAxis(jId, axis);
   * dJointSetPUAnchor(jId, 0, 0, 0);
   * // If you request the position you will have: dJointGetPUPosition(jId) == 0
   * dJointSetPUAnchorOffset(jId, 0, 0, 0, axis[X]*offset, axis[Y]*offset, axis[Z]*offset);
   * // If you request the position you will have: dJointGetPUPosition(jId) == offset
   * </PRE>
   * @param j The PU joint for which the anchor point will be set
   * @param x The X position of the anchor point in world frame
   * @param y The Y position of the anchor point in world frame
   * @param z The Z position of the anchor point in world frame
   * @param dx A delta to be substracted to the X position as if the anchor was set
   *           when body1 was at current_position[X] - dx
   * @param dx A delta to be substracted to the Y position as if the anchor was set
   *           when body1 was at current_position[Y] - dy
   * @param dx A delta to be substracted to the Z position as if the anchor was set
   *           when body1 was at current_position[Z] - dz
   */


func (j *Joint) SetPUAnchorOffset(x,y,z,dx,dy,dz Real) {
	C.dJointSetPUAnchorOffset(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z),
                                       C.dReal(dx), C.dReal(dy), C.dReal(dz))
}

  /**
   * @brief set the axis for the first axis or the universal articulation
   * @ingroup joints
   */

func (j *Joint) SetPUAxis1(x,y,z Real) {
	C.dJointSetPUAxis1(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z))
}

  /**
   * @brief set the axis for the second axis or the universal articulation
   * @ingroup joints
   */

func (j *Joint) SetPUAxis2(x,y,z Real) {
	C.dJointSetPUAxis2(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z))
}

  /**
   * @brief set the axis for the prismatic articulation
   * @ingroup joints
   */

func (j *Joint) SetPUAxis3(x,y,z Real) {
	C.dJointSetPUAxis3(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z))
}

  /**
   * @brief set the axis for the prismatic articulation
   * @ingroup joints
   * @note This function was added for convenience it is the same as
   *       dJointSetPUAxis3
   */

func (j *Joint) SetPUAxisP(x,y,z Real) {
	C.dJointSetPUAxisP(j.CID(), C.dReal(x), C.dReal(y), C.dReal(z))
}



  /**
   * @brief set joint parameter
   * @ingroup joints
   *
   * @note parameterX where X equal 2 refer to parameter for second axis of the
   *       universal articulation
   * @note parameterX where X equal 3 refer to parameter for prismatic
   *       articulation
   */
  ODE_API void dJointSetPUParam (dJointID, int parameter, dReal value);

  /**
   * @brief Applies the torque about the rotoide axis of the PU joint
   *
   * That is, it applies a torque with specified magnitude in the direction
   * of the rotoide axis, to body 1, and with the same magnitude but in opposite
   * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
   * @ingroup joints
   */
  ODE_API void dJointAddPUTorque (dJointID j, dReal torque);




  /**
   * @brief set the joint anchor
   * @ingroup joints
   */
  ODE_API void dJointSetPistonAnchor (dJointID, dReal x, dReal y, dReal z);

  /**
   * @brief Set the Piston anchor as if the 2 bodies were already at [dx,dy, dz] appart.
   * @ingroup joints
   *
   * This function initialize the anchor and the relative position of each body
   * as if the position between body1 and body2 was already the projection of [dx, dy, dz]
   * along the Piston axis. (i.e as if the body1 was at its current position - [dx,dy,dy] when the
   * axis is set).
   * Ex:
   * <PRE>
   * dReal offset = 3;
   * dVector3 axis;
   * dJointGetPistonAxis(jId, axis);
   * dJointSetPistonAnchor(jId, 0, 0, 0);
   * // If you request the position you will have: dJointGetPistonPosition(jId) == 0
   * dJointSetPistonAnchorOffset(jId, 0, 0, 0, axis[X]*offset, axis[Y]*offset, axis[Z]*offset);
   * // If you request the position you will have: dJointGetPistonPosition(jId) == offset
   * </PRE>
   * @param j The Piston joint for which the anchor point will be set
   * @param x The X position of the anchor point in world frame
   * @param y The Y position of the anchor point in world frame
   * @param z The Z position of the anchor point in world frame
   * @param dx A delta to be substracted to the X position as if the anchor was set
   *           when body1 was at current_position[X] - dx
   * @param dx A delta to be substracted to the Y position as if the anchor was set
   *           when body1 was at current_position[Y] - dy
   * @param dx A delta to be substracted to the Z position as if the anchor was set
   *           when body1 was at current_position[Z] - dz
   */
  ODE_API void dJointSetPistonAnchorOffset(dJointID j, dReal x, dReal y, dReal z,
                                           dReal dx, dReal dy, dReal dz);

    /**
     * @brief set the joint axis
   * @ingroup joints
   */
  ODE_API void dJointSetPistonAxis (dJointID, dReal x, dReal y, dReal z);

  /**
   * This function set prismatic axis of the joint and also set the position
   * of the joint.
   *
   * @ingroup joints
   * @param j The joint affected by this function
   * @param x The x component of the axis
   * @param y The y component of the axis
   * @param z The z component of the axis
   * @param dx The Initial position of the prismatic join in the x direction
   * @param dy The Initial position of the prismatic join in the y direction
   * @param dz The Initial position of the prismatic join in the z direction
   */
  ODE_API_DEPRECATED ODE_API void dJointSetPistonAxisDelta (dJointID j, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);

  /**
   * @brief set joint parameter
   * @ingroup joints
   */
  ODE_API void dJointSetPistonParam (dJointID, int parameter, dReal value);

  /**
   * @brief Applies the given force in the slider's direction.
   *
   * That is, it applies a force with specified magnitude, in the direction of
   * prismatic's axis, to body1, and with the same magnitude but opposite
   * direction to body2.  This function is just a wrapper for dBodyAddForce().
   * @ingroup joints
   */
  ODE_API void dJointAddPistonForce (dJointID joint, dReal force);


/**
 * @brief Call this on the fixed joint after it has been attached to
 * remember the current desired relative offset and desired relative
 * rotation between the bodies.
 * @ingroup joints
 */
ODE_API void dJointSetFixed (dJointID);

/*
 * @brief Sets joint parameter
 *
 * @ingroup joints
 */
ODE_API void dJointSetFixedParam (dJointID, int parameter, dReal value);

/**
 * @brief set the nr of axes
 * @param num 0..3
 * @ingroup joints
 */
ODE_API void dJointSetAMotorNumAxes (dJointID, int num);

/**
 * @brief set axis
 * @ingroup joints
 */
ODE_API void dJointSetAMotorAxis (dJointID, int anum, int rel,
			  dReal x, dReal y, dReal z);

/**
 * @brief Tell the AMotor what the current angle is along axis anum.
 *
 * This function should only be called in dAMotorUser mode, because in this
 * mode the AMotor has no other way of knowing the joint angles.
 * The angle information is needed if stops have been set along the axis,
 * but it is not needed for axis motors.
 * @ingroup joints
 */
ODE_API void dJointSetAMotorAngle (dJointID, int anum, dReal angle);

/**
 * @brief set joint parameter
 * @ingroup joints
 */
ODE_API void dJointSetAMotorParam (dJointID, int parameter, dReal value);

/**
 * @brief set mode
 * @ingroup joints
 */
ODE_API void dJointSetAMotorMode (dJointID, int mode);

/**
 * @brief Applies torque0 about the AMotor's axis 0, torque1 about the
 * AMotor's axis 1, and torque2 about the AMotor's axis 2.
 * @remarks
 * If the motor has fewer than three axes, the higher torques are ignored.
 * This function is just a wrapper for dBodyAddTorque().
 * @ingroup joints
 */
ODE_API void dJointAddAMotorTorques (dJointID, dReal torque1, dReal torque2, dReal torque3);

/**
 * @brief Set the number of axes that will be controlled by the LMotor.
 * @param num can range from 0 (which effectively deactivates the joint) to 3.
 * @ingroup joints
 */
ODE_API void dJointSetLMotorNumAxes (dJointID, int num);

/**
 * @brief Set the AMotor axes.
 * @param anum selects the axis to change (0,1 or 2).
 * @param rel Each axis can have one of three ``relative orientation'' modes
 * \li 0: The axis is anchored to the global frame.
 * \li 1: The axis is anchored to the first body.
 * \li 2: The axis is anchored to the second body.
 * @remarks The axis vector is always specified in global coordinates
 * regardless of the setting of rel.
 * @ingroup joints
 */
func (j *Joint) SetLMotorAxis(anum, rel int, x,y,z Real) {
	C.dJointSetLMotorAxis(j.CID(), C.int(anum), C.int(rel), C.dReal(x), C.dReal(y), C.dReal(z))
}

/**
 * @brief set joint parameter
 * @ingroup joints
 */
func (j *Joint) SetLMotorParam(parameter int, value Real) {
	C.dJointSetLMotorParam(j.CID(), C.int(parameter), C.dReal(value))
}

/**
 * @ingroup joints
 */
func (j *Joint) SetPlane2DXParam(parameter int, value Real) {
	C.dJointSetPlane2DXParam(j.CID(), C.int(parameter), C.dReal(value))
}

/**
 * @ingroup joints
 */

func (j *Joint) SetPlane2DYParam(parameter int, value Real) {
	C.dJointSetPlane2DYParam(j.CID(), C.int(parameter), C.dReal(value))
}

/**
 * @ingroup joints
 */
func (j *Joint) SetPlane2DAngleParam(parameter int, value Real) {
	C.dJointSetPlane2DAngleParam(j.CID(), C.int(parameter), C.dReal(value))
}

/**
 * @brief Get the joint anchor point, in world coordinates.
 *
 * This returns the point on body 1. If the joint is perfectly satisfied,
 * this will be the same as the point on body 2.
 */
func (j *Joint) GetBallAnchor() (result Vector3) {
	C.dJointGetBallAnchor(j.CID(), (*dReal)(&result[0]))
	return;
}

/**
 * @brief Get the joint anchor point, in world coordinates.
 *
 * This returns the point on body 2. You can think of a ball and socket
 * joint as trying to keep the result of dJointGetBallAnchor() and
 * dJointGetBallAnchor2() the same.  If the joint is perfectly satisfied,
 * this function will return the same value as dJointGetBallAnchor() to
 * within roundoff errors. dJointGetBallAnchor2() can be used, along with
 * dJointGetBallAnchor(), to see how far the joint has come apart.
 */
func (j *Joint) GetBallAnchor2() (result Vector3) {
	C.dJointGetBallAnchor2(j.CID(), (*dReal)(&result[0]))
	return;
}

/**
 * @brief get joint parameter
 * @ingroup joints
 */
func (j *Joint) GetBallParam() Real {
	return Real(C.dJointGetBallParam(j.CID(), C.int(parameter)))
}

/**
 * @brief Get the hinge anchor point, in world coordinates.
 *
 * This returns the point on body 1. If the joint is perfectly satisfied,
 * this will be the same as the point on body 2.
 * @ingroup joints
 */

func (j *Joint) GetHingeAnchor() (result Vector3) {
	C.dJointGetHingeAnchor(j.CID(), (*C.dReal)(&result[0]))
	return
}

/**
 * @brief Get the joint anchor point, in world coordinates.
 * @return The point on body 2. If the joint is perfectly satisfied,
 * this will return the same value as dJointGetHingeAnchor().
 * If not, this value will be slightly different.
 * This can be used, for example, to see how far the joint has come apart.
 * @ingroup joints
 */

func (j *Joint) GetHingeAnchor2() (result Vector3) {
	C.dJointGetHingeAnchor2(j.CID(), (*C.dReal)(&result[0]))
	return
}

/**
 * @brief get axis
 * @ingroup joints
 */

func (j *Joint) GetHingeAxis() (result Vector3) {
	C.dJointGetHingeAxis(j.CID(), (*C.dReal)(&result[0]))
	return
}

/**
 * @brief get joint parameter
 * @ingroup joints
 */

func (j *Joint) GetHingeParam(parameter int) Real {
	return Real(C.dJointGetHingeParam(j.CID(), int parameter)
}


/**
 * @brief Get the hinge angle.
 *
 * The angle is measured between the two bodies, or between the body and
 * the static environment.
 * The angle will be between -pi..pi.
 * Give the relative rotation with respect to the Hinge axis of Body 1 with
 * respect to Body 2.
 * When the hinge anchor or axis is set, the current position of the attached
 * bodies is examined and that position will be the zero angle.
 * @ingroup joints
 */

func (j *Joint) GetHingeAngle() Real {
	return Real(C.dJointGetHingeAngle(j.CID())
}


/**
 * @brief Get the hinge angle time derivative.
 * @ingroup joints
 */

func (j *Joint) GetHingeAngleRate() Real {
	return Real(C.dJointGetHingeAngleRate(j.CID())
}


/**
 * @brief Get the slider linear position (i.e. the slider's extension)
 *
 * When the axis is set, the current position of the attached bodies is
 * examined and that position will be the zero position.

 * The position is the distance, with respect to the zero position,
 * along the slider axis of body 1 with respect to
 * body 2. (A NULL body is replaced by the world).
 * @ingroup joints
 */

func (j *Joint) GetSliderPosition() Real {
	return Real(C.dJointGetSliderPosition(j.CID())
}


/**
 * @brief Get the slider linear position's time derivative.
 * @ingroup joints
 */

func (j *Joint) GetSliderPositionRate() Real {
	return Real(C.dJointGetSliderPositionRate(j.CID())
}


/**
 * @brief Get the slider axis
 * @ingroup joints
 */

func (j *Joint) GetSliderAxis() (result Vector3) {
	C.dJointGetSliderAxis(j.CID(), (*C.dReal)(&result[0]))
	return
}


/**
 * @brief get joint parameter
 * @ingroup joints
 */

func (j *Joint) GetSliderParam(parameter int) Real {
	return Real(C.dJointGetSliderParam(j.CID(), C.int(parameter))
}


/**
 * @brief Get the joint anchor point, in world coordinates.
 * @return the point on body 1.  If the joint is perfectly satisfied,
 * this will be the same as the point on body 2.
 * @ingroup joints
 */

func (j *Joint) GetHinge2Anchor() (result Vector3) {
	C.dJointGetHinge2Anchor(j.CID(), (*C.dReal)(&result[0]))
}


/**
 * @brief Get the joint anchor point, in world coordinates.
 * This returns the point on body 2. If the joint is perfectly satisfied,
 * this will return the same value as dJointGetHinge2Anchor.
 * If not, this value will be slightly different.
 * This can be used, for example, to see how far the joint has come apart.
 * @ingroup joints
 */

func (j *Joint) GetHinge2Anchor2() (result Vector3) {
	C.dJointGetHinge2Anchor2(j.CID(), (*C.dReal)(&result[0]))
}


/**
 * @brief Get joint axis
 * @ingroup joints
 */

func (j *Joint) GetHinge2Axis1() (result Vector3) {
	C.dJointGetHinge2Axis1(j.CID(), (*C.dReal)(&result[0]))
}


/**
 * @brief Get joint axis
 * @ingroup joints
 */

func (j *Joint) GetHinge2Axis2() (result Vector3) {
	C.dJointGetHinge2Axis2(j.CID(), (*C.dReal)(&result[0]))
}


/**
 * @brief get joint parameter
 * @ingroup joints
 */

func (j *Joint) GetHinge2Param(parameter int) Real {
	return Real(C.dJointGetHinge2Param(j.CID(), C.int(parameter))
}


/**
 * @brief Get angle
 * @ingroup joints
 */

func (j *Joint) GetHinge2Angle1() Real {
	return Real(C.dJointGetHinge2Angle1(j.CID());
}


/**
 * @brief Get time derivative of angle
 * @ingroup joints
 */

func (j *Joint) GetHinge2Angle1Rate() Real {
	return Real(C.dJointGetHinge2Angle1Rate(j.CID());
}


/**
 * @brief Get time derivative of angle
 * @ingroup joints
 */

func (j *Joint) GetHinge2Angle2Rate() Real {
	return Real(C.dJointGetHinge2Angle2Rate(j.CID());
}


/**
 * @brief Get the joint anchor point, in world coordinates.
 * @return the point on body 1. If the joint is perfectly satisfied,
 * this will be the same as the point on body 2.
 * @ingroup joints
 */

func (j *Joint) GetUniversalAnchor() (result Vector3) {
	C.dJointGetUniversalAnchor(j.CID(), (*C.dReal)(&result[0]))
}

/**
 * @brief Get the joint anchor point, in world coordinates.
 * @return This returns the point on body 2.
 * @remarks
 * You can think of the ball and socket part of a universal joint as
 * trying to keep the result of dJointGetBallAnchor() and
 * dJointGetBallAnchor2() the same. If the joint is
 * perfectly satisfied, this function will return the same value
 * as dJointGetUniversalAnchor() to within roundoff errors.
 * dJointGetUniversalAnchor2() can be used, along with
 * dJointGetUniversalAnchor(), to see how far the joint has come apart.
 * @ingroup joints
 */

func (j *Joint) GetUniversalAnchor2() (result Vector3) {
	C.dJointGetUniversalAnchor2(j.CID(), (*C.dReal)(&result[0]))
}

/**
 * @brief Get axis
 * @ingroup joints
 */

func (j *Joint) GetUniversalAxis1() (result Vector3) {
	C.dJointGetUniversalAxis1(j.CID(), (*C.dReal)(&result[0]))
}

/**
 * @brief Get axis
 * @ingroup joints
 */

func (j *Joint) GetUniversalAxis2() (result Vector3) {
	C.dJointGetUniversalAxis2(j.CID(), (*C.dReal)(&result[0]))
}


/**
 * @brief get joint parameter
 * @ingroup joints
 */

func (j *Joint) GetUniversalParam(parameter int) Real {
	return Real(C.dJointGetUniversalParam(j.CID(), C.int(parameter)))
}


/**
 * @brief Get both angles at the same time.
 * @ingroup joints
 *
 * @param joint   The universal joint for which we want to calculate the angles
 * @param angle1  The angle between the body1 and the axis 1
 * @param angle2  The angle between the body2 and the axis 2
 *
 * @note This function combine getUniversalAngle1 and getUniversalAngle2 together
 *       and try to avoid redundant calculation
 */
func (j *Joint) GetUniversalAngles() (angle1, angle2 Real) {
 	C.dJointGetUniversalAngles(j.CID(), (*C.dReal)(&angle1), (*C.dReal)(&angle2))
 	return
}

/**
 * @brief Get angle
 * @ingroup joints
 */

func (j *Joint) GetUniversalAngle1() Real {
	return Real(C.dJointGetUniversalAngle1(j.CID()))
}


/**
 * @brief Get angle
 * @ingroup joints
 */

func (j *Joint) GetUniversalAngle2() Real {
	return Real(C.dJointGetUniversalAngle2(j.CID()))
}


/**
 * @brief Get time derivative of angle
 * @ingroup joints
 */

func (j *Joint) GetUniversalAngle1Rate() Real {
	return Real(C.dJointGetUniversalAngle1Rate(j.CID()))
}


/**
 * @brief Get time derivative of angle
 * @ingroup joints
 */

func (j *Joint) GetUniversalAngle2Rate() Real {
	return Real(C.dJointGetUniversalAngle2Rate(j.CID()))
}




/**
 * @brief Get the joint anchor point, in world coordinates.
 * @return the point on body 1. If the joint is perfectly satisfied, 
 * this will be the same as the point on body 2.
 * @ingroup joints
 */

func (j *Joint) GetPRAnchor() (result Vector3) {
	C.dJointGetPRAnchor(j.CID(), (*C.dReal)(&result[0]))
	return
}


/**
 * @brief Get the PR linear position (i.e. the prismatic's extension)
 *
 * When the axis is set, the current position of the attached bodies is
 * examined and that position will be the zero position.
 *
 * The position is the "oriented" length between the
 * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
 *
 * @ingroup joints
 */
func (j *Joint) dJointGetPRPosition() Real {
	return Real(C.dJointGetPRPosition(j.CID()))
}

/**
 * @brief Get the PR linear position's time derivative
 *
 * @ingroup joints
 */
func (j *Joint) dJointGetPRPositionRate() Real {
	return Real(C.dJointGetPRPositionRate(j.CID()))
}


/**
   * @brief Get the PR angular position (i.e. the  twist between the 2 bodies)
   *
   * When the axis is set, the current position of the attached bodies is
   * examined and that position will be the zero position.
   * @ingroup joints
   */
func (j *Joint) dJointGetPRAngle() Real {
	return Real(C.dJointGetPRAngle(j.CID()))
}

/**
 * @brief Get the PR angular position's time derivative
 *
 * @ingroup joints
 */
func (j *Joint) dJointGetPRAngleRate() Real {
	return Real(C.dJointGetPRAngleRate(j.CID()))
}


/**
 * @brief Get the prismatic axis
 * @ingroup joints
 */

func (j *Joint) GetPRAxis1() (result Vector3) {
	C.dJointGetPRAxis1(j.CID(), (*C.dReal)(&result[0]))
	return
}


/**
 * @brief Get the Rotoide axis
 * @ingroup joints
 */

func (j *Joint) GetPRAxis2() (result Vector3) {
	C.dJointGetPRAxis2(j.CID(), (*C.dReal)(&result[0]))
}


/**
 * @brief get joint parameter
 * @ingroup joints
 */

func (j *Joint) GetPRParam() {
	return Real(C.dJointGetPRParam(j.CID()))
}
ODE_API dReal dJointGetPRParam (dJointID, int parameter);

    
    
  /**
   * @brief Get the joint anchor point, in world coordinates.
   * @return the point on body 1. If the joint is perfectly satisfied,
   * this will be the same as the point on body 2.
   * @ingroup joints
   */
  
func (j *Joint) GetPUAnchor() (result Vector3) {
	C.dJointGetPUAnchor(j.CID(), (*C.dReal)(&result[0]))
}
  

  /**
   * @brief Get the PU linear position (i.e. the prismatic's extension)
   *
   * When the axis is set, the current position of the attached bodies is
   * examined and that position will be the zero position.
   *
   * The position is the "oriented" length between the
   * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
   *
   * @ingroup joints
   */
  
func (j *Joint) GetPUPosition() {
	return Real(C.dJointGetPUPosition(j.CID()))
}
  ODE_API dReal dJointGetPUPosition (dJointID);

  /**
   * @brief Get the PR linear position's time derivative
   *
   * @ingroup joints
   */
  
func (j *Joint) GetPUPositionRate() {
	return Real(C.dJointGetPUPositionRate(j.CID()))
}
  ODE_API dReal dJointGetPUPositionRate (dJointID);

  /**
   * @brief Get the first axis of the universal component of the joint
   * @ingroup joints
   */
  
func (j *Joint) GetPUAxis1() (result Vector3) {
	C.dJointGetPUAxis1(j.CID(), (*C.dReal)(&result[0]))
}
  

  /**
   * @brief Get the second axis of the Universal component of the joint
   * @ingroup joints
   */

func (j *Joint) GetPUAxis2() (result Vector3) {
	C.dJointGetPUAxis2(j.CID(), (*C.dReal)(&result[0]))
}

  /**
   * @brief Get the prismatic axis
   * @ingroup joints
   */

func (j *Joint) GetPUAxis3() (result Vector3) {
	C.dJointGetPUAxis3(j.CID(), (*C.dReal)(&result[0]))
}

  /**
   * @brief Get the prismatic axis
   * @ingroup joints
   *
   * @note This function was added for convenience it is the same as
   *       dJointGetPUAxis3
   */

func (j *Joint) GetPUAxisP() (result Vector3) {
	C.dJointGetPUAxisP(j.CID(), (*C.dReal)(&result[0]))
}




  /**
   * @brief Get both angles at the same time.
   * @ingroup joints
   *
   * @param joint   The Prismatic universal joint for which we want to calculate the angles
   * @param angle1  The angle between the body1 and the axis 1
   * @param angle2  The angle between the body2 and the axis 2
   *
   * @note This function combine dJointGetPUAngle1 and dJointGetPUAngle2 together
   *       and try to avoid redundant calculation
   */
  ODE_API void dJointGetPUAngles (dJointID, dReal *angle1, dReal *angle2);

  /**
   * @brief Get angle
   * @ingroup joints
   */
  
func (j *Joint) GetPUAngle1() {
	return Real(C.dJointGetPUAngle1(j.CID()))
}

  /**
   * @brief * @brief Get time derivative of angle1
   *
   * @ingroup joints
   */ 
func (j *Joint) GetPUAngle1Rate() {
	return Real(C.dJointGetPUAngle1Rate(j.CID()))
}


  /**
   * @brief Get angle
   * @ingroup joints
   */ 
func (j *Joint) GetPUAngle2() {
	return Real(C.dJointGetPUAngle2(j.CID()))
}

  /**
   * @brief * @brief Get time derivative of angle2
   *
   * @ingroup joints
   */ 
func (j *Joint) GetPUAngle2Rate(parameter int) {
	return Real(C.dJointGetPUAngle2Rate(j.CID(), C.int(parameter))))
}

  /**
   * @brief get joint parameter
   * @ingroup joints
   */

func (j *Joint) GetPUParam(parameter int) {
	return Real(C.dJointGetPUParam(j.CID(),parameter))
}

/**
   * @brief Get the Piston linear position (i.e. the piston's extension)
   *
   * When the axis is set, the current position of the attached bodies is
   * examined and that position will be the zero position.
   * @ingroup joints
   */ 
func (j *Joint) GetPistonPosition() {
	return Real(C.dJointGetPistonPosition(j.CID()))
}

  /**
   * @brief Get the piston linear position's time derivative.
   * @ingroup joints
   */ 
func (j *Joint) GetPistonPositionRate() {
	return Real(C.dJointGetPistonPositionRate(j.CID()))
}

/**
   * @brief Get the Piston angular position (i.e. the  twist between the 2 bodies)
   *
   * When the axis is set, the current position of the attached bodies is
   * examined and that position will be the zero position.
   * @ingroup joints
   */ 
func (j *Joint) GetPistonAngle() {
	return Real(C.dJointGetPistonAngle(j.CID()))
}

  /**
   * @brief Get the piston angular position's time derivative.
   * @ingroup joints
   */ 
func (j *Joint) GetPistonAngleRate() {
	return Real(C.dJointGetPistonAngleRate(j.CID()))
}


  /**
   * @brief Get the joint anchor
   *
   * This returns the point on body 1. If the joint is perfectly satisfied,
   * this will be the same as the point on body 2 in direction perpendicular
   * to the prismatic axis.
   *
   * @ingroup joints
   */

func (j *Joint) GetPistonAnchor() (result Vector3) {
	C.dJointGetPistonAnchor(j.CID(), (*C.dReal)(&result[0]))
}

  /**
   * @brief Get the joint anchor w.r.t. body 2
   *
   * This returns the point on body 2. You can think of a Piston
   * joint as trying to keep the result of dJointGetPistonAnchor() and
   * dJointGetPistonAnchor2() the same in the direction perpendicular to the
   * pirsmatic axis. If the joint is perfectly satisfied,
   * this function will return the same value as dJointGetPistonAnchor() to
   * within roundoff errors. dJointGetPistonAnchor2() can be used, along with
   * dJointGetPistonAnchor(), to see how far the joint has come apart.
   *
   * @ingroup joints
   */

func (j *Joint) GetPistonAnchor2() (result Vector3) {
	C.dJointGetPistonAnchor2(j.CID(), (*C.dReal)(&result[0]))
}

  /**
   * @brief Get the prismatic axis (This is also the rotoide axis.
   * @ingroup joints
   */

func (j *Joint) GetPistonAxis() (result Vector3) {
	C.dJointGetPistonAxis(j.CID(), (*C.dReal)(&result[0]))
}

  /**
   * @brief get joint parameter
   * @ingroup joints
   */

func (j *Joint) GetPistonParam(parameter int) {
	return Real(C.dJointGetPistonParam(j.CID(),parameter))
}


  /**
 * @brief Get the number of angular axes that will be controlled by the
 * AMotor.
 * @param num can range from 0 (which effectively deactivates the
 * joint) to 3.
 * This is automatically set to 3 in dAMotorEuler mode.
 * @ingroup joints
 */
func (j *Joint) GetMotorNumAxes() int {
	return int(C.dJointGetAMotorNumAxes(j.CID()))
}

/**
 * @brief Get the AMotor axes.
 * @param anum selects the axis to change (0,1 or 2).
 * @param rel Each axis can have one of three ``relative orientation'' modes.
 * \li 0: The axis is anchored to the global frame.
 * \li 1: The axis is anchored to the first body.
 * \li 2: The axis is anchored to the second body.
 * @ingroup joints
 */

func (j *Joint) GetAMotorAxis(anum int) (result Vector3) {
	C.dJointGetAMotorAxis(j.CID(), C.int(anum), (*C.dReal)(&result[0]))
}

/**
 * @brief Get axis
 * @remarks
 * The axis vector is always specified in global coordinates regardless
 * of the setting of rel.
 * There are two GetAMotorAxis functions, one to return the axis and one to
 * return the relative mode.
 *
 * For dAMotorEuler mode:
 * \li	Only axes 0 and 2 need to be set. Axis 1 will be determined
	automatically at each time step.
 * \li	Axes 0 and 2 must be perpendicular to each other.
 * \li	Axis 0 must be anchored to the first body, axis 2 must be anchored
	to the second body.
 * @ingroup joints
 */

func (j *Joint) GetAMotorAxisRel(anum int) int {
	return int(C.dJointGetAMotorAxisRel(j.CID(),C.int(anum)))
}

/**
 * @brief Get the current angle for axis.
 * @remarks
 * In dAMotorUser mode this is simply the value that was set with
 * dJointSetAMotorAngle().
 * In dAMotorEuler mode this is the corresponding euler angle.
 * @ingroup joints
 */

func (j *Joint) GetAMotorAngle(anum int) Real {
	return Real(C.dJointGetAMotorAngle(j.CID(),C.int(anum)))
}

/**
 * @brief Get the current angle rate for axis anum.
 * @remarks
 * In dAMotorUser mode this is always zero, as not enough information is
 * available.
 * In dAMotorEuler mode this is the corresponding euler angle rate.
 * @ingroup joints
 */

func (j *Joint) GetAMotorAngleRate(anum int) Real {
	return Real(C.dJointGetAMotorAngleRate(j.CID(),C.int(anum)))
}

/**
 * @brief get joint parameter
 * @ingroup joints
 */

func (j *Joint) GetAMotorParam(parameter int) {
	return Real(C.dJointGetAMotorParam(j.CID(),parameter))
}

/**
 * @brief Get the angular motor mode.
 * @param mode must be one of the following constants:
 * \li dAMotorUser The AMotor axes and joint angle settings are entirely
 * controlled by the user.  This is the default mode.
 * \li dAMotorEuler Euler angles are automatically computed.
 * The axis a1 is also automatically computed.
 * The AMotor axes must be set correctly when in this mode,
 * as described below.
 * When this mode is initially set the current relative orientations
 * of the bodies will correspond to all euler angles at zero.
 * @ingroup joints
 */

func (j *Joint) GetAMotorMode() int {
	return int(C.dJointGetAMotorMode(j.CID()))
}

/**
 * @brief Get nr of axes.
 * @ingroup joints
 */

func (j *Joint) GetLMotorNumAxes() int {
	return int(C.dJointGetLMotorNumAxes(j.CID()))
}

/**
 * @brief Get axis.
 * @ingroup joints
 */
func (j *joint) GetLMotorAxis(anum int) (result Vector3) {
 	C.dJointGetLMotorAxis(j.CID(), C.int(anum), (*C.dReal)(&result[0]))
 	return
}

/**
 * @brief get joint parameter
 * @ingroup joints
 */

func (j *Joint) GetLMotorParam(parameter int) {
	return Real(C.dJointGetLMotorParam(j.CID(),parameter))
}

/**
 * @brief get joint parameter
 * @ingroup joints
 */

func (j *Joint) GetFixedParam(parameter int) {
	return Real(C.dJointGetFixedParam(j.CID(),parameter))
}


/**
 * @ingroup joints
 */
func (b *Body) ConnectingJoint(b2 *Body) *Joint {
	return JointPtr(C.dConnectingJoint(b.CID(), b2.CID())
}

/**
 * @ingroup joints
 */
func (b *Body) ConnectingJointList(b2 *Body) (list []*Joint) {
	return JointPtr(C.dConnectingJoint(b.CID(), b2.CID(), )
	ODE_API int dConnectingJointList (dBodyID, dBodyID, dJointID*);
}


/**
 * @brief Utility function
 * @return 1 if the two bodies are connected together by
 * a joint, otherwise return 0.
 * @ingroup joints
 */
func (this *Body) IsConnected(that *Body) bool {
	return toBool(C.dAreConnected(this.CID(), that.CID()))
}

/**
 * @brief Utility function
 * @return 1 if the two bodies are connected together by
 * a joint that does not have type @arg{joint_type}, otherwise return 0.
 * @param body1 A body to check.
 * @param body2 A body to check.
 * @param joint_type is a dJointTypeXXX constant.
 * This is useful for deciding whether to add contact joints between two bodies:
 * if they are already connected by non-contact joints then it may not be
 * appropriate to add contacts, however it is okay to add more contact between-
 * bodies that already have contacts.
 * @ingroup joints
 */

func (this *Body) IsConnectedExcluding(that *Body, joint_type JointType) bool {
	return toBool(C.dAreConnected(this.CID(), that.CID()), C.int(joint_type))
}
