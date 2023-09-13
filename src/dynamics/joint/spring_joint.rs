use na::Vector2;

use crate::dynamics::{GenericJoint, GenericJointBuilder, JointAxesMask};
use crate::dynamics::{JointAxis, MotorModel};
use crate::math::{Point, Real, UnitVector, Isometry, Vector};

use super::{JointLimits, JointMotor};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A spring joint!
pub struct SpringJoint {
    /// The underlying joint data.
    pub data: GenericJoint,
}

/*impl Default for SpringJoint {
    fn default() -> Self {
        SpringJoint::new(axis: UnitVector<Real>)
    }
}*/

impl SpringJoint {
    /// Creates a new rope joint limiting the max distance between to bodies
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::FREE_FIXED_AXES)
            .coupled_axes(JointAxesMask::LIN_AXES)
            .build();
        Self { data }
    }

    /// The underlying generic joint.
    pub fn data(&self) -> &GenericJoint {
        &self.data
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.data.contacts_enabled
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.data.set_contacts_enabled(enabled);
        self
    }

    /// The joint’s frame, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_frame1(&self) -> &Isometry<Real> {
        &self.data.local_frame1
    }

    /// Sets the joint’s frame, expressed in the first rigid-body’s local-space.
    pub fn set_local_frame1(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.data.set_local_frame1(local_frame);
        self
    }

    /// The joint’s frame, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_frame2(&self) -> &Isometry<Real> {
        &self.data.local_frame2
    }

    /// Sets joint’s frame, expressed in the second rigid-body’s local-space.
    pub fn set_local_frame2(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.data.set_local_frame2(local_frame);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(&self) -> Point<Real> {
        self.data.local_anchor1()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1(&mut self, anchor1: Point<Real>) -> &mut Self {
        self.data.set_local_anchor1(anchor1);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(&self) -> Point<Real> {
        self.data.local_anchor2()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2(&mut self, anchor2: Point<Real>) -> &mut Self {
        self.data.set_local_anchor2(anchor2);
        self
    }
    /// The principal axis of the joint, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_axis1(&self) -> UnitVector<Real> {
        self.data.local_axis1()
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    pub fn set_local_axis1(&mut self, axis1: UnitVector<Real>) -> &mut Self {
        self.data.set_local_axis1(axis1);
        self
    }

    /// The principal axis of the joint, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_axis2(&self) -> UnitVector<Real> {
        self.data.local_axis2()
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    pub fn set_local_axis2(&mut self, axis2: UnitVector<Real>) -> &mut Self {
        self.data.set_local_axis2(axis2);
        self
    }
    /// The motor affecting the joint’s translational degree of freedom.
    #[must_use]
    pub fn motor(&self, axis: JointAxis) -> Option<&JointMotor> {
        self.data.motor(axis)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::X, model);
        self.data.set_motor_model(JointAxis::Y, model);
        #[cfg(feature = "dim3")]
        self.data.set_motor_model(JointAxis::Z, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(&mut self, target_vel: Real, factor: Real) -> &mut Self {
        self.data
            .set_motor_velocity(JointAxis::X, target_vel, factor);
        self.data
            .set_motor_velocity(JointAxis::Y, target_vel, factor);
        #[cfg(feature = "dim3")]
        self.data
            .set_motor_velocity(JointAxis::Z, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn set_motor_position(
        &mut self,
        target_pos_vector: Vector<Real>,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor_position(JointAxis::X, target_pos_vector, stiffness, damping);
        self.data
            .set_motor_position(JointAxis::Y, target_pos_vector, stiffness, damping);
        #[cfg(feature = "dim3")]
        self.data
            .set_motor_position(JointAxis::Z, target_pos_vector, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn set_motor(
        &mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor(JointAxis::X, target_pos, target_vel, stiffness, damping);
        self.data
            .set_motor(JointAxis::Y, target_pos, target_vel, stiffness, damping);
        #[cfg(feature = "dim3")]
        self.data
            .set_motor(JointAxis::Z, target_pos, target_vel, stiffness, damping);
        self
    }
    /*/// Configure both the target angle and target velocity of the motor.
    pub fn set_motor_advanced(
        &mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor(JointAxis::X, target_pos, target_vel, stiffness, damping);
        self.data
            .set_motor(JointAxis::Y, target_pos, target_vel, stiffness, damping);
        #[cfg(feature = "dim3")]
        self.data
            .set_motor(JointAxis::Y, target_pos, target_vel, stiffness, damping);
        self
    }*/

    /// Sets the maximum force the motor can deliver.
    pub fn set_motor_max_force(&mut self, max_force: Real) -> &mut Self {
        self.data.set_motor_max_force(JointAxis::X, max_force);
        self.data.set_motor_max_force(JointAxis::Y, max_force);
        #[cfg(feature = "dim3")]
        self.data.set_motor_max_force(JointAxis::Z, max_force);
        self
    }

    /// The limit maximum distance attached bodies can translate.
    #[must_use]
    pub fn limits(&self, axis: JointAxis) -> Option<&JointLimits<Real>> {
        self.data.limits(axis)
    }

    /// Sets the `[min,max]` limit distances attached bodies can translate.
    pub fn set_limits(&mut self, limits: [Real; 2]) -> &mut Self {
        self.data.set_limits(JointAxis::X, limits);
        self.data.set_limits(JointAxis::Y, limits);
        #[cfg(feature = "dim3")]
        self.data.set_limits(JointAxis::Z, limits);
        self
    }

}

impl Into<GenericJoint> for SpringJoint {
    fn into(self) -> GenericJoint {
        self.data
    }
}

/// Create fixed joints using the builder pattern.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpringJointBuilder(pub SpringJoint);

impl SpringJointBuilder {
    /// Creates a new builder for fixed joints.
    pub fn new() -> Self {
        Self(SpringJoint::new())
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    #[must_use]
    pub fn contacts_enabled(mut self, enabled: bool) -> Self {
        self.0.set_contacts_enabled(enabled);
        self
    }

    /// Sets the joint’s frame, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_frame1(mut self, local_frame: Isometry<Real>) -> Self {
        self.0.set_local_frame1(local_frame);
        self
    }

    /// Sets joint’s frame, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_frame2(mut self, local_frame: Isometry<Real>) -> Self {
        self.0.set_local_frame2(local_frame);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }
    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_axis1(mut self, axis1: UnitVector<Real>) -> Self {
        self.0.set_local_axis1(axis1);
        self
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_axis2(mut self, axis2: UnitVector<Real>) -> Self {
        self.0.set_local_axis2(axis2);
        self
    }
    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    #[must_use]
    pub fn motor_model(mut self, model: MotorModel) -> Self {
        self.0.set_motor_model(model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    #[must_use]
    pub fn motor_velocity(mut self, target_vel: Real, factor: Real) -> Self {
        self.0.set_motor_velocity(target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn motor_position(mut self, target_pos_vector: Vector<Real>, stiffness: Real, damping: Real) -> Self {
        self.0.set_motor_position(target_pos_vector, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn set_motor(
        mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0.set_motor(target_pos, target_vel, stiffness, damping);
        self
    }

    /*pub fn set_motor_advanced(
        mut self,
        target1: Isometry<Real>,
        target2: Isometry<Real>,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0.set_motor_advanced(target1, target2, target_vel, stiffness, damping);
        self
    }*/

    /// Sets the maximum force the motor can deliver.
    #[must_use]
    pub fn motor_max_force(mut self, max_force: Real) -> Self {
        self.0.set_motor_max_force(max_force);
        self
    }

    /// Sets the `[min,max]` limit distances attached bodies can translate.
    #[must_use]
    pub fn limits(mut self, limits: [Real; 2]) -> Self {
        self.0.set_limits(limits);
        self
    }

    /// Build the fixed joint.
    #[must_use]
    pub fn build(self) -> SpringJoint {
        self.0
    }
}

impl Into<GenericJoint> for SpringJointBuilder {
    fn into(self) -> GenericJoint {
        self.0.into()
    }
}
