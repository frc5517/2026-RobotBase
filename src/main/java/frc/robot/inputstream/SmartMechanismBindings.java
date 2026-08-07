//package frc.robot.inputstream;
//
//import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.AngularVelocity;
//import edu.wpi.first.units.measure.Distance;
//import edu.wpi.first.units.measure.LinearVelocity;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import lombok.Getter;
//import yams.mechanisms.SmartMechanism;
//
//import java.util.Objects;
//import java.util.function.Supplier;
//
//public class SmartMechanismBindings {
//    /**
//     * The {@link SmartMechanism} to link control to.
//     */
//    @Getter
//    protected final SmartMechanism mechanism;
//    /**
//     * The {@link Subsystem} to run commands from.
//     */
//    protected final Subsystem subsystem;
//    /**
//     * Checks if this subsystem is present, if not, don't bind anything.
//     */
//    @Getter
//    protected final boolean isPresent;
//    /**
//     * The {@link InputStream} to attach to.
//     */
//    protected final InputStream inputStream;
//    /**
//     * A {@link Trigger} to determine when these controls are active.
//     */
//    protected final Trigger isMode;
//
//    /**
//     * The Generic SmartMechanism controls.
//     * This class holds controls that can be used on ANY brushed mechanism.
//     *
//     * @param mechanism the {@link SmartMechanism} to control.
//     * @param subsystem the {@link Subsystem} to run commands from. Should be the same as the one for the YAMS Mechanism.
//     * @param stream the {@link InputStream} to attach to.
//     * @param isPresent whether this subsystem is present. [subystems.mech != null]
//     */
//    protected SmartMechanismBindings(SmartMechanism mechanism, Subsystem subsystem, InputStream stream, boolean isPresent) {
//        this.mechanism = mechanism;
//        this.subsystem = subsystem;
//        this.inputStream = Objects.requireNonNull(stream);
//        this.isMode = inputStream.getIsMode();
//        this.isPresent = isPresent;
//    }
//
//    // DutyCycle
//    /**
//     * Runs the motor at given speed, without stopping.
//     *
//     * @param runDutyCycle then button to map.
//     * @param speed duty cycle speed to run. [-1, 1]
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunDutyCycleWithoutStopping(Trigger runDutyCycle, double speed) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runDutyCycle) // When active
//                .whileTrue(subsystem.run(() -> mechanism.set(speed))); // Run to the given setpoint.
//        return this;
//    }
//
//    /**
//     * Runs the motor at given speed, stopping when finished.
//     *
//     * @param runDutyCycle then button to map.
//     * @param speed duty cycle speed to run. [-1, 1]
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunDutyCycleUntilReleased(Trigger runDutyCycle, double speed) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runDutyCycle) // When active
//                .whileTrue(subsystem.run(() -> mechanism.set(speed))) // Run to the given setpoint.
//                .onFalse(subsystem.runOnce(() -> mechanism.set(0))); // Stopping when released.
//        return this;
//    }
//
//    // Angle
//    /**
//     * Runs the Mechanism to the given angle, without stopping.
//     *
//     * @param runAngle then button to map.
//     * @param angle the {@link Angle} goal.
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunAngleWithoutStopping(Trigger runAngle, Angle angle) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runAngle) // When active
//                .whileTrue(subsystem.run(() -> mechanism.setMechanismPositionSetpoint(angle))); // Run to the given setpoint.
//        return this;
//    }
//
//    /**
//     * Runs the Mechanism to the given angle, stopping when released.
//     *
//     * @param runAngle then button to map.
//     * @param angle the {@link Angle} goal.
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunAngleUntilReleased(Trigger runAngle, Angle angle) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runAngle) // When active
//                .whileTrue(subsystem.run(() -> mechanism.setMechanismPositionSetpoint(angle))) // Run to the given setpoint.
//                .onFalse(subsystem.runOnce(() -> mechanism.set(0))); // Stopping when released.
//        return this;
//    }
//
//    // Distance
//    /**
//     * Runs the Mechanism to the given distance.
//     *
//     * @param runDistance then button to map.
//     * @param distance the {@link Distance} goal.
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunDistanceWithoutStopping(Trigger runDistance, Distance distance) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runDistance) // When active
//                .whileTrue(subsystem.run(() -> mechanism.setMeasurementPositionSetpoint(distance))); // Run to the given setpoint.
//        return this;
//    }
//
//    /**
//     * Runs the Mechanism to the given distance.
//     *
//     * @param runDistance then button to map.
//     * @param distance the {@link Distance} goal.
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunDistanceUntilReleased(Trigger runDistance, Distance distance) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runDistance) // When active
//                .whileTrue(subsystem.run(() -> mechanism.setMeasurementPositionSetpoint(distance))) // Run to the given setpoint.
//                .onFalse(subsystem.runOnce(() -> mechanism.set(0))); // Stopping when released.
//        return this;
//    }
//
//    // Angular Velocity
//    /**
//     * Runs the Mechanism at the given speed, without stopping.
//     *
//     * @param runVelocity then button to map.
//     * @param speed duty cycle speed to run. [-1, 1]
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunAngularVelocityWithoutStopping(Trigger runVelocity, AngularVelocity speed) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runVelocity) // When active
//                .whileTrue(subsystem.run(() -> mechanism.setMechanismVelocitySetpoint(speed))) // Run to the given setpoint.
//                .onFalse(subsystem.runOnce(() -> mechanism.set(0))); // Stopping when released.
//        return this;
//    }
//
//    /**
//     * Runs the Mechanism at the given speed, stopping when finished.
//     *
//     * @param runVelocity then button to map.
//     * @param speed duty cycle speed to run. [-1, 1]
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunAngularVelocityUntilReleased(Trigger runVelocity, AngularVelocity speed) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runVelocity) // When active
//                .whileTrue(subsystem.run(() -> mechanism.setMechanismVelocitySetpoint(speed))) // Run to the given setpoint.
//                .onFalse(subsystem.runOnce(() -> mechanism.set(0))); // Stopping when released.
//        return this;
//    }
//
//    // Linear Velocity
//    /**
//     * Runs the Mechanism at the given speed, without stopping.
//     *
//     * @param runVelocity then button to map.
//     * @param speed duty cycle speed to run. [-1, 1]
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunLinearVelocityWithoutStopping(Trigger runVelocity, LinearVelocity speed) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runVelocity) // When active
//                .whileTrue(subsystem.run(() -> mechanism.setMeasurementVelocitySetpoint(speed))); // Run to the given setpoint.
//        return this;
//    }
//
//    /**
//     * Runs the Mechanism at the given speed, stopping when finished.
//     *
//     * @param runVelocity then button to map.
//     * @param speed duty cycle speed to run. [-1, 1]
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withRunLinearVelocityUntilReleased(Trigger runVelocity, LinearVelocity speed) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(runVelocity) // When active
//                .whileTrue(subsystem.run(() -> mechanism.setMeasurementVelocitySetpoint(speed))) // Run to the given setpoint.
//                .onFalse(subsystem.runOnce(() -> mechanism.set(0))); // Stopping when released.
//        return this;
//    }
//
//    // Misc
//    /**
//     * Stops the Mechanism control.
//     *
//     * @param stop then button to map.
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withStopMechanism(Trigger stop) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(stop) // When active
//                .onTrue(subsystem.runOnce(() -> mechanism.set(0))); // Stop the mechanism.
//        return this;
//    }
//
//    /**
//     * Sets a default command for this subsystem.
//     *
//     * @param defaultCommand the default command to set.
//     * @return this, for chaining.
//     */
//    public SmartMechanismBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//        if (!isPresent) {return this;} // If this mechanism isn't present, don't bind anything.
//        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystem.setDefaultCommand(defaultCommand.get())));
//        return this;
//    }
//
//    /**
//     * Leaves SmartMechanismBindings going back to the InputStream.
//     *
//     * @return this InputStream.
//     */
//    public InputStream back() {
//        return inputStream;
//    }
//}