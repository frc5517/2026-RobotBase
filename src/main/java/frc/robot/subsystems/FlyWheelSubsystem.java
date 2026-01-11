package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class FlyWheelSubsystem extends SubsystemBase
{
    /// Hardware Constants for the FlyWheel Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int MOTOR_ID = 9; // Spark Max CAN ID
        public static final boolean MOTOR_INVERTED = false; // Inverts control direction.
        public static final MechanismGearing GEAR_RATIO = new MechanismGearing(GearBox.fromReductionStages(3, 4)); // FlyWheel Gear Ratio
        /// Motor Tuning Values
        public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(
                /// Simple PID Tuning Values
                1, 0, 0, // PID - Proportional, Integral, Derivative.
                /// Trapezoid Motion Profiling Constraints.
                new TrapezoidProfile.Constraints(
                        DegreesPerSecond.of(5000).in(RPM), // Max Angular Velocity
                        DegreesPerSecondPerSecond.of(2500).in(RotationsPerSecondPerSecond))); // Max Angular Acceleration
        // Time it takes to reach max speed from 0.
        public static final Time RAMP_RATE = Seconds.of(0.25);
        // Feed Forwards, likely to be left empty.
        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
        // Limits the current, this is a simple indexer. We want the limit low so we don't break things in the case of a jam.
        public static final Current CURRENT_LIMIT = Amp.of(20);
        /// FlyWheel Constants
        public static final Distance FLYWHEEL_DIAMETER = Inches.of(3); // Diameter of the wheel, belt, whatever is spinning on the flywheel.
        public static final Mass FLYWHEEL_MASS = Pounds.of(1); //
        public static final AngularVelocity FLYWHEEL_MAX_SPEED = RPM.of(500); // Max RPM soft limits
    }
    public static class ControlConstants {
        public static final AngularVelocity VELOCITY_TOLERANCE = DegreesPerSecond.of(10); // How accurate the velocity should be.
        public static final AngularVelocity TARGET_VELOCITY = DegreesPerSecond.of(500); // How fast the flywheel should spin.
    }

    /// Our Normal Rev Vendor SparkMax Object.
    private final SparkMax                      indexerMotor    = new SparkMax(HardwareConstants.MOTOR_ID, MotorType.kBrushless);
    /// Our Smart Motor Controller Configuration.
    private final SmartMotorControllerConfig    motorConfig     = new SmartMotorControllerConfig(this)
            .withClosedLoopController(HardwareConstants.PID_CONTROLLER)
            .withGearing(HardwareConstants.GEAR_RATIO)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("Indexer Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(HardwareConstants.CURRENT_LIMIT)
            .withMotorInverted(HardwareConstants.MOTOR_INVERTED)
            .withClosedLoopRampRate(HardwareConstants.RAMP_RATE)
            .withOpenLoopRampRate(HardwareConstants.RAMP_RATE)
            .withFeedforward(HardwareConstants.FEED_FORWARD)
            .withSimFeedforward(HardwareConstants.FEED_FORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP);
    /// Our new Smart Motor Controller
    private final SmartMotorController          motor           = new SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfig);
    /// Our FlyWheel config.
    private final FlyWheelConfig                shooterConfig   = new FlyWheelConfig(motor)
            .withDiameter(HardwareConstants.FLYWHEEL_DIAMETER)
            .withMass(HardwareConstants.FLYWHEEL_MASS)
            .withTelemetry("FlyWheel Mech", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withSoftLimit(HardwareConstants.FLYWHEEL_MAX_SPEED.unaryMinus(), HardwareConstants.FLYWHEEL_MAX_SPEED)
            .withSpeedometerSimulation(HardwareConstants.FLYWHEEL_MAX_SPEED);
    /// Our final FlyWheel Mechanism to use as our smart Indexer.
    private final FlyWheel                      flyWheel        = new FlyWheel(shooterConfig);

    public FlyWheelSubsystem() {

    }

    /**
     * Spins the flywheel up to the predefined target velocity.
     *
     * @return a command that runs the flywheel.
     */
    public Command spinUp() {
        return run(
                // Set target speed
                () -> flyWheel.setSpeed(ControlConstants.TARGET_VELOCITY))
                // When the command finishes, stop the flywheel.
                .finallyDo(() -> flyWheel.set(0.0));
    }

    /**
     * Whether the flywheel is at the target velocity.
     * Indicating when we are ready to fire another fuel.
     *
     * @return whether the flywheel is at the target velocity.
     */
    public Trigger isReady() {
        return flyWheel.isNear(ControlConstants.TARGET_VELOCITY, ControlConstants.VELOCITY_TOLERANCE);
    }

    /**
     * Gets the YAMS Flywheel AKA our Indexer.
     *
     * @return the {@link FlyWheel} Indexer Mechanism.
     */
    public FlyWheel getFlyWheelMechanism() {
        return flyWheel;
    }

    @Override
    public void periodic() {
          flyWheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
          flyWheel.simIterate();
    }
}
