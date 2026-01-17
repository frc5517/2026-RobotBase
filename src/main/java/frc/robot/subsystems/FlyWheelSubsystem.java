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
import lombok.Getter;
import lombok.Setter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
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
        public static final int                     MOTOR_ID            = 25; // Spark Max CAN ID
        public static final boolean                 MOTOR_INVERTED      = false; // Inverts control direction.
        public static final MechanismGearing        GEAR_RATIO          = new MechanismGearing(GearBox.fromReductionStages(3, 4)); // FlyWheel Gear Ratio
        /// Motor Tuning Values
        public static final ProfiledPIDController   PID_CONTROLLER      = new ProfiledPIDController(
                                                                      1, 0, 0, // PID - Proportional, Integral, Derivative.
                                                                          new TrapezoidProfile.Constraints( /// Trapezoid Motion Profiling Constraints.
                                                                          DegreesPerSecond.of(5000).in(RPM), // Max Angular Velocity
                                                                          DegreesPerSecondPerSecond.of(2500).in(RotationsPerSecondPerSecond))); // Max Angular Acceleration
        public static final Time                    RAMP_RATE           = Seconds.of(0.25); // Time it takes to reach max speed from 0.
        public static final SimpleMotorFeedforward  FEED_FORWARD        = new SimpleMotorFeedforward(0, 0, 0); // Feed Forwards.
        public static final Current                 CURRENT_LIMIT       = Amp.of(40); // Current limit, Higher for faster control.
        /// FlyWheel Constants
        public static final Distance                FLYWHEEL_DIAMETER   = Inches.of(3); // Diameter of the wheel, belt, whatever is spinning on the flywheel.
        public static final Mass                    FLYWHEEL_MASS       = Pounds.of(1); // Weight of the flywheel, just what gets spun.
        public static final AngularVelocity         FLYWHEEL_MAX_SPEED  = RPM.of(500); // Max RPM soft limits
    }
    /// Control Constants for the FlyWheel Mechanism
    public static class ControlConstants {
        public static final AngularVelocity         VELOCITY_TOLERANCE  = DegreesPerSecond.of(10); // How accurate the velocity should be.
        public static final AngularVelocity         TARGET_VELOCITY     = DegreesPerSecond.of(500); // How fast the flywheel should spin.
    }
    /// Initialize the FlyWheel
    private final SparkMax                          indexerMotor        = new SparkMax(HardwareConstants.MOTOR_ID, MotorType.kBrushless); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig        motorConfig         = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
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
    private final SmartMotorController              motor               = new SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
    private final FlyWheelConfig                    flyWheelConfig      = new FlyWheelConfig(motor) /// The FlyWheel config.
            .withDiameter(HardwareConstants.FLYWHEEL_DIAMETER)
            .withMass(HardwareConstants.FLYWHEEL_MASS)
            .withTelemetry("FlyWheel", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withSoftLimit(HardwareConstants.FLYWHEEL_MAX_SPEED.unaryMinus(), HardwareConstants.FLYWHEEL_MAX_SPEED)
            .withSpeedometerSimulation(HardwareConstants.FLYWHEEL_MAX_SPEED);
    @Getter
    private final FlyWheel                          flyWheel            = new FlyWheel(flyWheelConfig); /// The final FlyWheel Mechanism.
    /// Reports the current flywheel state. To be used later in code and telemetry.
    public static class FlyWheelState {
        @Setter
        public static AngularVelocity              VelocityTolerance  = ControlConstants.VELOCITY_TOLERANCE;
        @Setter
        public static AngularVelocity              Velocity           = DegreesPerSecond.of(0);
        @Setter
        public static AngularVelocity              TargetVelocity     = ControlConstants.TARGET_VELOCITY;
        @Setter
        public static Trigger                      IsReady            = new Trigger(() -> false);
    }

    public FlyWheelSubsystem() {
        FlyWheelState.setIsReady(isReady());
    }

    /**
     * Resets the setters to default values.
     */
    public void resetSetters() {
        FlyWheelState.setTargetVelocity(ControlConstants.TARGET_VELOCITY);
    }

    /**
     * Spins the flywheel up to the predefined target velocity.
     *
     * @return a command that runs the flywheel.
     */
    public Command spinUp() {
        return run(
                // Set target speed
                () -> flyWheel.setSpeed(FlyWheelState.TargetVelocity))
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
        return flyWheel.isNear(FlyWheelState.TargetVelocity, FlyWheelState.VelocityTolerance);
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the flywheel mechanism's telemetry data to the network tables.
        flyWheel.updateTelemetry();
        FlyWheelState.setVelocity(flyWheel.getSpeed());
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        flyWheel.simIterate();
    }
}
