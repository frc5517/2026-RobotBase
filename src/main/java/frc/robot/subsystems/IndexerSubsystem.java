package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class IndexerSubsystem extends SubsystemBase
{
    /// Hardware Constants for the Indexer Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int MOTOR_ID = 9; // Spark Max CAN ID
        public static final boolean MOTOR_INVERTED = false; // Inverts control direction.
        public static final MechanismGearing GEAR_RATIO = new MechanismGearing(GearBox.fromReductionStages(3, 4)); // Indexer Gear Ratio
        /// Motor Tuning Values
        public static final ExponentialProfilePIDController PID_CONTROLLER = new ExponentialProfilePIDController(
                /// Simple PID Tuning Values
                1, 0, 0, // PID - Proportional, Integral, Derivative.
                /// Exponential Motion Profiling Constraints.
                ExponentialProfilePIDController.createConstraints(
                        Volts.of(10), // Max Control Voltage
                        DegreesPerSecond.of(180), // Max Angular Velocity
                        DegreesPerSecondPerSecond.of(360))); // Max Angular Acceleration
        // Time it takes to reach max speed from 0.
        public static final Time RAMP_RATE = Seconds.of(0.25);
        // Feed Forwards, likely to be left empty.
        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
        // Limits the current, this is a simple indexer. We want the limit low so we don't break things in the case of a jam.
        public static final Current CURRENT_LIMIT = Amp.of(20);
        /// Indexer Constants
        public static final Distance INDEXER_DIAMETER = Inches.of(3); // Diameter of the wheel, belt, whatever is spinning on the indexer.
        public static final AngularVelocity INDEXER_MAX_SPEED = RPM.of(120); // Max RPM soft limits
    }
    public static class ControlConstants {
        public static final AngularVelocity VELOCITY_TOLERANCE = DegreesPerSecond.of(1); // How accurate the velocity should be.
        public static final AngularVelocity TARGET_VELOCITY = DegreesPerSecond.of(60); // How fast the flywheel should spin.
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
    /// Our FlyWheel config for our Indexer.
    private final FlyWheelConfig                shooterConfig   = new FlyWheelConfig(motor)
            .withDiameter(HardwareConstants.INDEXER_DIAMETER)
            .withMass(Pounds.of(1)) // Indexer Doesn't need to specify weight.
            .withTelemetry("Indexer Mech", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withSoftLimit(HardwareConstants.INDEXER_MAX_SPEED.unaryMinus(), HardwareConstants.INDEXER_MAX_SPEED)
            .withSpeedometerSimulation(HardwareConstants.INDEXER_MAX_SPEED);
    /// Our final FlyWheel Mechanism to use as our smart Indexer.
    private final FlyWheel                      shooter         = new FlyWheel(shooterConfig);

    public IndexerSubsystem() {

    }

    /**
     * Indexes a single fuel piece using sensors.
     *
     * @return a command to index fuel.
     */
    public Command indexOne() {
        return Commands.none();
    }

    /**
     * Gets the YAMS Flywheel AKA our Indexer.
     *
     * @return the {@link FlyWheel} Indexer Mechanism.
     */
    public FlyWheel getIndexerMechanism() {
        return shooter;
    }

    @Override
    public void periodic() {
          shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
          shooter.simIterate();
    }
}
