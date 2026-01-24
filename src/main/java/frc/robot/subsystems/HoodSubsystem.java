package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import lombok.Getter;
import lombok.Setter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.HoodSubsystem.HardwareConstants.*;
import static frc.robot.subsystems.HoodSubsystem.ControlConstants.*;
import static frc.robot.subsystems.TurretSubsystem.HardwareConstants.TURRET_POSITION;

public class HoodSubsystem extends SubsystemBase {
    /// The Hardware Constants for the Hood Mechanism.
    public static final class HardwareConstants {
        /// Motor Constants
        public static final int                                 MOTOR_ID            = 24; // Spark Max CAN ID
        public static final boolean                             MOTOR_INVERTED      = false; // Inverts control direction.
        public static final MechanismGearing                    GEAR_RATIO          = new MechanismGearing(GearBox.fromReductionStages(3, 4)); // FlyWheel Gear Ratio
        /// Motor Tuning Values
        public static final ExponentialProfilePIDController     PID_CONTROLLER      = new ExponentialProfilePIDController( // Exponential Motion Profiling
                                                                                      20, 0, 0.01, // PID - Proportional, Integral, Derivative.
                                                                                      ExponentialProfilePIDController.createConstraints( /// Exponential Motion Profiling Constraints.
                                                                                      Volts.of(12), // Max Control Voltage
                                                                                      DegreesPerSecond.of(180), // Max Angular Velocity
                                                                                      DegreesPerSecondPerSecond.of(360))); // Max Angular Acceleration
        public static final Time                                RAMP_RATE           = Seconds.of(0.15); // Time it takes to reach max speed from 0.
        public static final ArmFeedforward                      FEED_FORWARD        = new ArmFeedforward(0, 0, 0); // Feed Forwards.
        public static final Current                             CURRENT_LIMIT       = Amp.of(40); // Current limit, Higher for faster control.
        /// Hood Constants
        public static final Mass                                HOOD_MASS           = Pounds.of(3); // Weight of the hood mechanism.
        public static final Distance                            HOOD_LENGTH         = Inches.of(8); // Hood Length, used in calculations and to visualize in sim.
        public static final Angle                               HORIZONTAL_OFFSET   = Degrees.of(0); // Offset required making angle 0 horizontal and parallel to the ground.
        public static final Angle                               HARD_LIMIT_REVERSE  = Degrees.of(30); // The hard limit should be a metal physical stop.
        public static final Angle                               HARD_LIMIT_FORWARD  = Degrees.of(135);
        public static final Angle                               SOFT_LIMIT_REVERSE  = Degrees.of(35); // A soft limit so we don't constantly hit the hard limit without reason.
        public static final Angle                               SOFT_LIMIT_FORWARD  = Degrees.of(130);
        /// IMPORTANT, this helps calculate the initial fuel velocity; it is the friction affecting on the ball from the hood.
        public static final double                              HOOD_COF_FACTOR     = 0.8;
        /// Sim Constants
        public static final Angle                               SIM_STARTING_ANGLE  = Degrees.of(0); // Starting Hood angle in sim.
        public static final Distance                            MAX_ROBOT_HEIGHT    = Inches.of(22); // Max robot height for visualization. TODO Push to swerve constants
        public static final Distance                            MAX_ROBOT_WIDTH     = Inches.of(29); // Max robot width for visualization.
        public static final Translation3d                       HOOD_POSITION       = TURRET_POSITION.plus(
                                                                                      new Translation3d( /// Hood position for visualization, relative to the turret position.
                                                                                      Inches.of(0).in(Meters),  // X-axis left positive relative to the robot center, Same as pose2d.
                                                                                      Inches.of(0).in(Meters),   // Y-axis front positive relative to the robot center, Same as pose2d.
                                                                                      Inches.of(0).in(Meters))); // Z-axis up relative to the floor.
    }
    /// The Control Constants for the Hood Mechanism.
    public static final class ControlConstants {
        public static final double                              HOOD_SPEED          = 0.3; // Predefined duty cycle speed.
        public static final Angle                               ANGLE_TOLERANCE     = Degrees.of(.1); // How accurate the angle should be.
        public static final Angle                               FUEL_ENTRY_ANGLE    = Degrees.of(40);
    }
    public static final class MathConstants {
        public static final LinearVelocity                      kExitVelocity       = MetersPerSecond.of(20);
    }
    /// Finally, we can initialize our mechanism.
    private final SparkMax                                      hoodMotor        = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig                    motorConfig         = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
            .withClosedLoopController(PID_CONTROLLER)
            .withGearing(GEAR_RATIO)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("Hood Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(CURRENT_LIMIT)
            .withMotorInverted(MOTOR_INVERTED)
            .withClosedLoopRampRate(RAMP_RATE)
            .withOpenLoopRampRate(RAMP_RATE)
            .withFeedforward(FEED_FORWARD)
            .withSimFeedforward(FEED_FORWARD)
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private final SmartMotorController                          motor               = new SparkWrapper(hoodMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
    private final MechanismPositionConfig                       robotToMechanism    = new MechanismPositionConfig() /// The Turret Position Config
            .withMaxRobotHeight(MAX_ROBOT_HEIGHT)
            .withMaxRobotLength(MAX_ROBOT_WIDTH)
            .withRelativePosition(HOOD_POSITION);
    private final ArmConfig m_config = new ArmConfig(motor) /// The Arm Config for the Hood Mechanism.
            .withLength(HOOD_LENGTH)
            .withHardLimit(HARD_LIMIT_REVERSE, HARD_LIMIT_FORWARD)
            .withSoftLimits(SOFT_LIMIT_REVERSE, SOFT_LIMIT_FORWARD)
            .withTelemetry("Hood", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withMass(HOOD_MASS)
            .withStartingPosition(SIM_STARTING_ANGLE)
            //.withHorizontalZero(HardwareConstants.HORIZONTAL_OFFSET)
            .withMechanismPositionConfig(robotToMechanism);
    @Getter
    private final Arm                               hood                = new Arm(m_config); /// The final Arm Mechanism to use as the hood.
    /// Reports the current Hood state. To be used later in code and telemetry.
    public static class HoodState {
        @Setter
        public static double    ManualSpeed     = HOOD_SPEED;
        @Setter
        public static Angle     AngleTolerance  = ANGLE_TOLERANCE;
        @Setter
        public static Angle     CurrentAngle    = Degrees.of(0);
        @Setter
        public static Angle     DesiredAngle    = Degrees.of(0);
        @Setter
        public static Trigger   AtDesiredAngle  = new Trigger(() -> false);
    }

    public HoodSubsystem() {
        HoodState.setAtDesiredAngle(atDesiredAngle());
        this.setDefaultCommand(hood.setAngle(Degrees.of(70)));
    }

    /**
     * Resets the setters to default values.
     */
    public void resetSetters() {
        HoodState.setManualSpeed(HOOD_SPEED);
    }

    /**
     * Whether the hood is at the desired angle.
     *
     * @return whether the hood is at the desired angle.
     */
    public Trigger atDesiredAngle() {
        return hood.isNear(HoodState.DesiredAngle, HoodState.AngleTolerance);
    }

    /**
     * Raise the hood manually at a predefined speed.
     *
     * @return a command to raise the hood manually.
     */
    public Command upManual() {
        return run(() -> hood.set(HoodState.ManualSpeed))
                .finallyDo(() -> hood.set(0.0));
    }

    /**
     * Lower the hood manually at a predefined speed.
     *
     * @return a command to lower the hood manually.
     */
    public Command downManual() {
        return run(() -> hood.set(HoodState.ManualSpeed))
                .finallyDo(() -> hood.set(0.0));
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the hood mechanism's telemetry data to the network tables.
        hood.updateTelemetry();
        HoodState.setCurrentAngle(hood.getAngle());
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        hood.simIterate();
    }
}