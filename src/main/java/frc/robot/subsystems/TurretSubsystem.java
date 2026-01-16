package frc.robot.subsystems;


import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretSubsystem extends SubsystemBase
{
    /// Hardware Constants for the Turret Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int                     MOTOR_ID            = 21; // Spark Max CAN ID
        public static final boolean                 MOTOR_INVERTED      = false; // Inverts control direction.
        public static final MechanismGearing        GEAR_RATIO          = new MechanismGearing(GearBox.fromReductionStages(3, 4)); // FlyWheel Gear Ratio
        /// Motor Tuning Values
        public static final ProfiledPIDController   PID_CONTROLLER      = new ProfiledPIDController( // Basic Trapezoidal Motion Profiling
                                                                          1, 0, 0, // PID - Proportional, Integral, Derivative.
                                                                          new TrapezoidProfile.Constraints( /// Trapezoid Motion Profiling Constraints.
                                                                          DegreesPerSecond.of(5000).in(RPM), // Max Angular Velocity
                                                                          DegreesPerSecondPerSecond.of(2500).in(RotationsPerSecondPerSecond))); // Max Angular Acceleration
        public static final Time                    RAMP_RATE           = Seconds.of(0.25); // Time it takes to reach max speed from 0.
        public static final SimpleMotorFeedforward  FEED_FORWARD        = new SimpleMotorFeedforward(0, 0, 0); // Feed Forwards.
        public static final Current                 CURRENT_LIMIT       = Amp.of(40); // Current limit, Higher for faster control.
        public static final Angle                   HARD_LIMIT_REVERSE  = Degrees.of(-180); // The hard limit should be a metal physical stop, not the cable chain.
        public static final Angle                   HARD_LIMIT_FORWARD  = Degrees.of(180);
        public static final Angle                   SOFT_LIMIT_REVERSE  = Degrees.of(-175); // A soft limit so we don't constantly hit the hard limit without reason.
        public static final Angle                   SOFT_LIMIT_FORWARD  = Degrees.of(175);
        /// Sim Constants
        public static final Angle                   SIM_STARTING_ANGLE  = Degrees.of(0); // Starting turret angle in sim.
        public static final Distance                MAX_ROBOT_HEIGHT    = Inches.of(22); // Max robot height for visualization. TODO Push to swerve constants
        public static final Distance                MAX_ROBOT_WIDTH     = Inches.of(29); // Max robot width for visualization.
        public static final Translation3d           TURRET_POSITION     = new Translation3d( /// Turret position for visualization.
                                                                          Inches.of(10).in(Meters),  // X-axis left positive relative to the robot center, Same as pose2d.
                                                                          Inches.of(7).in(Meters),   // Y-axis front positive relative to the robot center, Same as pose2d.
                                                                          Inches.of(15).in(Meters)); // Z-axis up relative to the floor.
    }
    /// Control Constants for the Turret Mechanism
    public static class ControlConstants {

    }
    /// The Normal Rev Vendor SparkMax Object.
    private final SparkMax                          indexerMotor        = new SparkMax(HardwareConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    /// The Smart Motor Controller Configuration.
    private final SmartMotorControllerConfig        motorConfig         = new SmartMotorControllerConfig(this)
            .withClosedLoopController(HardwareConstants.PID_CONTROLLER)
            .withGearing(HardwareConstants.GEAR_RATIO)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("Turret Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(HardwareConstants.CURRENT_LIMIT)
            .withMotorInverted(HardwareConstants.MOTOR_INVERTED)
            .withClosedLoopRampRate(HardwareConstants.RAMP_RATE)
            .withOpenLoopRampRate(HardwareConstants.RAMP_RATE)
            .withFeedforward(HardwareConstants.FEED_FORWARD)
            .withSimFeedforward(HardwareConstants.FEED_FORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP);
    /// The new Smart Motor Controller
    private final SmartMotorController              motor               = new SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfig);
    /// The Turret Position Config
    private final MechanismPositionConfig           robotToMechanism    = new MechanismPositionConfig()
            .withMaxRobotHeight(HardwareConstants.MAX_ROBOT_HEIGHT)
            .withMaxRobotLength(HardwareConstants.MAX_ROBOT_WIDTH)
            .withRelativePosition(HardwareConstants.TURRET_POSITION);
    /// The Pivot Config for the Turret
    private final PivotConfig                       m_config            = new PivotConfig(motor)
            .withHardLimit(HardwareConstants.HARD_LIMIT_REVERSE, HardwareConstants.HARD_LIMIT_FORWARD)
            .withSoftLimits(HardwareConstants.SOFT_LIMIT_REVERSE, HardwareConstants.SOFT_LIMIT_FORWARD)
            .withTelemetry("Turret", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStartingPosition(HardwareConstants.SIM_STARTING_ANGLE)
            .withMechanismPositionConfig(robotToMechanism)
            .withMOI(KilogramSquareMeters.of(0.001));
    /// The final Pivot Mechanism to use as the turret.
    private final Pivot                             turret              = new Pivot(m_config);

    public TurretSubsystem()
    {
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the turret mechanism's telemetry data to the network tables.
        turret.updateTelemetry();
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        turret.simIterate();
    }
}