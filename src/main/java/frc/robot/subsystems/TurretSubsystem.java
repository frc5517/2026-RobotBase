package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.InputBuilder;
import frc.robot.Telemetry;
import lombok.Getter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.local.SparkWrapper;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.TurretSubsystem.ControlConstants.ANGLE_TOLERANCE;
import static frc.robot.subsystems.TurretSubsystem.HardwareConstants.*;
import static yams.motorcontrollers.SmartMotorControllerConfig.MotorMode.BRAKE;

public class TurretSubsystem extends SubsystemBase {
    /// Hardware Constants for the Turret Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int MOTOR_ID = 17; // Spark Max CAN ID
        public static final boolean MOTOR_INVERTED = true; // Inverts control direction.
        public static final MechanismGearing GEAR_RATIO = new MechanismGearing(GearBox.fromReductionStages(5, 10)); // Turret Gear Ratio
        /// Motor Tuning Values
        public static final PIDController PID_CONTROLLER = new PIDController( // Exponential Motion Profiling
                60, 0, 0.01); // PID - Proportional, Integral, Derivative.

        /// Exponential Motion Profiling Constraints.
        public static final class Profiling {
            public static final Voltage MAX_CONTROL_VOLTAGE = Volts.of(12); // Max Control Voltage
            public static final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(360); // Max Angular Velocity
            public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(720); // Max Angular Acceleration
        }

        public static final Time RAMP_RATE = Seconds.of(0.25); // Time it takes to reach max speed from 0.
        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0); // Feed Forwards.
        public static final Current CURRENT_LIMIT = Amp.of(40); // Current limit, Higher for faster control.
        public static final Angle HARD_LIMIT_REVERSE = Degrees.of(-80); // The hard limit should be a metal physical stop, not the cable chain.
        public static final Angle HARD_LIMIT_FORWARD = Degrees.of(90);
        public static final Angle SOFT_LIMIT_REVERSE = Degrees.of(-80); // A soft limit so we don't constantly hit the hard limit without reason.
        public static final Angle SOFT_LIMIT_FORWARD = Degrees.of(90);
        /// Sim Constants
        public static final Angle SIM_STARTING_ANGLE = Degrees.of(-80); // Starting turret angle in sim.
        public static final Distance MAX_ROBOT_HEIGHT = Inches.of(30); // Max robot height for visualization. TODO Push to swerve constants
        public static final Distance MAX_ROBOT_WIDTH = Inches.of(29); // Max robot width for visualization.
        public static final Translation3d TURRET_POSITION = new Translation3d( /// Turret position for visualization.
                Inches.of(-6).in(Meters),  // X-axis left positive relative to the robot center, Same as pose2d.
                Inches.of(-.5).in(Meters),   // Y-axis front positive relative to the robot center, Same as pose2d.
                Inches.of(19.75).in(Meters)); // Z-axis up relative to the floor.
    }

    /// Control Constants for the Turret Mechanism
    public static class ControlConstants {
        public static final Angle PHYSICAL_STARTING_ANGLE = Degrees.of(-80);
        public static final Angle IDLE_ANGLE = SIM_STARTING_ANGLE;

        public static final Angle ANGLE_TOLERANCE = Degrees.of(8);

    }

    /// The Normal Rev Vendor SparkMax Object.
    private final SparkMax indexerMotor = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    /// The Smart Motor Controller Configuration.
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withExponentialProfile(Profiling.MAX_CONTROL_VOLTAGE, Profiling.MAX_ANGULAR_VELOCITY, Profiling.MAX_ANGULAR_ACCELERATION)
            .withClosedLoopController(PID_CONTROLLER)
            .withGearing(GEAR_RATIO)
            .withIdleMode(BRAKE)
            .withTelemetry("Turret Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(CURRENT_LIMIT)
            .withMotorInverted(MOTOR_INVERTED)
            .withClosedLoopRampRate(RAMP_RATE)
            .withOpenLoopRampRate(RAMP_RATE)
            .withFeedforward(FEED_FORWARD)
            .withSimFeedforward(FEED_FORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withContinuousWrapping(Degrees.of(-180), Degrees.of(180));
    /// The new Smart Motor Controller
    private final SmartMotorController motor = new SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfig);
    /// The Turret Position Config
    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
            .withMaxRobotHeight(MAX_ROBOT_HEIGHT)
            .withMaxRobotLength(MAX_ROBOT_WIDTH)
            .withRelativePosition(TURRET_POSITION);
    /// The Pivot Config for the Turret
    private final PivotConfig config = new PivotConfig(motor)
            .withStartingPosition(ControlConstants.PHYSICAL_STARTING_ANGLE)
            .withHardLimit(HARD_LIMIT_REVERSE, HARD_LIMIT_FORWARD)
            .withSoftLimits(SOFT_LIMIT_REVERSE, SOFT_LIMIT_FORWARD)
            .withTelemetry(Telemetry.yamsMechPath + "Turret", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStartingPosition(SIM_STARTING_ANGLE)
            .withMechanismPositionConfig(robotToMechanism)
            .withMOI(KilogramSquareMeters.of(0.001));
    /// The final Pivot Mechanism to use as the turret.
    @Getter
    private final Pivot turret = new Pivot(config);

    /// A trigger used to stop the motor when it is trying too hard.
    private final Trigger jammedTrigger = currentSensorTrigger(Amps.of(30), Seconds.of(0.25));

    public TurretSubsystem() {
        /// A safety to automatically stop the motor if it starts trying too hard.
        jammedTrigger.whileTrue(stopTurret());
    }

    /**
     * Creates a new current sensing trigger.
     * Can be used as many sensors.
     * It can detect various jams, it can detect a piece as soon as it was grabbed,
     * it can be used to home the absolute position.
     *
     * @param triggerCurrent how high the current draw should be before triggering.
     * @param debounceTime   how long the current should be above the threshold before triggering.
     * @return a new {@link Trigger} to sense current.
     */
    public Trigger currentSensorTrigger(Current triggerCurrent, Time debounceTime) {
        // Get our motor current using YAMS, not the vendor motor.
        return new Trigger(() -> turret.getMotorController().getStatorCurrent()
                // Then check if it is greater or equal to the given threshold
                .gte(triggerCurrent))
                // To prevent minor spikes, set a debounce to wait until it is above the threshold for the given time.
                .debounce(debounceTime.in(Seconds));
    }

    /**
     * Homes the mech to the starting position.
     *
     * @return a {@link Command} to home the mech.
     */
    public Command home() {
        // Disable closed loop so we don't hit soft limits, then move backwards at a low power.
        return Commands.startRun(() -> motor.setEncoderPosition(HARD_LIMIT_FORWARD),
                        () -> motor.setVoltage(Volts.of(-2)))
                // Until we hit something, the something should be our hard stop, but it shouldn't hurt to get your hand stuck.
                .until(currentSensorTrigger(Amps.of(25), Seconds.of(0.1)))
                .finallyDo(() -> { // Then we set our new zero point and restart the closed loop.
                    motor.setEncoderPosition(ControlConstants.PHYSICAL_STARTING_ANGLE);
                });
    }

    /**
     * Finds the angle to target a position directly.
     *
     * @param swervePose
     * @param target
     * @return
     */
    public Angle angleToPose(Pose2d swervePose, Translation2d target) {
        // Find the error between the turret and goal.
        Translation2d error = target.minus(swervePose.getTranslation());
        Rotation2d robotToTarget = new Rotation2d(error.getX(), error.getY()); // Makes a new angle based on the error between the 2 poses.
        return robotToTarget.minus(swervePose.getRotation()).getMeasure();
    }

    /**
     * Runs the turret at the given speed, using PID to hold the angle when finished.
     *
     * @param turretSpeed the DutyCycle speed to run at.
     * @param isCCW       whether to turn CCW, aka lefty loosey.
     * @return a command.
     */
    public Command runTurret(double turretSpeed, boolean isCCW) {
        return turret.set(isCCW ? turretSpeed : -turretSpeed);
    }

    /**
     * Stops all power to the turret.
     *
     * @return a command that stops the turret.
     */
    public Command stopTurret() {
        return turret.set(0.0);
    }

    /**
     * Creates a new Pose3D built from defined position and current angle.
     *
     * @return the turret current 3D pose.
     */
    public Pose3d getPose3D() {
        return new Pose3d(
                TURRET_POSITION,
                new Rotation3d(0.0,
                        0.0,
                        turret.getAngle().in(Radians))); // Turret Rotation is Yaw, looking left and right.
    }

    /**
     * Gets the turret angle relative to the field.
     *
     * @param subsystem all the subsystems
     * @return the current heading.
     */
    public Rotation2d getHeading(InputBuilder.Subsystems subsystem) {
        return subsystem.swerve().getSwerveDrive().getPose().getRotation().plus(new Rotation2d(turret.getAngle()));
    }

    public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle) {
        var robotAngleRads = robotAngle.in(Radians);
        double turretVelocityX =
                robotVelocity.vxMetersPerSecond
                        + robotVelocity.omegaRadiansPerSecond
                        * (getPose3D().getY() * Math.cos(robotAngleRads)
                        - getPose3D().getX() * Math.sin(robotAngleRads));
        double turretVelocityY =
                robotVelocity.vyMetersPerSecond
                        + robotVelocity.omegaRadiansPerSecond
                        * (getPose3D().getX() * Math.cos(robotAngleRads)
                        - getPose3D().getY() * Math.sin(robotAngleRads));

        return new ChassisSpeeds(turretVelocityX,
                turretVelocityY,
                robotVelocity.omegaRadiansPerSecond + motor.getMechanismVelocity().in(RadiansPerSecond));
    }


    /**
     * Pivot is near an angle.
     *
     * @param angle {@link Angle} to be near.
     * @return {@link Trigger} on when the pivot is near another angle.
     */
    public Trigger isNear(Supplier<Angle> angle) {
        return new Trigger(isNear(angle, ANGLE_TOLERANCE));
    }

    /**
     * Pivot is near an angle.
     *
     * @param angle  {@link Angle} to be near.
     * @param within {@link Angle} within.
     * @return {@link Trigger} on when the pivot is near another angle.
     */
    public BooleanSupplier isNear(Supplier<Angle> angle, Angle within) {
        return turret.isNear(angle.get(), within);
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        turret.updateTelemetry(); // Updates the turret mechanism's telemetry data to the network tables.
        SmartDashboard.putBoolean("Telemetry/Jammed Triggers/Turret", jammedTrigger.getAsBoolean());
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        turret.simIterate();

        // Update our Mech3d Pose.
        Telemetry.Publishers.Robot.Mech3D.turretPose.accept(getPose3D());
    }
}