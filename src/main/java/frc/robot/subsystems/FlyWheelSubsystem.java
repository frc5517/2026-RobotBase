package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import lombok.Getter;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.FlyWheelSubsystem.HardwareConstants.*;

public class FlyWheelSubsystem extends SubsystemBase
{
    /// Hardware Constants for the FlyWheel Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int                         MOTOR_ID                    = 25; // Spark Max CAN ID
        public static final boolean                     MOTOR_INVERTED              = false; // Inverts control direction.
        public static final MechanismGearing            GEAR_RATIO                  = new MechanismGearing(GearBox.fromReductionStages(1)); // FlyWheel Gear Ratio

        /// Motor Tuning Values
        public static final PIDController               PID_CONTROLLER              = new PIDController( // Exponential Motion Profiling
                                                                                    20, 0, 0.01); // PID - Proportional, Integral, Derivative.
        /// Trapezoidal Motion Profiling Constraints.
        public static final class Profiling {
            public static final AngularVelocity         MAX_ANGULAR_VELOCITY        = RPM.of(5000); // Max Angular Velocity
            public static final AngularAcceleration     MAX_ANGULAR_ACCELERATION    = DegreesPerSecondPerSecond.of(2500); // Max Angular Acceleration
        }
        public static final Time                        RAMP_RATE                   = Seconds.of(0.25); // Time it takes to reach max speed from 0.
        public static final SimpleMotorFeedforward      FEED_FORWARD                = new SimpleMotorFeedforward(0, 0, 0); // Feed Forwards.
        public static final Current                     CURRENT_LIMIT               = Amp.of(40); // Current limit, Higher for faster control.
        /// FlyWheel Constants
        public static final Distance                    FLYWHEEL_DIAMETER           = Inches.of(4); // Diameter of the wheel, belt, whatever is spinning on the flywheel.
        public static final Mass                        FLYWHEEL_MASS               = Pounds.of(1); // Weight of the flywheel, just what gets spun.
        public static final AngularVelocity             FLYWHEEL_MAX_SPEED          = RPM.of(8000); // Max RPM soft limits
        public static final double                      FLYWHEEL_EFFICIENCY         = .39; // Multiplicity factor used to determine how much speed was transferred,
    }
    /// Control Constants for the FlyWheel Mechanism
    public static class ControlConstants {
        public static final AngularVelocity             VELOCITY_TOLERANCE          = RPM.of(10); // How accurate the velocity should be.
        public static final AngularVelocity             TARGET_VELOCITY             = RPM.of(5000); // How fast the flywheel should spin.
    }
    /// Initialize the FlyWheel
    private final SparkMax                              indexerMotor                = new SparkMax(HardwareConstants.MOTOR_ID, MotorType.kBrushless); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig            motorConfig                 = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
            .withTrapezoidalProfile(Profiling.MAX_ANGULAR_VELOCITY, Profiling.MAX_ANGULAR_ACCELERATION)
            .withClosedLoopController(PID_CONTROLLER)
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
    private final SmartMotorController                  motor                       = new SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
    private final FlyWheelConfig                        flyWheelConfig              = new FlyWheelConfig(motor) /// The FlyWheel config.
            .withDiameter(HardwareConstants.FLYWHEEL_DIAMETER)
            .withMass(HardwareConstants.FLYWHEEL_MASS)
            .withTelemetry("FlyWheel", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withSoftLimit(HardwareConstants.FLYWHEEL_MAX_SPEED.unaryMinus(), HardwareConstants.FLYWHEEL_MAX_SPEED)
            .withSpeedometerSimulation(HardwareConstants.FLYWHEEL_MAX_SPEED);
    @Getter
    private final FlyWheel                              flyWheel                    = new FlyWheel(flyWheelConfig); /// The final FlyWheel Mechanism.

    public FlyWheelSubsystem() {
    }

    /**
     * Runs the flywheel at the given speed.
     *
     * @param flywheelSpeed the DutyCycle speed to run at.
     * @param isOut whether to spin out or in.
     * @return a command.
     */
    public Command runFlyWheel(double flywheelSpeed, boolean isOut) {
        return flyWheel.set(isOut ? flywheelSpeed : -flywheelSpeed);
    }

    /**
     * Stops all power to the flywheel.
     *
     * @return a command that stops the flywheel.
     */
    public Command stopFlyWheel() {
        return flyWheel.set(0.0);
    }

    /**
     * @return a {@link Command} that launches sim fuel.
     */
    public Command simShoot(Supplier<AngularVelocity> flyWheelSpeed) {
        return runOnce(() -> {
            if (RobotBase.isSimulation()) {
                SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
                    SwerveSubsystem.SwerveState.CurrentPose.getTranslation(),
                    new Translation2d(Inches.of(5), Inches.of(0)),
                    SwerveSubsystem.SwerveState.CurrentSpeeds,
                    TurretSubsystem.TurretState.getCurrentHeading().rotateBy(Rotation2d.k180deg),
                    Inches.of(23),
                    MetersPerSecond.of(flyWheelSpeed.get().in(RadiansPerSecond) * (FLYWHEEL_DIAMETER.in(Meters) / 2) * FLYWHEEL_EFFICIENCY),
                    HoodSubsystem.HoodState.CurrentAngle.plus(Degrees.of(90))));
            }});
    }

    /**
     * @return a {@link Command} that launches sim fuel.
     */
    public Command simShoot() {
        return runOnce(() -> {
            if (RobotBase.isSimulation()) {
                SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
                        SwerveSubsystem.SwerveState.CurrentPose.getTranslation(),
                        new Translation2d(Inches.of(5), Inches.of(0)),
                        SwerveSubsystem.SwerveState.CurrentSpeeds,
                        TurretSubsystem.TurretState.getCurrentHeading().rotateBy(Rotation2d.k180deg),
                        Inches.of(23),
                        MetersPerSecond.of(7),
                        HoodSubsystem.HoodState.CurrentAngle.plus(Degrees.of(90))));
            }});
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the flywheel mechanism's telemetry data to the network tables.
        flyWheel.updateTelemetry();
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
