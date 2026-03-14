package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.InputBuilder;
import frc.robot.Telemetry;
import lombok.Getter;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.FlyWheelSubsystem.HardwareConstants.*;
import static frc.robot.subsystems.FlyWheelSubsystem.ControlConstants.*;

public class FlyWheelSubsystem extends SubsystemBase
{
    /// Hardware Constants for the FlyWheel Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int                         MOTOR_ID                    = 15; // Spark Max CAN ID
        public static final boolean                     MOTOR_INVERTED              = true; // Inverts control direction.
        public static final MechanismGearing            GEAR_RATIO                  = new MechanismGearing(36 / 60.0); // FlyWheel Gear Ratio

        /// Motor Tuning Values
        public static final PIDController               PID_CONTROLLER              = new PIDController( // Exponential Motion Profiling
                                                                                    60, 0, 0.0); // PID - Proportional, Integral, Derivative.
        /// Trapezoidal Motion Profiling Constraints.
        public static final class Profiling {
            public static final AngularVelocity         MAX_ANGULAR_VELOCITY        = RPM.of(5000); // Max Angular Velocity
            public static final AngularAcceleration     MAX_ANGULAR_ACCELERATION    = DegreesPerSecondPerSecond.of(15000); // Max Angular Acceleration
        }
        public static final Time                        RAMP_RATE                   = Seconds.of(0.25); // Time it takes to reach max speed from 0.
        public static final SimpleMotorFeedforward      FEED_FORWARD                = new SimpleMotorFeedforward(0, 1, 0); // Feed Forwards.
        public static final Current                     CURRENT_LIMIT               = Amp.of(60); // Current limit, Higher for faster control.
        /// FlyWheel Constants
        public static final Distance                    FLYWHEEL_DIAMETER           = Inches.of(4); // Diameter of the wheel, belt, whatever is spinning on the flywheel.
        public static final Mass                        FLYWHEEL_MASS               = Pounds.of(1); // Weight of the flywheel, just what gets spun.
        public static final AngularVelocity             FLYWHEEL_MAX_SPEED          = RPM.of(80000); // Max RPM soft limits
        public static final double                      FLYWHEEL_EFFICIENCY         = .38; // Multiplicity factor used to determine how much speed was transferred,
    }
    /// Control Constants for the FlyWheel Mechanism
    public static class ControlConstants {
        public static final AngularVelocity             VELOCITY_TOLERANCE          = RPM.of(100); // How accurate the velocity should be.
    }
    /// Initialize the FlyWheel
    private final TalonFX                              indexerMotor                = new TalonFX(HardwareConstants.MOTOR_ID); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig            motorConfig                 = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
            .withTrapezoidalProfile(Profiling.MAX_ANGULAR_VELOCITY, Profiling.MAX_ANGULAR_ACCELERATION)
                .withVelocityTrapezoidalProfile(true)
            .withClosedLoopController(PID_CONTROLLER)
            .withGearing(HardwareConstants.GEAR_RATIO)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("FlyWheel Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(HardwareConstants.CURRENT_LIMIT)
            .withMotorInverted(HardwareConstants.MOTOR_INVERTED)
            .withClosedLoopRampRate(HardwareConstants.RAMP_RATE)
            .withOpenLoopRampRate(HardwareConstants.RAMP_RATE)
            .withFeedforward(HardwareConstants.FEED_FORWARD)
            .withSimFeedforward(HardwareConstants.FEED_FORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController                  motor                       = new TalonFXWrapper(indexerMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
    private final FlyWheelConfig                        flyWheelConfig              = new FlyWheelConfig(motor) /// The FlyWheel config.
            .withDiameter(HardwareConstants.FLYWHEEL_DIAMETER)
            .withMass(HardwareConstants.FLYWHEEL_MASS)
            .withTelemetry(Telemetry.yamsMechPath + "FlyWheel", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withSoftLimit(HardwareConstants.FLYWHEEL_MAX_SPEED.unaryMinus(), HardwareConstants.FLYWHEEL_MAX_SPEED)
            .withSpeedometerSimulation(HardwareConstants.FLYWHEEL_MAX_SPEED);
    @Getter
    private final FlyWheel                              flyWheel                    = new FlyWheel(flyWheelConfig); /// The final FlyWheel Mechanism.

    /// A trigger used to stop the motor when it is trying too hard.
    private final Trigger jammedTrigger = currentSensorTrigger(Amps.of(40), Seconds.of(0.2));

    public FlyWheelSubsystem() {
        /// A safety to automatically stop the motor if it starts trying too hard.
        // Disabled for now since I want to see about pushing 60a at some point.
        //jammedTrigger.whileTrue(stopFlyWheel());
    }

    /**
     * Creates a new current sensing trigger.
     * Can be used as many sensors.
     * It can detect various jams, it can detect a piece as soon as it was grabbed,
     * it can be used to home the absolute position.
     *
     * @param triggerCurrent how high the current draw should be before triggering.
     * @param debounceTime how long the current should be above the threshold before triggering.
     *
     * @return a new {@link Trigger} to sense current.
     */
    public Trigger currentSensorTrigger(Current triggerCurrent, Time debounceTime) {
        // Get our motor current using YAMS, not the vendor motor.
        return new Trigger(() -> flyWheel.getMotorController().getStatorCurrent()
                // Then check if it is greater or equal to the given threshold
                .gte(triggerCurrent))
                // To prevent minor spikes, set a debounce to wait until it is above the threshold for the given time.
                .debounce(debounceTime.in(Seconds));
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
    public Command simShoot(InputBuilder.Subsystems subsystems, Supplier<AngularVelocity> flyWheelSpeed) {
        return Commands.runOnce(() -> {
            if (RobotBase.isSimulation()) {
                SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
                    subsystems.swerve().getSwerveDrive().getSimulationDriveTrainPose().get().getTranslation(),
                    new Translation2d(Inches.of(5), Inches.of(0)),
                    subsystems.swerve().getSwerveDrive().getFieldVelocity(),
                    subsystems.turret().getHeading(subsystems).rotateBy(Rotation2d.k180deg),
                    Inches.of(23),
                    MetersPerSecond.of(flyWheelSpeed.get().in(RadiansPerSecond) * (FLYWHEEL_DIAMETER.in(Meters) / 2) * FLYWHEEL_EFFICIENCY),
                    subsystems.hood().getHood().getAngle().plus(Degrees.of(90)))
                    .withProjectileTrajectoryDisplayCallBack((trajectory) -> Telemetry.Publishers.Robot.Mech3D.fuelTrajectory.accept(trajectory.toArray(Pose3d[]::new))));
            }});
    }

    /**
     * @return a {@link Command} that launches sim fuel.
     */
    public Command simShoot(InputBuilder.Subsystems subsystems) {
        return runOnce(() -> {
            if (RobotBase.isSimulation()) {
                SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
                        subsystems.swerve().getSwerveDrive().getSimulationDriveTrainPose().get().getTranslation(),
                        new Translation2d(Inches.of(5), Inches.of(0)),
                        subsystems.swerve().getSwerveDrive().getFieldVelocity(),
                        subsystems.turret().getHeading(subsystems).rotateBy(Rotation2d.k180deg),
                        Inches.of(23),
                        MetersPerSecond.of(7),
                        subsystems.hood().getHood().getAngle().plus(Degrees.of(90)))
                        .withProjectileTrajectoryDisplayCallBack((trajectory) -> Telemetry.Publishers.Robot.Mech3D.fuelTrajectory.accept(trajectory.toArray(Pose3d[]::new))));
            }});
    }

    /**
     * FlyWheel is near a speed.
     *
     * @param speed  {@link AngularVelocity} to be near.
     * @return Trigger on when the FlyWheel is near another speed.
     */
    public Trigger isNear(Supplier<AngularVelocity> speed)
    {
        return new Trigger(isNear(speed, VELOCITY_TOLERANCE));
    }


    /**
     * FlyWheel is near a speed.
     *
     * @param speed  {@link AngularVelocity} to be near.
     * @param within {@link AngularVelocity} within.
     * @return Trigger on when the FlyWheel is near another speed.
     */
    public BooleanSupplier isNear(Supplier<AngularVelocity> speed, AngularVelocity within)
    {
        return () -> flyWheel.getSpeed().isNear(speed.get(), within);
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the flywheel mechanism's telemetry data to the network tables.
        flyWheel.updateTelemetry();
        SmartDashboard.putBoolean("Telemetry/Jammed Triggers/FlyWheel", jammedTrigger.getAsBoolean());
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