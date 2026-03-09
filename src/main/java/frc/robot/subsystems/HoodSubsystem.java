package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
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
import lombok.Setter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.HoodSubsystem.ControlConstants.ANGLE_TOLERANCE;
import static frc.robot.subsystems.HoodSubsystem.ControlConstants.PHYSICAL_STARTING_ANGLE;
import static frc.robot.subsystems.HoodSubsystem.HardwareConstants.*;
import static frc.robot.subsystems.TurretSubsystem.HardwareConstants.TURRET_POSITION;

public class HoodSubsystem extends SubsystemBase {
    /// The Hardware Constants for the Hood Mechanism.
    public static final class HardwareConstants {
        /// Motor Constants
        public static final int                                 MOTOR_ID                    = 14; // Spark Max CAN ID
        public static final boolean                             MOTOR_INVERTED              = false; // Inverts control direction.
        public static final MechanismGearing                    GEAR_RATIO                  = new MechanismGearing(GearBox.fromReductionStages(5, 4, 4, 56 / 40.0)); // FlyWheel Gear Ratio
        /// Motor Tuning Values
        public static final PIDController                       PID_CONTROLLER              = new PIDController( // Exponential Motion Profiling
                                                                                            60, 0, 0.01); // PID - Proportional, Integral, Derivative.
        /// Exponential Motion Profiling Constraints.
        public static final class Profiling {
            public static final Voltage                         MAX_CONTROL_VOLTAGE         = Volts.of(12); // Max Control Voltage
            public static final AngularVelocity                 MAX_ANGULAR_VELOCITY        = DegreesPerSecond.of(360); // Max Angular Velocity
            public static final AngularAcceleration             MAX_ANGULAR_ACCELERATION    = DegreesPerSecondPerSecond.of(720); // Max Angular Acceleration
        }
        public static final Time                                RAMP_RATE                   = Seconds.of(0.15); // Time it takes to reach max speed from 0.
        public static final ArmFeedforward                      FEED_FORWARD                = new ArmFeedforward(0.01, 0.2, 0.01); // Feed Forwards.
        public static final Current                             CURRENT_LIMIT               = Amp.of(40); // Current limit, Higher for faster control.
        /// Hood Constants
        public static final Mass                                HOOD_MASS                   = Pounds.of(3); // Weight of the hood mechanism.
        public static final Distance                            HOOD_LENGTH                 = Inches.of(8); // Hood Length, used in calculations and to visualize in sim.
        public static final Angle                               HORIZONTAL_OFFSET           = Degrees.of(0); // Offset required making angle 0 horizontal and parallel to the ground.
        public static final Angle                               HARD_LIMIT_REVERSE          = Degrees.of(0); // The hard limit should be a metal physical stop.
        public static final Angle                               HARD_LIMIT_FORWARD          = Degrees.of(80);
        public static final Angle                               SOFT_LIMIT_REVERSE          = Degrees.of(0); // A soft limit so we don't constantly hit the hard limit without reason.
        public static final Angle                               SOFT_LIMIT_FORWARD          = Degrees.of(80);
        /// IMPORTANT, this helps calculate the initial fuel velocity; it is the friction affecting on the ball from the hood.
        public static final double                              HOOD_COF_FACTOR             = 0.8;
        /// Sim Constants
        public static final Angle                               SIM_STARTING_ANGLE          = Degrees.of(0); // Starting Hood angle in sim.
        public static final Distance                            MAX_ROBOT_HEIGHT            = Inches.of(22); // Max robot height for visualization. TODO Push to swerve constants
        public static final Distance                            MAX_ROBOT_WIDTH             = Inches.of(29); // Max robot width for visualization.
        public static final Translation3d                       HOOD_POSITION               = TURRET_POSITION.plus(
                                                                                            new Translation3d( /// Hood position for visualization, relative to the turret position.
                                                                                            Inches.of(4.5).in(Meters),  // X-axis left positive relative to the robot center, Same as pose2d.
                                                                                            Inches.of(0).in(Meters),   // Y-axis front positive relative to the robot center, Same as pose2d.
                                                                                            Inches.of(0).in(Meters))); // Z-axis up relative to the floor.
    }
    /// The Control Constants for the Hood Mechanism.
    public static final class ControlConstants {
        public static final Angle                               PHYSICAL_STARTING_ANGLE     = Degrees.of(0);
        public static final double                              HOOD_SPEED                  = 0.3; // Predefined duty cycle speed.
        public static final Angle                               ANGLE_TOLERANCE             = Degrees.of(5); // How accurate the angle should be.
    }
    public static final class MathConstants {
        public static final LinearVelocity                      kExitVelocity               = MetersPerSecond.of(10);
    }
    /// Finally, we can initialize our mechanism.
    private final SparkMax                                      hoodMotor                   = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig                    motorConfig                 = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
            .withExponentialProfile(Profiling.MAX_CONTROL_VOLTAGE, Profiling.MAX_ANGULAR_VELOCITY, Profiling.MAX_ANGULAR_ACCELERATION)
            .withClosedLoopController(PID_CONTROLLER)
            .withGearing(GEAR_RATIO)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("Hood Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(CURRENT_LIMIT)
            .withMotorInverted(MOTOR_INVERTED)
            .withClosedLoopRampRate(RAMP_RATE)
            .withOpenLoopRampRate(RAMP_RATE)
            //.withFeedforward(FEED_FORWARD)
            .withSimFeedforward(FEED_FORWARD)
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private final SmartMotorController                          motor               = new SparkWrapper(hoodMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
    private final MechanismPositionConfig                       robotToMechanism    = new MechanismPositionConfig() /// The Turret Position Config
            .withMaxRobotHeight(MAX_ROBOT_HEIGHT)
            .withMaxRobotLength(MAX_ROBOT_WIDTH)
            .withRelativePosition(HOOD_POSITION);
    private final ArmConfig m_config = new ArmConfig(motor) /// The Arm Config for the Hood Mechanism.
            .withStartingPosition(PHYSICAL_STARTING_ANGLE)
            .withLength(HOOD_LENGTH)
            .withHardLimit(HARD_LIMIT_REVERSE, HARD_LIMIT_FORWARD)
            .withSoftLimits(SOFT_LIMIT_REVERSE, SOFT_LIMIT_FORWARD)
            .withTelemetry(Telemetry.yamsMechPath + "Hood", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withMass(HOOD_MASS)
            .withStartingPosition(SIM_STARTING_ANGLE)
            //.withHorizontalZero(HardwareConstants.HORIZONTAL_OFFSET)
            .withMechanismPositionConfig(robotToMechanism);
    @Getter
    private final Arm                               hood                = new Arm(m_config); /// The final Arm Mechanism to use as the hood.

    private final TurretSubsystem turret;

    /// A trigger used to stop the motor when it is trying too hard.
    private final Trigger jammedTrigger = currentSensorTrigger(Amps.of(20), Seconds.of(0.1));

    public HoodSubsystem(TurretSubsystem turretSubsystem) {
        this.turret = turretSubsystem;
        /// A safety to automatically stop the motor if it starts trying too hard.
        jammedTrigger.whileTrue(stopHood());
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
        return new Trigger(() -> hood.getMotorController().getStatorCurrent()
                // Then check if it is greater or equal to the given threshold
                .gte(triggerCurrent))
                // To prevent minor spikes, set a debounce to wait until it is above the threshold for the given time.
                .debounce(debounceTime.in(Seconds));
    }

    /**
     * Homes the mech to the starting position.
     * @return a {@link Command} to home the mech.
     */
    public Command home() {
        // Disable closed loop so we don't hit soft limits, then move backwards at a low power.
        return Commands.startRun(motor::stopClosedLoopController,
                        () -> motor.setVoltage(Volts.of(-2)))
                // Until we hit something, the something should be our hard stop, but it shouldn't hurt to get your hand stuck.
                .until(currentSensorTrigger(Amps.of(1.5), Seconds.of(0)))
                .finallyDo(() -> { // Then we set our new zero point and restart the closed loop.
                    motor.setPosition(PHYSICAL_STARTING_ANGLE);
                    motor.startClosedLoopController();
                });
    }


    /**
     * Runs the hood at the given speed.
     *
     * @param hoodSpeed the DutyCycle speed to run at.
     * @param isUp whether to go up.
     * @return a command.
     */
    public Command runHood(double hoodSpeed, boolean isUp) {
        return hood.set(isUp ? hoodSpeed : -hoodSpeed);
    }

    /**
     * Stops all power to the hood.
     *
     * @return a command that stops the hood.
     */
    public Command stopHood() {
        return hood.set(0.0);
    }

    /**
     * Creates a new Pose3D built from defined position and current angle.
     *
     * @return the hood current 3D pose.
     */
    public Pose3d getPose3D(TurretSubsystem turret) {
        Angle turretRotation; // Null safe turretRotation
        if (turret == null) {
            turretRotation = Degrees.of(0);
        } else {
            turretRotation = turret.getTurret().getAngle();
        }
        return new Pose3d(
                // Rotate Hood around turret
                HOOD_POSITION.rotateAround(TURRET_POSITION, new Rotation3d(
                        Degrees.of(0),
                        Degrees.of(0),
                        turretRotation)),
                // Then Rotate the hood based on turret rotation.
                new Rotation3d(
                        Degrees.of(0),
                        hood.getAngle(), // Hood Rotation is Pitch, looking up and down.
                        turretRotation));

    }

    /**
     * Hood is near an angle.
     *
     * @param angle  {@link Angle} to be near.
     * @return {@link Trigger} on when the pivot is near another angle.
     */
    public Trigger isNear(Supplier<Angle> angle)
    {
        return new Trigger(isNear(angle, ANGLE_TOLERANCE));
    }

    /**
     * Hood is near an angle.
     *
     * @param angle  {@link Angle} to be near.
     * @param within {@link Angle} within.
     * @return {@link Trigger} on when the pivot is near another angle.
     */
    public BooleanSupplier isNear(Supplier<Angle> angle, Angle within) {
        return hood.isNear(angle.get(), within);
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the hood mechanism's telemetry data to the network tables.
        hood.updateTelemetry();
        SmartDashboard.putBoolean("Telemetry/Jammed Triggers/FlyWheel", jammedTrigger.getAsBoolean());
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        hood.simIterate();

        // Update our Mech3d Pose.
        Telemetry.Publishers.Robot.Mech3D.hoodPose.accept(getPose3D(turret));
    }
}