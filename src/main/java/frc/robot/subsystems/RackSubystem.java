package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import lombok.Getter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.RackSubystem.ControlConstants.*;
import static frc.robot.subsystems.RackSubystem.HardwareConstants.*;

public class RackSubystem extends SubsystemBase {
    /// The Hardware Constants for the Rack Mechanism.
    public static final class HardwareConstants {
        /// Motor Constants
        public static final int MOTOR_ID = 17; // Spark Max CAN ID
        public static final int FOLLOWER_ID = 16;
        public static final boolean MOTOR_INVERTED = true; // Inverts control direction.
        public static final MechanismGearing GEAR_RATIO = new MechanismGearing(GearBox.fromReductionStages(3, 5)); // FlyWheel Gear Ratio
        /// Motor Tuning Values
        public static final PIDController PID_CONTROLLER = new PIDController( // Exponential Motion Profiling
                60, 0, 0.01); // PID - Proportional, Integral, Derivative.

        /// Exponential Motion Profiling Constraints.
        public static final class Profiling {
            public static final Voltage MAX_CONTROL_VOLTAGE = Volts.of(12); // Max Control Voltage
            public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(10); // Max Angular Velocity
            public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(50); // Max Angular Acceleration
        }

        public static final Time RAMP_RATE = Seconds.of(0.15); // Time it takes to reach max speed from 0.
        public static final ArmFeedforward FEED_FORWARD = new ArmFeedforward(0.1, 0.1, 0.0); // Feed Forwards.
        public static final Current CURRENT_LIMIT = Amp.of(30); // Current limit, Higher for faster control.
        /// Rack Constants
        public static final Distance PINION_CIRCUMFERENCE = Inches.of(6.66432);
        public static final Mass INTAKE_WEIGHT = Pounds.of(10); // Weight of the rack mechanism.
        public static final Angle ANGLE_FROM_FLOOR = Degrees.of(-10); // How angled from the floor the rack is, up is positive, Parallel is 0.
        public static final Distance HARD_LIMIT_REVERSE = Inches.of(0); // The hard limit used in simulation acting like a metal physical stop.
        public static final Distance HARD_LIMIT_FORWARD = Inches.of(12);
        public static final Distance SOFT_LIMIT_REVERSE = Inches.of(0); // A soft limit so we don't constantly hit the hard limit without reason.
        public static final Distance SOFT_LIMIT_FORWARD = Inches.of(12);
        /// Sim Constants
        public static final Distance SIM_STARTING_HEIGHT = Inches.of(0); // Starting Extension in sim.
        public static final Distance MAX_ROBOT_HEIGHT = Inches.of(22); // Max robot height for visualization. TODO Push to swerve constants
        public static final Distance MAX_ROBOT_WIDTH = Inches.of(29); // Max robot width for visualization.
    }

    /// The Control Constants for the Rack Mechanism.
    public static final class ControlConstants {
        public static final Distance PHYSICAL_STARTING_EXTENSION = Inches.of(0);
        public static final Distance DISTANCE_TOLERANCE = Inches.of(0.5); // How accurate the angle should be.
        public static final Distance GOAL_FOR_INTAKE = Inches.of(11);
    }

    /// Finally, we can initialize our mechanism.
    private final SparkMax rackMotor = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax followerMotor = new SparkMax(FOLLOWER_ID, SparkLowLevel.MotorType.kBrushless);
    /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
            .withExponentialProfile(Profiling.MAX_CONTROL_VOLTAGE, Profiling.MAX_ANGULAR_VELOCITY, Profiling.MAX_ANGULAR_ACCELERATION)
            .withClosedLoopController(PID_CONTROLLER)
            .withGearing(GEAR_RATIO)
            .withMechanismCircumference(PINION_CIRCUMFERENCE)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("Rack Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(CURRENT_LIMIT)
            .withMotorInverted(MOTOR_INVERTED)
            .withClosedLoopRampRate(RAMP_RATE)
            .withOpenLoopRampRate(RAMP_RATE)
            //.withFeedforward(FEED_FORWARD)
            .withSimFeedforward(FEED_FORWARD)
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);

    private final SmartMotorControllerConfig followerConfig = motorConfig.clone()
            .withMotorInverted(!MOTOR_INVERTED); // Follower is reverse of leader.
    private final SmartMotorController followerSMC = new SparkWrapper(followerMotor, DCMotor.getNEO(1), followerConfig);

    private final SmartMotorControllerConfig leaderConfig = motorConfig.clone()
            .withLooselyCoupledFollowers(followerSMC);

    private final SmartMotorController motor = new SparkWrapper(rackMotor, DCMotor.getNEO(1), leaderConfig);
    /// The new Smart Motor Controller
    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig() /// The Turret Position Config
            .withMaxRobotHeight(MAX_ROBOT_HEIGHT)
            .withMaxRobotLength(MAX_ROBOT_WIDTH);
    private final ElevatorConfig m_config = new ElevatorConfig(motor) /// The Arm Config for the Rack Mechanism.
            .withHorizontalElevator()
            .withAngle(ANGLE_FROM_FLOOR)
            .withHardLimits(HARD_LIMIT_REVERSE, HARD_LIMIT_FORWARD)
            .withSoftLimits(SOFT_LIMIT_REVERSE, SOFT_LIMIT_FORWARD)
            .withTelemetry("Rack", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withMass(INTAKE_WEIGHT)
            .withSimStartingHeight(SIM_STARTING_HEIGHT)
            .withMechanismPositionConfig(robotToMechanism);
    @Getter
    private final Elevator rack = new Elevator(m_config);

    /// A trigger used to stop the motor when it is trying too hard.
    private final Trigger jammedTrigger = currentSensorTrigger(Amps.of(20), Seconds.of(0.25));

    public RackSubystem() {
        /// A safety to automatically stop the motor if it starts trying too hard.
        jammedTrigger.whileTrue(stopRack());
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
        return new Trigger(() -> rack.getMotorController().getStatorCurrent()
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
        return Commands.startRun(() -> motor.setEncoderPosition(HardwareConstants.HARD_LIMIT_FORWARD.times(2)),
                        () -> motor.setVoltage(Volts.of(-1.5)))
                // Until we hit something, the something should be our hard stop, but it shouldn't hurt to get your hand stuck.
                .until(currentSensorTrigger(Amps.of(10), Seconds.of(0.05)))
                .andThen(stopRack())
                .finallyDo(() -> { // Then we set our new zero point and restart the closed loop.
                    motor.setEncoderPosition(PHYSICAL_STARTING_EXTENSION);
                });
    }

    /**
     * Extends or retracts the hopper depending on isOut.
     *
     * @param isOut whether to extend or retract the rack to the working distance.
     * @return a command to move the rack to a goal and hold it there.
     */
    public Command runToRack(boolean isOut) {
        Distance goal = isOut ? GOAL_FOR_INTAKE : Inches.of(0.125);// Retract to 1/8in away from the hard stop to be safe.
        return rack.runTo(goal, Inches.of(.1));
    }

    /**
     * Checks whether the rack is at the goal within tolerance.
     *
     * @param isOut whether the goal is working extension or retraction.
     * @return whether the rack is at the distance.
     */
    public Trigger atGoal(boolean isOut) {
        Distance goal = isOut ? GOAL_FOR_INTAKE : Inches.of(0);
        return rack.isNear(goal, DISTANCE_TOLERANCE);
    }

    /**
     * Runs the rack at the given speed.
     *
     * @param rackSpeed the DutyCycle speed to run at.
     * @param isUp      whether to go up.
     * @return a command.
     */
    public Command runRack(double rackSpeed, boolean isUp) {
        return rack.set(isUp ? rackSpeed : -rackSpeed);
    }

    /**
     * Stops all power to the rack.
     *
     * @return a command that stops the rack.
     */
    public Command stopRack() {
        return rack.set(0.0);
    }


    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the rack mechanism's telemetry data to the network tables.
        rack.updateTelemetry();
        SmartDashboard.putBoolean("Telemetry/Jammed Triggers/FlyWheel", jammedTrigger.getAsBoolean());
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        rack.simIterate();
    }
}