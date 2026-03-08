package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import lombok.Getter;
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
import static frc.robot.subsystems.KickerSubsystem.HardwareConstants.*;

public class KickerSubsystem extends SubsystemBase
{
    /// Hardware Constants for the Kicker Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int                                 MOTOR_ID            = 13; // Spark Max CAN ID
        public static final boolean                             MOTOR_INVERTED      = false; // Inverts control direction.
        public static final MechanismGearing                    GEAR_RATIO          = new MechanismGearing(GearBox.fromReductionStages(3, 4)); // Kicker Gear Ratio
        /// Motor Tuning Values
        public static final PIDController PID_CONTROLLER              = new PIDController( // Exponential Motion Profiling
                20, 0, 0.01); // PID - Proportional, Integral, Derivative.
        /// Exponential Motion Profiling Constraints.
        public static final class Profiling {
            public static final Voltage                         MAX_CONTROL_VOLTAGE         = Volts.of(12); // Max Control Voltage
            public static final AngularVelocity                 MAX_ANGULAR_VELOCITY        = DegreesPerSecond.of(180); // Max Angular Velocity
            public static final AngularAcceleration             MAX_ANGULAR_ACCELERATION    = DegreesPerSecondPerSecond.of(360); // Max Angular Acceleration
        }
        public static final Time                                RAMP_RATE           = Seconds.of(0.25); // Time it takes to reach max speed from 0.
        public static final SimpleMotorFeedforward              FEED_FORWARD        = new SimpleMotorFeedforward(0, 0, 0); // Feed Forwards, likely to be left empty.
        public static final Current                             CURRENT_LIMIT       = Amp.of(20); // Limits the current, this is a simple kicker. We want the limit low so we don't break things in the case of a jam.
        /// Kicker Constants
        public static final int                                 MAX_FUEL_CAPACITY   = 50;
        public static final Distance                            INDEXER_DIAMETER    = Inches.of(3); // Diameter of the wheel, belt, whatever is spinning on the kicker.
        public static final AngularVelocity                     INDEXER_MAX_SPEED   = RPM.of(200); // Max RPM soft limits
    }
    /// Control Constants for the Kicker Mechanism.
    public static class ControlConstants {
        public static final boolean                             INFINITE_SIM_INTAKE = true; // Makes it so you can kicker more than the max capacity.
        public static final AngularVelocity                     VELOCITY_TOLERANCE  = DegreesPerSecond.of(1); // How accurate the velocity should be.
        public static final AngularVelocity                     TARGET_VELOCITY     = DegreesPerSecond.of(60); // How fast the flywheel should spin.
    }
    private final SparkMax                                      kickerMotor    = new SparkMax(MOTOR_ID, MotorType.kBrushed); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig                    motorConfig     = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
                .withExponentialProfile(KickerSubsystem.HardwareConstants.Profiling.MAX_CONTROL_VOLTAGE, KickerSubsystem.HardwareConstants.Profiling.MAX_ANGULAR_VELOCITY, KickerSubsystem.HardwareConstants.Profiling.MAX_ANGULAR_ACCELERATION)
                .withClosedLoopController(PID_CONTROLLER)
                .withGearing(GEAR_RATIO)
                .withIdleMode(MotorMode.BRAKE)
                .withTelemetry("Kicker Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withMotorInverted(MOTOR_INVERTED)
                .withClosedLoopRampRate(RAMP_RATE)
                .withOpenLoopRampRate(RAMP_RATE)
                //.withFeedforward(FEED_FORWARD)
                .withSimFeedforward(FEED_FORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController                          motor           = new SparkWrapper(kickerMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
    private final FlyWheelConfig                                kickerConfig    = new FlyWheelConfig(motor) /// The FlyWheel config for the Kicker.
                .withDiameter(INDEXER_DIAMETER)
                .withMass(Pounds.of(1)) // Kicker Doesn't need to specify weight.
                .withTelemetry("Kicker", Telemetry.telemetryVerbosity.yamsVerbosity)
                .withSoftLimit(INDEXER_MAX_SPEED.unaryMinus(), INDEXER_MAX_SPEED)
                .withSpeedometerSimulation(INDEXER_MAX_SPEED);
    @Getter
    private final FlyWheel kicker = new FlyWheel(kickerConfig); /// The final FlyWheel Mechanism to use as the smart Kicker.

    /// A trigger used to stop the motor when it is trying too hard.
    private final Trigger jammedTrigger = currentSensorTrigger(Amps.of(40), Seconds.of(0.5));

    /**
     *
     */
    public KickerSubsystem() {
        /// A safety to automatically stop the motor if it starts trying too hard.
        jammedTrigger.whileTrue(stopKicker());
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
        return new Trigger(() -> kicker.getMotorController().getStatorCurrent()
                // Then check if it is greater or equal to the given threshold
                .gte(triggerCurrent))
                // To prevent minor spikes, set a debounce to wait until it is above the threshold for the given time.
                .debounce(debounceTime.in(Seconds));
    }


    /**
     * Runs the kicker at the given speed.
     *
     * @param kickerSpeed the DutyCycle speed to run at.
     * @param isIn whether to spin in or out.
     * @return a command.
     */
    public Command runKicker(double kickerSpeed, boolean isIn) {
        return kicker.set(isIn ? kickerSpeed : -kickerSpeed).unless(jammedTrigger);
    }

    /**
     * Stops all power to the kicker.
     *
     * @return a command that stops the kicker.
     */
    public Command stopKicker() {
        return kicker.set(0.0);
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the kicker mechanism's telemetry data to the network tables.
        kicker.updateTelemetry();
        SmartDashboard.putBoolean("Telemetry/Jammed Triggers/Kicker", jammedTrigger.getAsBoolean());
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        kicker.simIterate();
    }
}