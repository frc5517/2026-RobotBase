package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import frc.robot.util.BrushedSparkWrapper;
import lombok.Getter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.IndexerSubsystem.HardwareConstants.*;

public class IndexerSubsystem extends SubsystemBase {
    /// Hardware Constants for the Indexer Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int                                 MOTOR_ID            = 12; // Spark Max CAN ID
        public static final boolean                             MOTOR_INVERTED      = false; // Inverts control direction.
        public static final MechanismGearing                    GEAR_RATIO          = new MechanismGearing(GearBox.fromReductionStages(5, 3)); // Indexer Gear Ratio
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
        public static final Current                             CURRENT_LIMIT       = Amp.of(30); // Limits the current, this is a simple indexer. We want the limit low so we don't break things in the case of a jam.
        /// Indexer Constants
        public static final Distance                            INDEXER_DIAMETER    = Inches.of(3); // Diameter of the wheel, belt, whatever is spinning on the indexer.
        public static final AngularVelocity                     INDEXER_MAX_SPEED   = RPM.of(120); // Max RPM soft limits
    }
    /// Control Constants for the Indexer Mechanism.
    public static class ControlConstants {
        public static final AngularVelocity                     VELOCITY_TOLERANCE  = DegreesPerSecond.of(1); // How accurate the velocity should be.
        public static final AngularVelocity                     TARGET_VELOCITY     = DegreesPerSecond.of(60); // How fast the flywheel should spin.
    }
    /// Indexer uses a custom Brushed wrapper, brushless code is just commented out.
//    private final SparkMax                                      indexerMotor    = new SparkMax(HardwareConstants.MOTOR_ID, MotorType.kBrushless); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig                    motorConfig     = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
            .withExponentialProfile(Profiling.MAX_CONTROL_VOLTAGE, Profiling.MAX_ANGULAR_VELOCITY, Profiling.MAX_ANGULAR_ACCELERATION)
            .withClosedLoopController(PID_CONTROLLER)
            .withGearing(HardwareConstants.GEAR_RATIO)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("Indexer Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(HardwareConstants.CURRENT_LIMIT)
            .withMotorInverted(HardwareConstants.MOTOR_INVERTED)
            .withClosedLoopRampRate(HardwareConstants.RAMP_RATE)
            .withOpenLoopRampRate(HardwareConstants.RAMP_RATE)
            //.withFeedforward(HardwareConstants.FEED_FORWARD)
            .withSimFeedforward(HardwareConstants.FEED_FORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP);
//    private final SmartMotorController                          motor           = new SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
//    private final FlyWheelConfig                                indexerConfig   = new FlyWheelConfig(motor) /// The FlyWheel config for the Indexer.
//            .withDiameter(HardwareConstants.INDEXER_DIAMETER)
//            .withMass(Pounds.of(1)) // Indexer Doesn't need to specify weight.
//            .withTelemetry("Indexer", Telemetry.telemetryVerbosity.yamsVerbosity)
//            .withSoftLimit(HardwareConstants.INDEXER_MAX_SPEED.unaryMinus(), HardwareConstants.INDEXER_MAX_SPEED)
//            .withSpeedometerSimulation(HardwareConstants.INDEXER_MAX_SPEED);
//    @Getter
//    private final FlyWheel                                      indexer         = new FlyWheel(indexerConfig); /// The final FlyWheel Mechanism to use as the smart Indexer.
    private final BrushedSparkWrapper indexer = new BrushedSparkWrapper(MOTOR_ID, motorConfig); /// Custom dumb brushed spark wrapper made to easily replace a YAMS Spark Max object.

    /// A trigger used to stop the motor when it is trying too hard.
    private final Trigger jammedTrigger = currentSensorTrigger(CURRENT_LIMIT, Seconds.of(1));

    public IndexerSubsystem() {
        /// A safety to automatically stop the motor if it starts trying too hard.
        //jammedTrigger.whileTrue(stopIndexer());
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
        return new Trigger(() -> indexer.getMotorController().getStatorCurrent()
                // Then check if it is greater or equal to the given threshold
                .gte(triggerCurrent))
                // To prevent minor spikes, set a debounce to wait until it is above the threshold for the given time.
                .debounce(debounceTime.in(Seconds));
    }

    /**
     * Runs the indexer at the given speed.
     *
     * @param indexSpeed the DutyCycle speed to run at.
     * @param isIn whether to spin in or out.
     * @return a command.
     */
    public Command runIndexer(double indexSpeed, boolean isIn) {
        return indexer.set(isIn ? indexSpeed : -indexSpeed);
    }

    /**
     * Stops all power to the indexer.
     *
     * @return a command that stops the indexer.
     */
    public Command stopIndexer() {
        return indexer.set(0.0);
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the indexer mechanism's telemetry data to the network tables.
        indexer.updateTelemetry();
        SmartDashboard.putBoolean("Telemetry/Jammed Triggers/Indexer", jammedTrigger.getAsBoolean());
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        indexer.simIterate();
    }
}
