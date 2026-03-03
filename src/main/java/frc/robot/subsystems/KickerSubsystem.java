package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
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

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.KickerSubsystem.ControlConstants.VELOCITY_TOLERANCE;
import static frc.robot.subsystems.KickerSubsystem.HardwareConstants.*;

public class KickerSubsystem extends SubsystemBase
{
    /// Hardware Constants for the Kicker Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int                         MOTOR_ID                    = 28; // Spark Max CAN ID
        public static final boolean                     MOTOR_INVERTED              = false; // Inverts control direction.
        public static final MechanismGearing            GEAR_RATIO                  = new MechanismGearing(GearBox.fromReductionStages(1)); // Kicker Gear Ratio

        /// Motor Tuning Values
        public static final PIDController               PID_CONTROLLER              = new PIDController( // Exponential Motion Profiling
                                                                                    60, 0, 0.01); // PID - Proportional, Integral, Derivative.
        /// Trapezoidal Motion Profiling Constraints.
        public static final class Profiling {
            public static final AngularVelocity         MAX_ANGULAR_VELOCITY        = RPM.of(5000); // Max Angular Velocity
            public static final AngularAcceleration     MAX_ANGULAR_ACCELERATION    = DegreesPerSecondPerSecond.of(15000); // Max Angular Acceleration
        }
        public static final Time                        RAMP_RATE                   = Seconds.of(0.25); // Time it takes to reach max speed from 0.
        public static final SimpleMotorFeedforward      FEED_FORWARD                = new SimpleMotorFeedforward(0, 0, 0); // Feed Forwards.
        public static final Current                     CURRENT_LIMIT               = Amp.of(40); // Current limit, Higher for faster control.
        /// Kicker Constants
        public static final Distance                    FLYWHEEL_DIAMETER           = Inches.of(4); // Diameter of the wheel, belt, whatever is spinning on the kicker.
        public static final Mass                        FLYWHEEL_MASS               = Pounds.of(1); // Weight of the kicker, just what gets spun.
        public static final AngularVelocity             FLYWHEEL_MAX_SPEED          = RPM.of(8000); // Max RPM soft limits
        public static final double                      FLYWHEEL_EFFICIENCY         = .39; // Multiplicity factor used to determine how much speed was transferred,
    }
    /// Control Constants for the Kicker Mechanism
    public static class ControlConstants {
        public static final AngularVelocity             VELOCITY_TOLERANCE          = RPM.of(1000); // How accurate the velocity should be.
    }
    /// Initialize the Kicker
    private final SparkMax                              indexerMotor                = new SparkMax(HardwareConstants.MOTOR_ID, MotorType.kBrushless); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig            motorConfig                 = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
            .withTrapezoidalProfile(Profiling.MAX_ANGULAR_VELOCITY, Profiling.MAX_ANGULAR_ACCELERATION)
            .withClosedLoopController(PID_CONTROLLER)
            .withGearing(HardwareConstants.GEAR_RATIO)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("Kicker Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withStatorCurrentLimit(HardwareConstants.CURRENT_LIMIT)
            .withMotorInverted(HardwareConstants.MOTOR_INVERTED)
            .withClosedLoopRampRate(HardwareConstants.RAMP_RATE)
            .withOpenLoopRampRate(HardwareConstants.RAMP_RATE)
            .withFeedforward(HardwareConstants.FEED_FORWARD)
            .withSimFeedforward(HardwareConstants.FEED_FORWARD)
            .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController                  motor                       = new SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
    private final FlyWheelConfig                        flyWheelConfig              = new FlyWheelConfig(motor) /// The Kicker config.
            .withDiameter(HardwareConstants.FLYWHEEL_DIAMETER)
            .withMass(HardwareConstants.FLYWHEEL_MASS)
            .withTelemetry("Kicker", Telemetry.telemetryVerbosity.yamsVerbosity)
            .withSoftLimit(HardwareConstants.FLYWHEEL_MAX_SPEED.unaryMinus(), HardwareConstants.FLYWHEEL_MAX_SPEED)
            .withSpeedometerSimulation(HardwareConstants.FLYWHEEL_MAX_SPEED);
    @Getter
    private final FlyWheel kicker = new FlyWheel(flyWheelConfig); /// The final Kicker Mechanism.

    public KickerSubsystem() {
    }

    /**
     * Runs the kicker at the given speed.
     *
     * @param kickerSpeed the DutyCycle speed to run at.
     * @param isOut whether to spin out or in.
     * @return a command.
     */
    public Command runKicker(double kickerSpeed, boolean isOut) {
        return kicker.set(isOut ? kickerSpeed : -kickerSpeed);
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
     * Kicker is near a speed.
     *
     * @param speed  {@link AngularVelocity} to be near.
     * @return Trigger on when the Kicker is near another speed.
     */
    public Trigger isNear(Supplier<AngularVelocity> speed)
    {
        return new Trigger(isNear(speed, VELOCITY_TOLERANCE));
    }


    /**
     * Kicker is near a speed.
     *
     * @param speed  {@link AngularVelocity} to be near.
     * @param within {@link AngularVelocity} within.
     * @return Trigger on when the Kicker is near another speed.
     */
    public BooleanSupplier isNear(Supplier<AngularVelocity> speed, AngularVelocity within)
    {
        return () -> kicker.getSpeed().isNear(speed.get(), within);
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the kicker mechanism's telemetry data to the network tables.
        kicker.updateTelemetry();
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
