package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Telemetry;
import lombok.Getter;
import lombok.Setter;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.IntakeSubsystem.IntakeState.*;
import static frc.robot.subsystems.IntakeSubsystem.ControlConstants.*;
import static frc.robot.subsystems.IntakeSubsystem.HardwareConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    /// Hardware Constants for the Intake Mechanism.
    public static class HardwareConstants {
        /// Motor Constants
        public static final int                                 MOTOR_ID            = 27; // Spark Max CAN ID
        public static final boolean                             MOTOR_INVERTED      = false; // Inverts control direction.
        public static final MechanismGearing                    GEAR_RATIO          = new MechanismGearing(GearBox.fromReductionStages(3, 4)); // Intake Gear Ratio
        /// Motor Tuning Values
        public static final ExponentialProfilePIDController     PID_CONTROLLER      = new ExponentialProfilePIDController( // Exponential Motion Profiling
                1, 0, 0, // PID - Proportional, Integral, Derivative.
                ExponentialProfilePIDController.createConstraints( /// Exponential Motion Profiling Constraints.
                        Volts.of(10), // Max Control Voltage
                        DegreesPerSecond.of(180), // Max Angular Velocity
                        DegreesPerSecondPerSecond.of(360))); // Max Angular Acceleration
        public static final Time                                RAMP_RATE           = Seconds.of(0.25); // Time it takes to reach max speed from 0.
        public static final SimpleMotorFeedforward              FEED_FORWARD        = new SimpleMotorFeedforward(0, 0, 0); // Feed Forwards, likely to be left empty.
        public static final Current                             CURRENT_LIMIT       = Amp.of(20); // Limits the current, this is a simple intake. We want the limit low so we don't break things in the case of a jam.
        /// Intake Constants
        public static final int                                 MAX_FUEL_CAPACITY   = 50;
        public static final Distance                            INDEXER_DIAMETER    = Inches.of(3); // Diameter of the wheel, belt, whatever is spinning on the intake.
        public static final AngularVelocity                     INDEXER_MAX_SPEED   = RPM.of(200); // Max RPM soft limits
    }
    /// Control Constants for the Intake Mechanism.
    public static class ControlConstants {
        public static final boolean                             INFINITE_SIM_INTAKE = true; // Makes it so you can intake more than the max capacity.
        public static final AngularVelocity                     VELOCITY_TOLERANCE  = DegreesPerSecond.of(1); // How accurate the velocity should be.
        public static final AngularVelocity                     TARGET_VELOCITY     = DegreesPerSecond.of(60); // How fast the flywheel should spin.
    }
    private final SparkMax                                      intakeMotor    = new SparkMax(MOTOR_ID, MotorType.kBrushless); /// The Normal Rev Vendor SparkMax Object.
    private final SmartMotorControllerConfig                    motorConfig     = new SmartMotorControllerConfig(this) /// The Smart Motor Controller Configuration.
                    .withClosedLoopController(PID_CONTROLLER)
                    .withGearing(GEAR_RATIO)
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("Intake Motor", Telemetry.telemetryVerbosity.yamsVerbosity)
                    .withStatorCurrentLimit(CURRENT_LIMIT)
                    .withMotorInverted(MOTOR_INVERTED)
                    .withClosedLoopRampRate(RAMP_RATE)
                    .withOpenLoopRampRate(RAMP_RATE)
                    .withFeedforward(FEED_FORWARD)
                    .withSimFeedforward(FEED_FORWARD)
                    .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController                          motor           = new SparkWrapper(intakeMotor, DCMotor.getNEO(1), motorConfig); /// The new Smart Motor Controller
    private final FlyWheelConfig                                intakeConfig   = new FlyWheelConfig(motor) /// The FlyWheel config for the Intake.
                    .withDiameter(INDEXER_DIAMETER)
                    .withMass(Pounds.of(1)) // Intake Doesn't need to specify weight.
                    .withTelemetry("Intake", Telemetry.telemetryVerbosity.yamsVerbosity)
                    .withSoftLimit(INDEXER_MAX_SPEED.unaryMinus(), INDEXER_MAX_SPEED)
                    .withSpeedometerSimulation(INDEXER_MAX_SPEED);
    @Getter
    private final FlyWheel                                      intake          = new FlyWheel(intakeConfig); /// The final FlyWheel Mechanism to use as the smart Intake.

    @Getter
    private final IntakeSimulation                              intakeSim;
   
    /// Reports the current Intake state. To be used later in code and telemetry.
    public static class IntakeState {
        @Setter
        public static AngularVelocity              VelocityTolerance  = VELOCITY_TOLERANCE;
        @Setter
        public static AngularVelocity              Velocity           = DegreesPerSecond.of(0);
        @Setter
        public static AngularVelocity              TargetVelocity     = TARGET_VELOCITY;
        @Setter
        public static Trigger                      IsReady            = new Trigger(() -> false);
    }

    /**
     *
     * @param swerve only used in constructor once for driveSim.
     */
    public IntakeSubsystem(SwerveSubsystem swerve) {
        // If in sim
        if (RobotBase.isSimulation()) {
            // Create our intake at runtime.
            intakeSim = IntakeSimulation.InTheFrameIntake( /// The MapleSim Intake Simulation.
                    "Fuel",
                    swerve.getSwerveDrive().getMapleSimDrive().get(), // We know the driveSim exists.
                    Inches.of(12),
                    IntakeSimulation.IntakeSide.FRONT,
                    35);
            // Register our intake sim for updates.
            intakeSim.register();
        } else {
            intakeSim = null; // Doesn't like not being constructed, we'll just make it null.
        }
    }

    /**
     * Resets the setters to default values.
     */
    public void resetSetters() {
        IntakeState.setTargetVelocity(TARGET_VELOCITY);
    }
    
    public Command runSimIntake() {
        return Commands.startRun(intakeSim::startIntake, // Start our intake at once.
                () -> {
                    if (INFINITE_SIM_INTAKE) { // If Infinite GOBBLE
                        intakeSim.setGamePiecesCount(MAX_FUEL_CAPACITY - 3); // Hold only enough to keep collecting while still being able to score.
                    }
                } // Finally, stop our intake.
                ).finallyDo(intakeSim::stopIntake);
    }

    /**
     * Runs the intake at the {@link frc.robot.subsystems.IntakeSubsystem.IntakeState} TargetVelocity.
     * 
     * @return a command to run the intake at commanded velocity.
     */
    public Command runIntake() {
        System.out.println("***** RUNNING INTAKE *****");
        return Robot.isReal() // If real run the intake
                ? intake.setSpeed(TargetVelocity).finallyDo(intakeSim::stopIntake)
                : runSimIntake(); // If in sim run the sim intake.
    }

    /**
     * Gets the YAMS Flywheel AKA our Intake.
     *
     * @return the {@link FlyWheel} Intake Mechanism.
     */
    public FlyWheel getIntakeMechanism() {
        return intake;
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the intake mechanism's telemetry data to the network tables.
        intake.updateTelemetry();
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        intake.simIterate();
    }
}
