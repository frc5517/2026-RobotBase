package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.systems.ScoringSystem;
import frc.robot.util.ZoneTrigger;
import frc.robot.util.borrowed.math.AllianceFlipUtil;
import frc.robot.util.borrowed.math.FieldConstants;
import lombok.*;
import lombok.experimental.Accessors;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.InputBuilder.InputSelections.*;
import static frc.robot.Telemetry.Publishers.Robot.*;

@SuppressWarnings("unused")
public class InputBuilder
{
    /// Our subsystems and systems to call to from RobotContainer
    private final Subsystems subsystems;
    private final ScoringSystem scoring;
    // Control chooser for dashboard
    private static final SendableChooser<InputSelections> inputSelector = new SendableChooser<>();
    // Control input devices
    private final CommandXboxController driverXbox;
    private final CommandXboxController operatorXbox;

    public InputBuilder(Subsystems subsystems, ScoringSystem scoring) {
        /// Stop the console spam from not all buttons showing up.
        DriverStation.silenceJoystickConnectionWarning(true);
        /// Initialize our subsystem record
        this.subsystems = subsystems;
        /// Initialize our system record
        this.scoring = scoring;
        /// Initialize Controllers here.
        this.driverXbox = new CommandXboxController(0);
        this.operatorXbox = new CommandXboxController(1);
        /// Initialize input publisher
        Telemetry.Publishers.Robot.inputPublisher.accept(() -> inputSelector);
    }

    /**
     * Method used to construct input streams
     */
    public void configureBindings() {
        // Stop all movement.
        DISABLED.isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(subsystems.swerve::removeDefaultCommand));
        /// Testing control gets randomly changed all the time.
        final InputStream testing = new InputStream().
                StartingMethods
                .defaultXboxDrive(TESTING.isMode,   driverXbox)
                .FlyWheelBindings
                .withSetVelocity(driverXbox.rightTrigger(), RPM.of(3250))
                .back().SmartBindings
                .withIntakeIntoHopper(driverXbox.leftBumper())
                .withIndexIntoFlyWheel(driverXbox.rightBumper())
                .back();
        final InputStream sotmCalbration = new InputStream(SOTM_CALIBRATION.isMode)
                .SmartBindings
                .withSOTMCalibration(driverXbox.rightTrigger(), driverXbox.a(), Amps.of(60))
                .withIntakeIntoHopper(driverXbox.leftTrigger())
                .back().KickerBindings
                .withRunKicker(driverXbox.rightBumper().toggleOnTrue(Commands.none()), true)
                .back().HoodBindings
                .back().FlyWheelBindings
                .back();
        /// Single xbox controller with manual duty cycle control of all the subsystems.
        final InputStream singleManual = new InputStream(MANUAL_CONTROL.isMode)
                // A starting method should be set first.
                //.StartingMethods
                //.defaultXboxDrive(MANUAL_CONTROL.isMode, driverXbox)
                // Now we can set .with our controller bindings.
                .SwerveBindings /// Swerve Controller Bindings.
                // When in the subsystem, set our desired constants with .set methods.
                .setNormalRotation(.2)
                .setNormalTranslation(.2)
                .setSlowRotation(.4)
                .setSlowTranslation(.4)
                .setBoostRotation(1)
                .setBoostTranslation(1)
                // After any changes are made, we can then use .with methods to bind actions.
                //.withSlowDrive(         driverXbox.b())
                //.withToggleCentricity(  driverXbox.back(), true)
                //.withResetSimOdometry(  driverXbox.start())
                .back().IntakeBindings /// Intake Controller Bindings.
                .setIntakeSpeed(0.75)
                .withRunIntake(         driverXbox.leftBumper(), true)
                .withRunIntake(         driverXbox.rightBumper(), false)
                .back().IndexerBindings /// Indexer Controller Bindings
                .setIndexSpeed(0.75)
                .withRunIndexer(        driverXbox.x(), true)
                .withRunIndexer(        driverXbox.y(), false)
                .back().KickerBindings /// Kicker Controller Bindings
                .setKickerSpeed(.75)
                .withRunKicker(         driverXbox.a(), true)
                .withRunKicker(         driverXbox.b(), false)
                .back().TurretBindings /// Turret Controller Bindings
                .setTurretSpeed(0.5)
                .withRunTurret(         driverXbox.pov(90), true)
                .withRunTurret(         driverXbox.pov(270), false)
                .back().HoodBindings /// Hood Controller Bindings
                .setHoodSpeed(0.15)
                .withSetAngle(driverXbox.pov(0), Degrees.of(15))
                .withSetAngle(driverXbox.pov(180), Degrees.of(30))
                //.withRunHood(           driverXbox.pov(0), true)
                //.withRunHood(           driverXbox.pov(180), false)
                .back().FlyWheelBindings /// Turret Controller Bindings
                .setFlyWheelSpeed(1)
                .withRunFlyWheel(         driverXbox.rightTrigger(), true)
                .withRunFlyWheel(         driverXbox.leftTrigger(), false)
                .back().FlyWheelBindings
                .back().MiscBindings /// Miscellaneous Controller Bindings
                .resetField(            driverXbox.start())
                .back(); // Return to our InputStream.
        /// Single xbox controller with manual PID control of all the subsystems.
        final InputStream singlePID = new InputStream()
                // A starting method should be set first.
                .StartingMethods
                .headingXboxDrive(PID_CONTROL.isMode, driverXbox)
                // Now we can set .with our controller bindings.
                .SwerveBindings /// Swerve Controller Bindings.
                // When in the subsystem, set our desired constants with .set methods.
                .setNormalRotation(.8)
                .setNormalTranslation(.8)
                .setSlowRotation(.4)
                .setSlowTranslation(.4)
                .setBoostRotation(1)
                .setBoostTranslation(1)
                // After any changes are made, we can then use .with methods to bind actions.
                .withSlowDrive(         driverXbox.b())
                .withToggleCentricity(  driverXbox.back(), true)
                .withResetSimOdometry(  driverXbox.start())
                .back().IntakeBindings /// Intake Controller Bindings.
                .withRunIntake(         driverXbox.leftBumper(), true)
                .withRunIntake(         driverXbox.rightBumper(), false)
                .back().IndexerBindings /// Indexer Controller Bindings
                .withRunIndexer(        driverXbox.x(), true)
                .withRunIndexer(        driverXbox.y(), false)
                .back().KickerBindings /// Kicker Controller Bindings
                .withRunKicker(         driverXbox.a(), true)
                .back().TurretBindings /// Turret Controller Bindings
                //.withDefaultCommand(() -> subsystems.turret.getTurret().setAngle(Degrees.of(MathUtil.interpolate(TurretSubsystem.HardwareConstants.HARD_LIMIT_REVERSE.in(Degrees), TurretSubsystem.HardwareConstants.HARD_LIMIT_FORWARD.in(Degrees), 0.5))))
                .withSetAngle(          driverXbox.pov(90), Degrees.of(MathUtil.interpolate(TurretSubsystem.HardwareConstants.HARD_LIMIT_REVERSE.in(Degrees), TurretSubsystem.HardwareConstants.HARD_LIMIT_FORWARD.in(Degrees), 0.25)))
                .withSetAngle(          driverXbox.pov(270), Degrees.of(MathUtil.interpolate(TurretSubsystem.HardwareConstants.HARD_LIMIT_REVERSE.in(Degrees), TurretSubsystem.HardwareConstants.HARD_LIMIT_FORWARD.in(Degrees), 0.75)))
                .back().HoodBindings /// Hood Controller Bindings
                .withDefaultCommand(() -> subsystems.hood.getHood().setAngle(Degrees.of(MathUtil.interpolate(HoodSubsystem.HardwareConstants.HARD_LIMIT_REVERSE.in(Degrees), HoodSubsystem.HardwareConstants.HARD_LIMIT_FORWARD.in(Degrees), 0.5))))
                .withSetAngle(          driverXbox.pov(0), Degrees.of(MathUtil.interpolate(HoodSubsystem.HardwareConstants.HARD_LIMIT_REVERSE.in(Degrees), HoodSubsystem.HardwareConstants.HARD_LIMIT_FORWARD.in(Degrees), 0.25)))
                .withSetAngle(          driverXbox.pov(180), Degrees.of(MathUtil.interpolate(HoodSubsystem.HardwareConstants.HARD_LIMIT_REVERSE.in(Degrees), HoodSubsystem.HardwareConstants.HARD_LIMIT_FORWARD.in(Degrees), 0.75)))
                .back().FlyWheelBindings /// Turret Controller Bindings
                .withRunFlyWheel(         driverXbox.rightTrigger(), true)
                .withRunFlyWheel(         driverXbox.leftTrigger(), false)
                .back().FlyWheelBindings
                .back().MiscBindings /// Miscellaneous Controller Bindings
                .resetField(            driverXbox.start())
                .back(); // Return to our InputStream.

        final InputStream bradStream =  new InputStream()
                .StartingMethods.headingXboxDrive(BRAD_CONTROL.isMode, driverXbox)
                .SmartBindings
                .withShootOnTheMove(driverXbox.b())
                .back()
                .SwerveBindings
                .setSlowTranslation(0.5)
                .setSlowRotation(0.6)
                .setNormalTranslation(0.7)
                .setNormalRotation(0.8)
                .setBoostTranslation(1)
                .setBoostRotation(1)
                .setDeadzone(0.05)
                .withSlowDrive(driverXbox.leftBumper())
                .withBoostDrive(driverXbox.rightBumper())
                .withResetSimOdometry(driverXbox.start())
                .withToggleCentricity(driverXbox.back(), true)
                .withToggleZones(CustomTriggers.enteringBumpZone.getTrigger())
                .withToggleZones(driverXbox.a())
                .withLookAtHubThenFire(driverXbox.y())
                .back().IntakeBindings
                .setIntakeSpeed(0.75)
                .withRunIntake(driverXbox.leftTrigger(), true)
                .back().MiscBindings
                .resetField(driverXbox.start())
                .back();
    }

    // Control binding type enum
    public enum InputSelections {
        /// Default Input Schema
        DISABLED("DISABLED - Temp", true),
        BRAD_CONTROL("Brad Xbox", false),
        SINGLE_XBOX("Single Xbox - NC"),
        TESTING("Testing", false),
        SOTM_CALIBRATION("SOTM Calibration", false),
        MANUAL_CONTROL("Manual Control", false),
        PID_CONTROL("Basic PID Control", false),

        /**  Define Student Input Selections here  */

        NEW_INPUT("New Student Bindings");

        // BindType Name
        public final String name;
        public final Trigger isMode;

        /**
         * Constructor for BindingType
         * @param name The name of the control type, used in publishing.
         * @param isDefault whether to make this the default input. There can only be one. No extra handling is done.
         */
        InputSelections(String name, boolean isDefault) {
            this.name = name;
            this.isMode = new Trigger(() -> inputSelector.getSelected() == this);
            if (isDefault) {
                inputSelector.setDefaultOption(name, this);
            } else {
                inputSelector.addOption(name, this);
            }
        }

        /**
         * Constructor for BindingType
         * @param name The name of the control type, used in publishing.
         */
        InputSelections(String name) {
            this(name, false);
        }
    }

    /**
     *
     *  A class that holds various triggers for control logic.
     *
     */
    public static class CustomTriggers {
        public static ZoneTrigger allianceZone = new ZoneTrigger("Alliance",
                Pair.of(new Translation2d(0, 0), new Translation2d(4, 8)));
        public static ZoneTrigger neutralZone = new ZoneTrigger("Neutral",
                Pair.of(new Translation2d(5.25, 0), new Translation2d(9, 8)));
        public static ZoneTrigger enteringBumpZone = new ZoneTrigger("Entering Bump",
                Pair.of(new Translation2d(3.25, 5), new Translation2d(6, 6.25)),
                Pair.of(new Translation2d(3.25, 1.75), new Translation2d(6, 3)));
        public static ZoneTrigger bumpZone = new ZoneTrigger("Bump",
                Pair.of(new Translation2d(3.75, 1.5), new Translation2d(5.5, 3.5)),
                Pair.of(new Translation2d(3.75, 4.5), new Translation2d(5.5, 6.5)),
                Pair.of(new Translation2d(11, 4.5), new Translation2d(12.75, 6.5)),
                Pair.of(new Translation2d(11, 1.5), new Translation2d(12.75, 3.5)));

        public static ZoneTrigger aimingZone = new ZoneTrigger("Aiming",
                Pair.of(new Translation2d(1.5, 0.5), new Translation2d(4.2, 7.5)));

        public static ZoneTrigger scoringZone = new ZoneTrigger("Scoring",
                Pair.of(new Translation2d(1.5, 1), new Translation2d(2.5, 7)));
    }

    /**
     *
     *  Input Stream Class
     *
     *
     */
    @Accessors(chain = true)
    private class InputStream {
        /**
         * Trigger used to determine when this InputStream is in control.
         */
        @Setter private Trigger isMode;
        private Trigger onChange;

        /// Inner Config Namespaces
        public final StartingMethods StartingMethods = new StartingMethods();
        public final SmartBindings SmartBindings = new SmartBindings();
        public final IntakeBindings IntakeBindings = new IntakeBindings();
        public final IndexerBindings IndexerBindings = new IndexerBindings();
        public final KickerBindings KickerBindings = new KickerBindings();
        public final FlyWheelBindings FlyWheelBindings = new FlyWheelBindings();
        public final HoodBindings HoodBindings = new HoodBindings();
        public final TurretBindings TurretBindings = new TurretBindings();
        public final SwerveBindings SwerveBindings = new SwerveBindings();
        public final MiscBindings MiscBindings = new MiscBindings();

        private SwerveInputStream swerveInputStream;

        /// Initialize our binding options only when the subsystem is not null.
        InputStream() {}

        /// Default Constructor with no drive.
        InputStream(Trigger isMode) {
            this();
            this.isMode = isMode;
        }

        /// Default Basic Drive Constructor
        InputStream(Trigger isMode,
                    DoubleSupplier x,
                    DoubleSupplier y) {
            this(isMode);
            this.swerveInputStream = SwerveInputStream.of(
                    subsystems.swerve.getSwerveDrive(), x, y) // Make the input stream.
                    .cubeTranslationControllerAxis(true)
                    .cubeRotationControllerAxis(true)
                    .scaleTranslation(SwerveBindings.normalTranslation)
                    .scaleRotation(SwerveBindings.normalRotation)
                    .deadband(SwerveBindings.deadzone)
                    .robotRelative(true)
                    .allianceRelativeControl(false);
            this.SwerveBindings.withDefaultCommand(() -> subsystems.swerve.driveFieldOriented(() -> swerveInputStream.get()));
        }

        /**
         * Basic Control Methods to help get started.
         */
        private class StartingMethods {
            /**
             * Default Xbox Drive Constructor with regular rotation.
             * WARNING: Creates a new stream, DO NOT use inline.
             *
             * @param isMode The trigger telling the stream we are in the Input Selection.
             * @param driverXbox the {@link CommandXboxController} to bind to.
             */
            public InputStream defaultXboxDrive(Trigger isMode, CommandXboxController driverXbox) {
                // Load default drive constructor.
                var xboxDrive = new InputStream(isMode,
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1);
                // Set heading drive.
                xboxDrive.swerveInputStream.withControllerRotationAxis(() -> driverXbox.getRightX() * -1);
                // Return our new stream.
                return xboxDrive;
            }

            /**
             * Default Xbox Drive Constructor with heading rotation.
             * WARNING: Creates a new stream, DO NOT use inline.
             *
             * @param isMode The trigger telling the stream we are in the Input Selection.
             * @param driverXbox the {@link CommandXboxController} to bind to.
             */
            public InputStream headingXboxDrive(Trigger isMode, CommandXboxController driverXbox) {
                // Load default drive constructor.
                var headingDrive = new InputStream(isMode,
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1);
                // Set heading rotation.
                headingDrive.swerveInputStream
                        .cubeRotationControllerAxis(false)
                        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
                        .headingWhile(true);
                // Return our new stream.
                return headingDrive;
            }
        }

        /// ***** Auto Bindings ***** ///

        /**
         * Smart controls that make use of automatic sequencing controls.
         */
        public class SmartBindings {

            public SmartBindings withSOTMCalibration(Trigger indexIntoFlywheel, Trigger ballHasHitFloor, Current threshold) {
                final Timer timeOfFlightTimer = new Timer();
                final List<Double> dataLog =  new ArrayList<>();

                this.withIndexIntoFlyWheel(indexIntoFlywheel);
                isMode.and(indexIntoFlywheel).and(subsystems.flywheel.currentSensorTrigger(threshold, Seconds.of(0)))
                        .onTrue(Commands.runOnce(timeOfFlightTimer::start).andThen(Commands.waitUntil(ballHasHitFloor).andThen(
                                () -> {
                                    timeOfFlightTimer.stop();
                                    final Time tofGuess = Seconds.of(timeOfFlightTimer.get());
                                    final Angle hoodAngle = subsystems.hood.getHood().getAngle();
                                    final AngularVelocity flyWheelSpeed = subsystems.flywheel.getFlyWheel().getSpeed();
                                    timeOfFlightTimer.reset();

                                    dataLog.add(tofGuess.in(Seconds));
                                    dataLog.add(hoodAngle.in(Degrees));
                                    dataLog.add(flyWheelSpeed.in(RPM));

                                    DriverStation.reportWarning("SOTM Data - ToF: " + tofGuess + " hA: " + hoodAngle.in(Degrees) + " fV: " + flyWheelSpeed.in(RPM), false);
                                })));
                isMode.and(() -> !dataLog.isEmpty()).and(DriverStation::isDisabled).onTrue(Commands.runOnce(() -> DriverStation.reportWarning("SOTM DataLog: " + dataLog, false)).ignoringDisable(true));
                return this;
            }

            /**
             * Shoots while moving using double interpolating maps while calculating for ToF, Phase Delay, Velocity, ect.
             * Active while inside a preset zone while the hub is active.
             *
             * @return this, for chaining.
             */
            public SmartBindings withAutoShootOnTheMove() {
                if (!TurretBindings.isPresent || !HoodBindings.isPresent || !FlyWheelBindings.isPresent) {return this;}
                withShootOnTheMove(isMode.and(CustomTriggers.aimingZone.getTrigger()).and(Telemetry.FMSTriggers.isHubActive));
                return this;
            }

            /**
             * Shoots while moving using double interpolating maps while calculating for ToF, Phase Delay, Velocity, ect.
             *
             * @param shootOnTheMoveWhile the button to map.
             * @return this, for chaining.
             */
            public SmartBindings withShootOnTheMove(Trigger shootOnTheMoveWhile) {
                if (!TurretBindings.isPresent || !HoodBindings.isPresent || !FlyWheelBindings.isPresent) {return this;}
                isMode.and(shootOnTheMoveWhile).whileTrue(scoring.shootOnTheMove());
                return this;
            }

            /**
             * Runs all PID Subsystems to various goals.
             * Set any as null if you don't want it to move.
             *
             * @param runAll the button to map.
             * @param hoodAngle the hood angle to set.
             * @param turretAngle the turret angle to set.
             * @param flyWheelSpeed the flywheel speed to run.
             * @return this, for chaining.
             */
            public SmartBindings withRunAll(
                    Trigger runAll,
                    Supplier<Angle> hoodAngle,
                    Supplier<Angle> turretAngle,
                    Supplier<AngularVelocity> flyWheelSpeed) {
                if (!TurretBindings.isPresent || !HoodBindings.isPresent || !FlyWheelBindings.isPresent) {return this;}
                isMode.and(runAll).whileTrue(
                        subsystems.hood.getHood().run(hoodAngle).onlyIf(() -> hoodAngle != null)
                                .alongWith(subsystems.turret.getTurret().run(turretAngle).onlyIf(() -> turretAngle != null))
                                .alongWith(subsystems.flywheel.getFlyWheel().run(flyWheelSpeed).onlyIf(() -> flyWheelSpeed != null)));
                return this;
            }

            /**
             * Runs intake in while running the kicker and indexer in reverse. This should help fill the hopper up.
             *
             * @param intake the button to map.
             * @return this, for chaining.
             */
            public SmartBindings withIntakeIntoHopper(Trigger intake) {
                if (!IndexerBindings.isPresent || !IntakeBindings.isPresent || !KickerBindings.isPresent) {return this;}
                // Runs intake in while running the kicker and indexer in reverse. This should help fill the hopper up.
                isMode.and(intake).whileTrue(subsystems.intake.intake(IntakeBindings.intakeSpeed, true)
                                .alongWith(subsystems.indexer().runIndexer(IndexerBindings.indexSpeed, false))
                                .alongWith(subsystems.kicker.runKicker(KickerBindings.kickerSpeed, false)));
                return this;
            }

            /**
             * THIS DOES NOT RUN THE FLYWHEEL
             * This indexes the hopper into the shooter.
             * Runs the indexer in and kicker in.
             * Intake does not run.
             *
             * @param index the button to map.
             * @return this, for chaining.
             */
            public SmartBindings withIndexIntoFlyWheel(Trigger index) {
                if (!IndexerBindings.isPresent || !IntakeBindings.isPresent || !KickerBindings.isPresent) {return this;}
                isMode.and(index).whileTrue(subsystems.indexer().runIndexer(IndexerBindings.indexSpeed, true)
                                .alongWith(subsystems.kicker.runKicker(KickerBindings.kickerSpeed, true)));
                return this;
            }

            /**
             * Turns all spinners on to take a fuel from the intake and fire it right away.
             * This is for manual testing
             * <p>
             * Starts Flywheel and kicker, waits a second.
             * Then it turns on the indexer and intake.
             *
             * @param turnOnAll the button to map.
             * @return this, for chaining
             */
            public SmartBindings withDutyCycleAll(Trigger turnOnAll) {
                if (!IntakeBindings.isPresent || !IndexerBindings.isPresent || !KickerBindings.isPresent || !FlyWheelBindings.isPresent) {return this;}
                isMode.and(turnOnAll).whileTrue(
                        subsystems.flywheel.runFlyWheel(FlyWheelBindings.flyWheelSpeed, true)
                                .alongWith(subsystems.kicker.runKicker(KickerBindings.kickerSpeed,  true))
                                .alongWith(
                                        Commands.waitSeconds(1)
                                                .alongWith(subsystems.indexer.runIndexer(IndexerBindings.indexSpeed,  true))
                                                .alongWith(subsystems.intake.intake(IntakeBindings.intakeSpeed, true))));
                return this;
            }

            /**
             * Leaves SmartBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }


        /// ***** Separate Bindings ***** ///

        /**
         * Intake button bindings.
         */
        private class IntakeBindings {
            /**
             * Checks if this subsystem is present, if not, don't bind anything.
             */
            private final boolean isPresent;
            /**
             * The default duty cycle speed to run at.
             */
            @Setter private double intakeSpeed = 0.5;

            private IntakeBindings() {
                this.isPresent = subsystems.intake != null;
            }

            /**
             * Runs the intake at preset speed, stopping when finished.
             *
             * @param runIntake then button to map.
             * @param isIn whether to spin in or out.
             * @return this, for chaining.
             */
            public IntakeBindings withRunIntake(Trigger runIntake, boolean isIn) {
                if (!isPresent) {return this;}
                isMode.and(runIntake).whileTrue(subsystems.intake.intake(intakeSpeed, isIn))
                        .onFalse(subsystems.intake.stopIntake());
                return this;
            }

            /**
             * Sets a default command for this subsystem.
             *
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public IntakeBindings withDefaultCommand(Supplier<Command> defaultCommand) {
                if (!isPresent) {return this;}
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.intake.setDefaultCommand(defaultCommand.get())));
                return this;
            }

            /**
             * Leaves IntakeBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }


        private class IndexerBindings {
            /**
             * Checks if this subsystem is present, if not, don't bind anything.
             */
            private final boolean isPresent;
            /**
             * The default duty cycle speed to run at.
             */
            @Setter private double indexSpeed = 0.75;


            private IndexerBindings() {
                this.isPresent = subsystems.indexer != null;
            }

            /**
             * Runs the indexer at preset speed, stopping when finished.
             *
             * @param runIndexer then button to map.
             * @param isIn whether to spin in or out.
             * @return this, for chaining.
             */
            public IndexerBindings withRunIndexer(Trigger runIndexer, boolean isIn) {
                if (!isPresent) {return this;}
                isMode.and(runIndexer).whileTrue(subsystems.indexer.runIndexer(indexSpeed, isIn))
                        .onFalse(subsystems.indexer.stopIndexer());
                return this;
            }

            /**
             * Sets a default command for this subsystem.
             *
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public IndexerBindings withDefaultCommand(Supplier<Command> defaultCommand) {
                if (!isPresent) {return this;}
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.indexer.setDefaultCommand(defaultCommand.get())));
                return this;
            }

            /**
             * Leaves IndexerBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }

        private class KickerBindings {
            /**
             * Checks if this subsystem is present, if not, don't bind anything.
             */
            private final boolean isPresent;

            private KickerBindings() {
                this.isPresent = subsystems.kicker != null;
            }
            /**
             * The default duty cycle speed to run at.
             */
            @Setter private double kickerSpeed = 0.75;

            /**
             * Runs the kicker at preset speed, stopping when finished.
             *
             * @param runkicker then button to map.
             * @param isOut whether to spin out.
             * @return this, for chaining.
             */
            public KickerBindings withRunKicker(Trigger runkicker, boolean isOut) {
                if (!isPresent) {return this;}
                isMode.and(runkicker).whileTrue(subsystems.kicker.runKicker(kickerSpeed, isOut))
                        .onFalse(subsystems.kicker.stopKicker());
                return this;
            }

            /**
             * Sets a default command for this subsystem.
             *
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public KickerBindings withDefaultCommand(Supplier<Command> defaultCommand) {
                if (!isPresent) {return this;}
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.kicker.setDefaultCommand(defaultCommand.get())));
                return this;
            }

            /**
             * Leaves KickerBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }

        /// ***** Scoring Mechanisms ***** ///

        /**
         * FlyWheel Button Bindings
         */
        private class FlyWheelBindings {
            /**
             * Checks if this subsystem is present, if not, don't bind anything.
             */
            private final boolean isPresent;
            /**
             * The default duty cycle speed to run at.
             */
            @Setter private double flyWheelSpeed = 0.75;

            private FlyWheelBindings() {
                this.isPresent = subsystems.flywheel != null;
            }

            /**
             * Sets the flywheel velocity goal.
             *
             * @param setVelocity the button to map.
             * @param flywheelVelocity the velocity goal.
             * @return this, for chaining.
             */
            public FlyWheelBindings withSetVelocity(Trigger setVelocity, AngularVelocity flywheelVelocity) {
                if (!isPresent) {return this;}
                isMode.and(setVelocity).whileTrue(subsystems.flywheel.getFlyWheel().run(flywheelVelocity));
                return this;
            }

            /**
             * Runs the flywheel at preset speed, stopping when finished.
             *
             * @param runFlyWheel then button to map.
             * @param isOut whether to spin out.
             * @return this, for chaining.
             */
            public FlyWheelBindings withRunFlyWheel(Trigger runFlyWheel, boolean isOut) {
                if (!isPresent) {return this;}
                isMode.and(runFlyWheel).whileTrue(subsystems.flywheel.runFlyWheel(flyWheelSpeed, isOut))
                        .onFalse(subsystems.flywheel.stopFlyWheel());
                return this;
            }

            /**
             * Simulates the turret shooting fuel.
             *
             * @param shoot the button to map.
             * @return this, for chaining.
             */
            public FlyWheelBindings withSimShoot(Trigger shoot) {
                if (!isPresent) {return this;}
                isMode.and(shoot).whileTrue(subsystems.flywheel.simShoot(subsystems).andThen(Commands.waitSeconds(.25)).repeatedly());
                return this;
            }

            /**
             * Sets a default command for this subsystem.
             *
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public FlyWheelBindings withDefaultCommand(Supplier<Command> defaultCommand) {
                if (!isPresent) {return this;}
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.flywheel.setDefaultCommand(defaultCommand.get())));
                return this;
            }

            /**
             * Leaves FlyWheelBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }

        /**
         * Hood Button Bindings
         */
        private class HoodBindings {
            /**
             * Checks if this subsystem is present, if not, don't bind anything.
             */
            private final boolean isPresent;
            /**
             * The default duty cycle speed to run at.
             */
            @Setter private double hoodSpeed = 0.25;

            private HoodBindings() {
                this.isPresent = subsystems.hood != null;
            }

            /**
             * Sets the hood angle to the given angle.
             * Horizontal to the floor is 0 degrees.
             *w
             * @param setAngle the button to map.
             * @param hoodAngle the angle to move to.
             * @return this, for chaining.
             */
            public HoodBindings withSetAngle(Trigger setAngle, Angle hoodAngle) {
                if (!isPresent) {return this;}
                isMode.and(setAngle).whileTrue(subsystems.hood.getHood().setAngle(hoodAngle));
                return this;
            }

            /**
             * Runs the hood at preset speed, stopping when finished.
             *
             * @param runHood the button to map.
             * @param isUp whether to go up.
             * @return this, for chaining.
             */
            public HoodBindings withRunHood(Trigger runHood, boolean isUp) {
                if (!isPresent) {return this;}
                isMode.and(runHood).whileTrue(subsystems.hood.runHood(hoodSpeed, isUp))
                        .onFalse(subsystems.hood.stopHood());
                return this;
            }

            /**
             * Sets a default command for this subsystem.
             *
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public HoodBindings withDefaultCommand(Supplier<Command> defaultCommand) {
                if (!isPresent) {return this;}
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.hood.setDefaultCommand(defaultCommand.get())));
                return this;
            }

            /**
             * Leaves HoodBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }

        /**
         * Turret Button Bindings
         */
        private class TurretBindings {
            /**
             * Checks if this subsystem is present, if not, don't bind anything.
             */
            private final boolean isPresent;
            /**
             * The default duty cycle speed to run at.
             */
            @Setter private double turretSpeed = .25;

            private TurretBindings() {
                this.isPresent = subsystems.turret != null;
            }

            /**
             * Sets the turret angle to the given angle.
             * Horizontal to the floor is 0 degrees.
             *w
             * @param setAngle the button to map.
             * @param turretAngle the angle to move to.
             * @return this, for chaining.
             */
            public TurretBindings withSetAngle(Trigger setAngle, Angle turretAngle) {
                if (!isPresent) {return this;}
                isMode.and(setAngle).whileTrue(subsystems.turret.getTurret().setAngle(turretAngle));
                return this;
            }

            /**
             * Runs the turret at preset speed, stopping when finished.
             *
             * @param runTurret then button to map.
             * @param isCCW whether to turn CCW, aka lefty loosey.
             * @return this, for chaining.
             */
            public TurretBindings withRunTurret(Trigger runTurret, boolean isCCW) {
                if (!isPresent) {return this;}
                isMode.and(runTurret).whileTrue(subsystems.turret.runTurret(turretSpeed, isCCW))
                        .onFalse(subsystems.turret.stopTurret());
                return this;
            }

            /**
             * Sets a default command for this subsystem.
             *
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public TurretBindings withDefaultCommand(Supplier<Command> defaultCommand) {
                if (!isPresent) {return this;}
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.turret.setDefaultCommand(defaultCommand.get())));
                return this;
            }

            /**
             * Leaves TurretBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }

        /**
         * Swerve Drive Button Bindings
         */
        private class SwerveBindings {
            /**
             * Controller Dead Zone
             */
            @Setter private double deadzone = 0.01;
            /**
             * Move Speed Scalar when slowing drive speed.
             */
            @Setter private double slowTranslation = 0.3;
            /**
             * Rotation Speed Scalar when slowing drive speed.
             */
            @Setter private double slowRotation = 0.4;
            /**
             * Move Speed Scalar when driving normally.
             */
            @Setter private double normalTranslation = .6;
            /**
             * Rotation Speed Scalar when driving normally.
             */
            @Setter private double normalRotation = .6;
            /**
             * Move Speed Scalar when boosting drive speed.
             */
            @Setter private double boostTranslation = 0.8;
            /**
             * Rotation Speed Scalar when boosting drive speed.
             */
            @Setter private double boostRotation = 0.85;

            private final boolean isPresent;
            private SwerveBindings() {
                this.isPresent = subsystems.swerve != null;
            }
            /**
             * Changes inputs to the given input selection.
             */
            public SwerveBindings withToggleCentricity(Trigger toggleCentricity, boolean fieldDefault) {
                if (!isPresent) {return this;}
                // Set our default.
                swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault);
                // Toggle when pressed.
                isMode.and(toggleCentricity).toggleOnTrue(Commands.runEnd(
                        () -> swerveInputStream.robotRelative(fieldDefault).allianceRelativeControl(!fieldDefault),
                        () -> swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault)));
                return this;
            }

            /**
             * Automatically swap between the Neutral and Alliance zone crossing tbe bump.
             *
             * @param toggleZones the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withToggleZones(Trigger toggleZones) {
                if (!isPresent) {return this;}
                isMode.and(toggleZones).whileTrue(subsystems.swerve.toggleZones());
                return this;
            }

            /**
             * Theoretically uses vision to track fuel if present.
             *
             * @param collectFuel the button to map.
             * @return this for chaining.
             */
            public SwerveBindings withCollectFuel(Trigger collectFuel) {
                if (!isPresent || !SwerveSubsystem.ControlConstants.RUN_VISION) {return this;}
                //isMode.and(collectFuel).whileTrue(subsystems.swerve.driveToNearestFuel());
                return this;
            }

            /**
             * Looks at the hub with the drive and scores while moving. Doesn't account for distance.
             *
             * @param autoAimDriveThenFireWhile the button to map.
             * @param readyToScore additional {@link Trigger} to tell when it's ok to fire.
             * @return this, for chaining.
             */
            public SwerveBindings withLookAtHubThenFire(Trigger autoAimDriveThenFireWhile, Trigger readyToScore) {
                if (!isPresent || !FlyWheelBindings.isPresent) {return this;}
                // Aim at the hub with the drive
                this.withAimWhile(autoAimDriveThenFireWhile,
                        new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero),
                        new Trigger(() -> false),
                        new Trigger(() -> false)); // Leave look ahead time alone.
                // Score when aim is locked.
                this.back().FlyWheelBindings
                        .withSimShoot(isMode.and(readyToScore).and(swerveInputStream.aimLock(Degrees.of(1.5))));
                return this;
            }

            /**
             * Looks at the hub with the drive and scores while moving. Doesn't account for distance.
             *
             * @param autoAimDriveThenFireWhile the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withLookAtHubThenFire(Trigger autoAimDriveThenFireWhile) {
                if (!isPresent) {return this;}
                return withLookAtHubThenFire(autoAimDriveThenFireWhile, new Trigger(() -> true));
            }

            /**
             * Aims at a pose while held.
             *
             * @param aimWhile the button to map.
             * @param aimWhilePose the pose to aim at.
             * @param lookAheadUp increase look ahead time.
             * @param lookAheadDown decrease look ahead time.
             * @return this, for chaining.
             */
            public SwerveBindings withAimWhile(Trigger aimWhile, Pose2d aimWhilePose, Trigger lookAheadUp, Trigger lookAheadDown) {
                if (!isPresent) {return this;}
                // Update Telemetry Continuously
                isMode.and(DriverStation::isEnabled).whileTrue(Commands.run(() -> {
                    SmartDashboard.putBoolean("Telemetry/RobotTelemetry/Swerve/Drive is Aimed", swerveInputStream.aimLock(Degrees.of(1)).getAsBoolean());
                }));
                // Save an adjustable atomic time.
                final int[] lookAheadTime = {0};
                // Add 1s to lookAheadTime
                isMode.and(lookAheadUp).onTrue(Commands.runOnce(() -> {
                    lookAheadTime[0]++;
                    swerveInputStream.aimLookahead(Seconds.of(lookAheadTime[0]));
                }));
                // Remove 1s from lookAheadTime
                isMode.and(lookAheadDown).onTrue(Commands.runOnce(() -> {
                    lookAheadTime[0]--;
                    swerveInputStream.aimLookahead(Seconds.of(lookAheadTime[0]));
                }));
                // Aim while held
                isMode.and(aimWhile).onTrue(Commands.runOnce(() -> {
                    // Update our pose and aim supplier when isMode and aimWhile.
                    swerveInputStream.aim(AllianceFlipUtil.ifShouldFlip(aimWhilePose));
                    swerveInputStream.aimWhile(isMode.and(aimWhile));
                }));
                // Return this for chaining.
                return this;
            }

            /**
             * Holding the button slows the drive.
             *
             * @param slowDrive the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withSlowDrive(Trigger slowDrive) {
                if (!isPresent) {return this;}
                isMode.and(slowDrive)
                        .onTrue(Commands.runOnce(() -> swerveInputStream.scaleTranslation(slowTranslation).scaleRotation(slowRotation)))
                        .onFalse(Commands.runOnce(() -> swerveInputStream.scaleTranslation(normalTranslation).scaleRotation(normalRotation)));
                return this;
            }

            /**
             * Holding the button speeds up the drive.
             *
             * @param boostDrive the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withBoostDrive(Trigger boostDrive) {
                if (!isPresent) {return this;}
                isMode.and(boostDrive)
                        .onTrue(Commands.runOnce(() -> swerveInputStream.scaleTranslation(boostTranslation).scaleRotation(boostRotation)))
                        .onFalse(Commands.runOnce(() -> swerveInputStream.scaleTranslation(normalTranslation).scaleRotation(normalRotation)));
                return this;
            }

            /**
             * Resets the robot odometry. Zeros the gyro. Updates pose to real pose in sim.
             *
             * @param resetOdometry the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withResetSimOdometry(Trigger resetOdometry) {
                if (!isPresent) {return this;}
                isMode.and(resetOdometry).onTrue(Commands.runOnce(() -> subsystems.swerve.getSwerveDrive().resetOdometry(subsystems.swerve.getSwerveDrive().getSimulationDriveTrainPose().get())));
                return this;
            }

            /**
             * Sets a default command for this subsystem.
             *
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public SwerveBindings withDefaultCommand(Supplier<Command> defaultCommand) {
                if (!isPresent) {return this;}
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.swerve.setDefaultCommand(defaultCommand.get())));
                return this;
            }

            /**
             * Leaves SwerveBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                /// Update things here, also runs on binding entry.
                if (Objects.nonNull(swerveInputStream)) {
                    swerveInputStream
                            .scaleTranslation(normalTranslation)
                            .scaleRotation(normalRotation)
                            .deadband(deadzone);
                }
                return InputStream.this;
            }
        }

        /**
         * Various Bindings that don't fit into a category yet.
         */
        private class MiscBindings {
            /**
             * Resets the simulated field.
             *
             * @param resetField button to map.
             * @return this, for chaining.
             */
            public MiscBindings resetField(Trigger resetField) {
                isMode.and(RobotBase::isSimulation).and(resetField).onTrue(
                        Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto()));
                return this;
            }
            /**
             * Changes inputs to the given input selection.
             *
             * @param bindingType The input to change to.
             * @param changeInput button to map.
             * @return this, for chaining.
             */
            public MiscBindings withChangeInput(InputSelections bindingType, Trigger changeInput) {
                isMode.and(changeInput).onTrue(Commands.runOnce(() -> inputOverride.set(bindingType.name)));
                return this;
            }

            /**
             * Leaves MiscBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }
    }

    /**
     * Neatly packs our subsystems into a bite size struct.
     */
    public record Subsystems(
            SwerveSubsystem swerve,
            FlyWheelSubsystem flywheel,
            HoodSubsystem hood,
            IndexerSubsystem indexer,
            IntakeSubsystem intake,
            KickerSubsystem kicker,
            TurretSubsystem turret) {}
}