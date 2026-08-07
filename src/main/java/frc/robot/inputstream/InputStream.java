//package frc.robot.inputstream;
//
//import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.AngularVelocity;
//import edu.wpi.first.units.measure.Current;
//import edu.wpi.first.units.measure.Time;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.InputBuilder;
//import frc.robot.inputstream.subsystems.SwerveBindings;
//import frc.robot.systems.ScoringSystem;
//import lombok.Getter;
//import lombok.Setter;
//import lombok.experimental.Accessors;
//import swervelib.SwerveInputStream;
//
//import java.util.ArrayList;
//import java.util.List;
//import java.util.function.DoubleSupplier;
//import java.util.function.Supplier;
//
//import static edu.wpi.first.units.Units.*;
//import static edu.wpi.first.units.Units.Degrees;
//import static edu.wpi.first.units.Units.RPM;
//import static edu.wpi.first.units.Units.Seconds;
//import static frc.robot.RobotContainer.subsystems;
//
///**
// *
// * Input Stream Class
// *
// *
// */
//@Accessors(chain = true)
//public class InputStream {
//    /**
//     * Trigger used to determine when this InputStream is in control.
//     */
//    @Setter @Getter
//    private Trigger isMode;
//    private Trigger onChange; //TODO
//
//    /// Inner Config Namespaces
//    public final InputBuilder.InputStream.StartingMethods StartingMethods = new InputBuilder.InputStream.StartingMethods();
//    public final InputBuilder.InputStream.SmartBindings SmartBindings = new InputBuilder.InputStream.SmartBindings();
//    public final InputBuilder.InputStream.IntakeBindings IntakeBindings = new InputBuilder.InputStream.IntakeBindings();
//    public final InputBuilder.InputStream.IndexerBindings IndexerBindings = new InputBuilder.InputStream.IndexerBindings();
//    public final InputBuilder.InputStream.AgitatorBindings AgitatorBindings = new InputBuilder.InputStream.AgitatorBindings();
//    public final InputBuilder.InputStream.KickerBindings KickerBindings = new InputBuilder.InputStream.KickerBindings();
//    public final InputBuilder.InputStream.FlyWheelBindings FlyWheelBindings = new InputBuilder.InputStream.FlyWheelBindings();
//    public final InputBuilder.InputStream.HoodBindings HoodBindings = new InputBuilder.InputStream.HoodBindings();
//    public final InputBuilder.InputStream.TurretBindings TurretBindings = new InputBuilder.InputStream.TurretBindings();
//    public final SwerveBindings SwerveBindings = new SwerveBindings(this);
//    public final InputBuilder.InputStream.MiscBindings MiscBindings = new InputBuilder.InputStream.MiscBindings();
//
//    /** Gets the YAGSL SwerveInputStream */
//    @Getter
//    private SwerveInputStream swerveInputStream;
//
//    /// Initialize our binding options only when the subsystem is not null.
//    InputStream() {
//    }
//
//    /// Default Constructor with no drive.
//    InputStream(Trigger isMode) {
//        this();
//        this.isMode = isMode;
//    }
//
//    /// Default Basic Drive Constructor
//    InputStream(Trigger isMode,
//                DoubleSupplier x,
//                DoubleSupplier y) {
//        this(isMode);
//        this.swerveInputStream = SwerveInputStream.of(
//                        subsystems.swerve().getSwerveDrive(), x, y) // Make the input stream.
//                .cubeTranslationControllerAxis(true)
//                .cubeRotationControllerAxis(true)
//                .scaleTranslation(SwerveBindings.normalTranslation)
//                .scaleRotation(SwerveBindings.normalRotation)
//                .deadband(SwerveBindings.deadzone)
//                .robotRelative(true)
//                .allianceRelativeControl(false);
//        this.SwerveBindings.withDefaultCommand(() -> subsystems.swerve.driveFieldOriented(() -> swerveInputStream.get()));
//    }
//
//    /**
//     * Basic Control Methods to help get started.
//     */
//    private class StartingMethods {
//        /**
//         * Default Xbox Drive Constructor with regular rotation.
//         * WARNING: Creates a new stream, DO NOT use inline.
//         *
//         * @param isMode     The trigger telling the stream we are in the Input Selection.
//         * @param driverXbox the {@link CommandXboxController} to bind to.
//         */
//        public InputBuilder.InputStream defaultXboxDrive(Trigger isMode, CommandXboxController driverXbox) {
//            // Load default drive constructor.
//            var xboxDrive = new InputBuilder.InputStream(isMode,
//                    () -> driverXbox.getLeftY() * -1,
//                    () -> driverXbox.getLeftX() * -1);
//            // Set heading drive.
//            xboxDrive.swerveInputStream.withControllerRotationAxis(() -> driverXbox.getRightX() * -1);
//            // Return our new stream.
//            return xboxDrive;
//        }
//
//        /**
//         * Default Xbox Drive Constructor with heading rotation.
//         * WARNING: Creates a new stream, DO NOT use inline.
//         *
//         * @param isMode     The trigger telling the stream we are in the Input Selection.
//         * @param driverXbox the {@link CommandXboxController} to bind to.
//         */
//        public InputBuilder.InputStream headingXboxDrive(Trigger isMode, CommandXboxController driverXbox) {
//            // Load default drive constructor.
//            var headingDrive = new InputBuilder.InputStream(isMode,
//                    () -> driverXbox.getLeftY() * -1,
//                    () -> driverXbox.getLeftX() * -1);
//            // Set heading rotation.
//            headingDrive.swerveInputStream
//                    .cubeRotationControllerAxis(false)
//                    .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
//                    .headingWhile(true);
//            // Return our new stream.
//            return headingDrive;
//        }
//    }
//
//    /// ***** Auto Bindings ***** ///
//
//    /**
//     * Smart controls that make use of automatic sequencing controls.
//     */
//    public class SmartBindings {
//
//        public InputBuilder.InputStream.SmartBindings withBasicAim(Trigger basicAim) {
//            this.back().TurretBindings.withSetAngle(basicAim, Degrees.of(0))
//                    .back().HoodBindings.withSetAngle(basicAim, Degrees.of(15))
//                    .back().FlyWheelBindings.withSetVelocity(basicAim, RotationsPerSecond.of(47))
//                    .back().SwerveBindings.withLookAtHub(basicAim);
//            return this;
//        }
//
//        public InputBuilder.InputStream.SmartBindings withHomeAll(Trigger home) {
//            this.back().HoodBindings
//                    .withAutoHome(home)
//                    .back().TurretBindings
//                    .withAutoHome(home);
//            return this;
//        }
//
//        public InputBuilder.InputStream.SmartBindings withFuelControl(Trigger intake, Trigger index, Trigger spinUp) {
//            this.withBlockingIntakeIntoHopper(index.negate().and(intake)); // If not indexing, intake to hopper.
//            this.withIndexIntoFlyWheel(intake.negate()
//                    .and(index)); // If not intaking, index into flywheel.
//            this.back().KickerBindings.withRunKicker(spinUp, true);
//            return this;
//        }
//
//        public InputBuilder.InputStream.SmartBindings withFuelControl(Trigger intake, Trigger index, Trigger spinUp, AngularVelocity flyWheelSpeed) {
//            this.withBlockingIntakeIntoHopper(index.negate().and(intake)); // If not indexing, intake to hopper.
//            this.withIndexIntoFlyWheel(intake.negate()
//                    .and(subsystems.flywheel.isNear(() -> flyWheelSpeed))
//                    .and(index)); // If not intaking, index into flywheel.
//            this.withSpinUp(spinUp, flyWheelSpeed);
//
//            return this;
//        }
//
//        public InputBuilder.InputStream.SmartBindings withSOTMCalibration(Trigger indexIntoFlywheel, Trigger ballHasHitFloor, Current threshold) {
//            final Timer timeOfFlightTimer = new Timer();
//            final List<Double> dataLog = new ArrayList<>();
//
//            this.withIndexIntoFlyWheel(indexIntoFlywheel);
//            isMode.and(indexIntoFlywheel).and(subsystems.flywheel.currentSensorTrigger(threshold, Seconds.of(0)))
//                    .onTrue(Commands.runOnce(timeOfFlightTimer::start).andThen(Commands.waitUntil(ballHasHitFloor).andThen(
//                            () -> {
//                                timeOfFlightTimer.stop();
//                                final Time tofGuess = Seconds.of(timeOfFlightTimer.get());
//                                final Angle hoodAngle = subsystems.hood.getHood().getAngle();
//                                final AngularVelocity flyWheelSpeed = subsystems.flywheel.getFlyWheel().getSpeed();
//                                timeOfFlightTimer.reset();
//
//                                dataLog.add(tofGuess.in(Seconds));
//                                dataLog.add(hoodAngle.in(Degrees));
//                                dataLog.add(flyWheelSpeed.in(RPM));
//
//                                DriverStation.reportWarning("SOTM Data - ToF: " + tofGuess + " hA: " + hoodAngle.in(Degrees) + " fV: " + flyWheelSpeed.in(RPM), false);
//                            })));
//            isMode.and(() -> !dataLog.isEmpty()).and(DriverStation::isDisabled).onTrue(Commands.runOnce(() -> DriverStation.reportWarning("SOTM DataLog: " + dataLog, false)).ignoringDisable(true));
//            return this;
//        }
//
//        /**
//         * Shoots while moving using double interpolating maps while calculating for ToF, Phase Delay, Velocity, ect.
//         * Fires fuel to our side from the best spot.
//         *
//         * @param shootOnTheMoveWhile the button to map.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.SmartBindings withToggleAutoHoardFuel(Trigger shootOnTheMoveWhile, boolean startEnabled) {
//            if (!TurretBindings.isPresent || !HoodBindings.isPresent || !FlyWheelBindings.isPresent) {
//                return this;
//            }
//            final boolean[] isToggled = {startEnabled};
//            isMode.and(shootOnTheMoveWhile.toggleOnTrue(Commands.runOnce(() -> isToggled[0] = !isToggled[0])));
//            Trigger trigger = isMode.and(() -> isToggled[0]).and(InputBuilder.CustomTriggers.neutralZone.getTrigger());
//            trigger.whileTrue(scoring.shootOnTheMove(() -> subsystems.swerve().getHoardingTarget()));
//            shootOnTheMoveWhile.onFalse(subsystems.flywheel.stopFlyWheel());
//            return this;
//        }
//
//        /**
//         * Shoots while moving using double interpolating maps while calculating for ToF, Phase Delay, Velocity, ect.
//         *
//         * @param shootOnTheMoveWhile the button to map.
//         * @param withDrive           whether to aim the drive as well.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.SmartBindings withToggleAutoScoreHub(Trigger shootOnTheMoveWhile, boolean withDrive, boolean startEnabled) {
//            if (!TurretBindings.isPresent || !HoodBindings.isPresent || !FlyWheelBindings.isPresent) {
//                return this;
//            }
//            final boolean[] isToggled = {startEnabled};
//            isMode.and(shootOnTheMoveWhile.toggleOnTrue(Commands.runOnce(() -> isToggled[0] = !isToggled[0])));
//            Trigger trigger = isMode.and(() -> isToggled[0]).and(InputBuilder.CustomTriggers.allianceZone.getTrigger());
//            trigger.whileTrue(scoring.shootOnTheMove(() -> ScoringSystem.ControlConstants.SOTMTargets.HUB));
//            this.back().SwerveBindings.withLookAtHub(trigger.and(() -> withDrive));
//            shootOnTheMoveWhile.onFalse(subsystems.flywheel.stopFlyWheel());
//            return this;
//        }
//
//        /**
//         * Runs all PID Subsystems to various goals.
//         * Set any as null if you don't want it to move.
//         *
//         * @param `runAll`      the button to map.
//         * @param hoodAngle     the hood angle to set.
//         * @param turretAngle   the turret angle to set.
//         * @param flyWheelSpeed the flywheel speed to run.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.SmartBindings withRunAll(
//                Trigger runAll,
//                Supplier<Angle> hoodAngle,
//                Supplier<Angle> turretAngle,
//                Supplier<AngularVelocity> flyWheelSpeed) {
//            if (!TurretBindings.isPresent || !HoodBindings.isPresent || !FlyWheelBindings.isPresent) {
//                return this;
//            }
//            isMode.and(runAll).whileTrue(
//                    subsystems.hood.getHood().run(hoodAngle).onlyIf(() -> hoodAngle != null)
//                            .alongWith(subsystems.turret.getTurret().run(turretAngle).onlyIf(() -> turretAngle != null))
//                            .alongWith(subsystems.flywheel.getFlyWheel().run(flyWheelSpeed).onlyIf(() -> flyWheelSpeed != null)));
//            return this;
//        }
//
//        /**
//         * Runs intake in while running the kicker and indexer in reverse. This should help fill the hopper up.
//         * WARNING: If Intake, Agitator, or Kicker have commands already running, this will interrupt.
//         * IE; If called while scoring balls suddenly will stop feeding into the flywheel because we are forcing it into the hopper instead.
//         *
//         * @param intake the button to map.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.SmartBindings withBlockingIntakeIntoHopper(Trigger intake) {
//            if (!IndexerBindings.isPresent || !IntakeBindings.isPresent || !KickerBindings.isPresent) {
//                return this;
//            }
//            // Runs intake in while running the kicker and indexer in reverse. This should help fill the hopper up.
//            this.back().IndexerBindings
//                    .withRunIndexer(intake, true)
//                    .back().IntakeBindings
//                    .withRunIntake(intake, true)
//                    .back().AgitatorBindings
//                    .withRunAgitator(intake, true)
//                    .back().KickerBindings.withRunKicker(intake, false, 0.25);
//            return this;
//        }
//
//        /**
//         * THIS DOES NOT RUN THE FLYWHEEL
//         * This indexes the hopper into the shooter.
//         * Runs the indexer in.
//         * Intake does not run.
//         *
//         * @param index the button to map.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.SmartBindings withIndexIntoFlyWheel(Trigger index) {
//            if (!IndexerBindings.isPresent || !IntakeBindings.isPresent || !KickerBindings.isPresent) {
//                return this;
//            }
//            this.back().IndexerBindings
//                    .withRunIndexer(index, true)
//                    .back().AgitatorBindings.withRunAgitator(index, false)
//                    .back().IntakeBindings.withRunWithDelays(index, true, Seconds.of(1), Seconds.of(2));
//            return this;
//        }
//
//        /**
//         * THIS DOES NOT RUN THE FLYWHEEL
//         * This intakes the hopper and intake into the shooter.
//         * Runs the indexer and intake in.
//         * Intake does not run.
//         *
//         * @param intake the button to map.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.SmartBindings withIntakeIntoFlyWheel(Trigger intake) {
//            this.back().IndexerBindings
//                    .withRunIndexer(intake, true)
//                    .back().IntakeBindings
//                    .withRunIntake(intake, true)
//                    .back().AgitatorBindings
//                    .withRunAgitator(intake, false);
//            return this;
//        }
//
//        /**
//         * Spins up the kicker and flywheel at the same time.
//         *
//         * @param spinUp          the button to map.
//         * @param angularVelocity flywheel velocity goal.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.SmartBindings withSpinUp(Trigger spinUp, AngularVelocity angularVelocity) {
//            this.back().FlyWheelBindings
//                    .withSetVelocity(spinUp, angularVelocity)
//                    .back().KickerBindings
//                    .withRunKicker(spinUp, true);
//            return this;
//        }
//
//        /**
//         * Turns all spinners on to take a fuel from the intake and fire it right away.
//         * This is for manual testing
//         * <p>
//         * Starts Flywheel and kicker, waits a second.
//         * Then it turns on the indexer and intake.
//         *
//         * @param turnOnAll the button to map.
//         * @return this, for chaining
//         */
//        public InputBuilder.InputStream.SmartBindings withDutyCycleAll(Trigger turnOnAll) {
//            if (!IntakeBindings.isPresent || !IndexerBindings.isPresent || !KickerBindings.isPresent || !FlyWheelBindings.isPresent) {
//                return this;
//            }
//            isMode.and(turnOnAll).whileTrue(
//                    subsystems.flywheel.runFlyWheel(FlyWheelBindings.flyWheelSpeed, true)
//                            .alongWith(subsystems.kicker.runKicker(KickerBindings.kickerSpeed, true))
//                            .alongWith(
//                                    Commands.waitSeconds(1)
//                                            .alongWith(subsystems.indexer.runIndexer(IndexerBindings.indexSpeed, true))
//                                            .alongWith(subsystems.intake.intake(IntakeBindings.intakeSpeed, true))));
//            return this;
//        }
//
//        /**
//         * Leaves SmartBindings going back to the InputStream.
//         *
//         * @return this InputStream.
//         */
//        public InputBuilder.InputStream back() {
//            return InputBuilder.InputStream.this;
//        }
//    }
//
//
//    /// ***** Separate Bindings ***** ///
//
//
//
//
//    private class IndexerBindings {
//        /**
//         * Checks if this subsystem is present, if not, don't bind anything.
//         */
//        private final boolean isPresent;
//        /**
//         * The default duty cycle speed to run at.
//         */
//        @Setter
//        private double indexSpeed = 1;
//
//
//        private IndexerBindings() {
//            this.isPresent = subsystems.indexer != null;
//        }
//
//        /**
//         * Runs the indexer at preset speed, stopping when finished.
//         *
//         * @param runIndexer then button to map.
//         * @param isIn       whether to spin in or out.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.IndexerBindings withRunIndexer(Trigger runIndexer, boolean isIn) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(runIndexer).whileTrue(subsystems.indexer.runIndexer(indexSpeed, isIn))
//                    .onFalse(subsystems.indexer.stopIndexer());
//            return this;
//        }
//
//        /**
//         * Sets a default command for this subsystem.
//         *
//         * @param defaultCommand the default command to set.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.IndexerBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.indexer.setDefaultCommand(defaultCommand.get())));
//            return this;
//        }
//
//        /**
//         * Leaves IndexerBindings going back to the InputStream.
//         *
//         * @return this InputStream.
//         */
//        public InputBuilder.InputStream back() {
//            return InputBuilder.InputStream.this;
//        }
//    }
//
//    private class AgitatorBindings {
//        /**
//         * Checks if this subsystem is present, if not, don't bind anything.
//         */
//        private final boolean isPresent;
//        /**
//         * The default duty cycle speed to run at.
//         */
//        @Setter
//        private double agitatorSpeed = 1;
//
//
//        private AgitatorBindings() {
//            this.isPresent = subsystems.agitator != null;
//        }
//
//        /**
//         * Runs the agitator at preset speed, stopping when finished.
//         *
//         * @param runAgitator then button to map.
//         * @param isIn        whether to spin in or out.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.AgitatorBindings withRunAgitator(Trigger runAgitator, boolean isIn) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(runAgitator).whileTrue(subsystems.agitator.runAgitator(agitatorSpeed, isIn))
//                    .onFalse(subsystems.agitator.stopAgitator());
//            return this;
//        }
//
//        /**
//         * Sets a default command for this subsystem.
//         *
//         * @param defaultCommand the default command to set.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.AgitatorBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.agitator.setDefaultCommand(defaultCommand.get())));
//            return this;
//        }
//
//        /**
//         * Leaves AgitatorBindings going back to the InputStream.
//         *
//         * @return this InputStream.
//         */
//        public InputBuilder.InputStream back() {
//            return InputBuilder.InputStream.this;
//        }
//    }
//
//    private class KickerBindings {
//        /**
//         * Checks if this subsystem is present, if not, don't bind anything.
//         */
//        private final boolean isPresent;
//
//        private KickerBindings() {
//            this.isPresent = subsystems.kicker != null;
//        }
//
//        /**
//         * The default duty cycle speed to run at.
//         */
//        @Setter
//        private double kickerSpeed = 1;
//
//        /**
//         * Runs the kicker at preset speed, stopping when finished.
//         *
//         * @param runkicker then button to map.
//         * @param isOut     whether to spin out.
//         * @param speed     duty cycle speed to run. [-1, 1]
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.KickerBindings withRunKicker(Trigger runkicker, boolean isOut, double speed) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(runkicker).whileTrue(subsystems.kicker.runKicker(speed, isOut))
//                    .onFalse(subsystems.kicker.stopKicker());
//            return this;
//        }
//
//        /**
//         * Runs the kicker at preset speed, stopping when finished.
//         *
//         * @param runkicker then button to map.
//         * @param isOut     whether to spin out.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.KickerBindings withRunKicker(Trigger runkicker, boolean isOut) {
//            if (!isPresent) {
//                return this;
//            }
//            this.withRunKicker(runkicker, isOut, kickerSpeed);
//            return this;
//        }
//
//        /**
//         * Sets a default command for this subsystem.
//         *
//         * @param defaultCommand the default command to set.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.KickerBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.kicker.setDefaultCommand(defaultCommand.get())));
//            return this;
//        }
//
//        /**
//         * Leaves KickerBindings going back to the InputStream.
//         *
//         * @return this InputStream.
//         */
//        public InputBuilder.InputStream back() {
//            return InputBuilder.InputStream.this;
//        }
//    }
//
//    /// ***** Scoring Mechanisms ***** ///
//
//    /**
//     * FlyWheel Button Bindings
//     */
//    private class FlyWheelBindings {
//        /**
//         * Checks if this subsystem is present, if not, don't bind anything.
//         */
//        private final boolean isPresent;
//        /**
//         * The default duty cycle speed to run at.
//         */
//        @Setter
//        private double flyWheelSpeed = 0.75;
//
//        private FlyWheelBindings() {
//            this.isPresent = subsystems.flywheel != null;
//        }
//
//        /**
//         * Sets the flywheel velocity goal.
//         *
//         * @param setVelocity      the button to map.
//         * @param flywheelVelocity the velocity goal.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.FlyWheelBindings withSetVelocity(Trigger setVelocity, AngularVelocity flywheelVelocity) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(setVelocity).whileTrue(subsystems.flywheel.getFlyWheel().run(flywheelVelocity))
//                    .onFalse(subsystems.flywheel.stopFlyWheel());
//            return this;
//        }
//
//        /**
//         * Runs the flywheel at preset speed, stopping when finished.
//         *
//         * @param runFlyWheel then button to map.
//         * @param isOut       whether to spin out.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.FlyWheelBindings withRunFlyWheel(Trigger runFlyWheel, boolean isOut) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(runFlyWheel).whileTrue(subsystems.flywheel.runFlyWheel(flyWheelSpeed, isOut))
//                    .onFalse(subsystems.flywheel.stopFlyWheel());
//            return this;
//        }
//
//        /**
//         * Simulates the turret shooting fuel.
//         *
//         * @param shoot the button to map.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.FlyWheelBindings withSimShoot(Trigger shoot) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(shoot).whileTrue(subsystems.flywheel.simShoot(subsystems).andThen(Commands.waitSeconds(.25)).repeatedly());
//            return this;
//        }
//
//        /**
//         * Sets a default command for this subsystem.
//         *
//         * @param defaultCommand the default command to set.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.FlyWheelBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.flywheel.setDefaultCommand(defaultCommand.get())));
//            return this;
//        }
//
//        /**
//         * Leaves FlyWheelBindings going back to the InputStream.
//         *
//         * @return this InputStream.
//         */
//        public InputBuilder.InputStream back() {
//            return InputBuilder.InputStream.this;
//        }
//    }
//
//    /**
//     * Hood Button Bindings
//     */
//    private class HoodBindings {
//        /**
//         * Checks if this subsystem is present, if not, don't bind anything.
//         */
//        private final boolean isPresent;
//        /**
//         * The default duty cycle speed to run at.
//         */
//        @Setter
//        private double hoodSpeed = 0.25;
//
//        private HoodBindings() {
//            this.isPresent = subsystems.hood != null;
//        }
//
//        /**
//         * Homes the mechanism using current sensing.
//         *
//         * @param home the button to map.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.HoodBindings withAutoHome(Trigger home) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(home).whileTrue(subsystems.hood.home());
//            return this;
//        }
//
//        /**
//         * Sets the hood angle to the given angle.
//         * Horizontal to the floor is 0 degrees.
//         * w
//         *
//         * @param setAngle  the button to map.
//         * @param hoodAngle the angle to move to.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.HoodBindings withSetAngle(Trigger setAngle, Angle hoodAngle) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(setAngle).whileTrue(subsystems.hood.getHood().setAngle(hoodAngle));
//            return this;
//        }
//
//        /**
//         * Runs the hood at preset speed, stopping when finished.
//         *
//         * @param runHood the button to map.
//         * @param isUp    whether to go up.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.HoodBindings withRunHood(Trigger runHood, boolean isUp) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(runHood).whileTrue(subsystems.hood.runHood(hoodSpeed, isUp))
//                    .onFalse(subsystems.hood.stopHood());
//            return this;
//        }
//
//        /**
//         * Sets a default command for this subsystem.
//         *
//         * @param defaultCommand the default command to set.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.HoodBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.hood.setDefaultCommand(defaultCommand.get())));
//            return this;
//        }
//
//        /**
//         * Leaves HoodBindings going back to the InputStream.
//         *
//         * @return this InputStream.
//         */
//        public InputBuilder.InputStream back() {
//            return InputBuilder.InputStream.this;
//        }
//    }
//
//    /**
//     * Turret Button Bindings
//     */
//    private class TurretBindings {
//        /**
//         * Checks if this subsystem is present, if not, don't bind anything.
//         */
//        private final boolean isPresent;
//        /**
//         * The default duty cycle speed to run at.
//         */
//        @Setter
//        private double turretSpeed = .25;
//
//        private TurretBindings() {
//            this.isPresent = subsystems.turret != null;
//        }
//
//        /**
//         * Homes the mechanism using current sensing.
//         *
//         * @param home the button to map.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.TurretBindings withAutoHome(Trigger home) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(home).whileTrue(subsystems.turret.home());
//            return this;
//        }
//
//        /**
//         * Sets the turret angle to the given angle.
//         * Horizontal to the floor is 0 degrees.
//         * w
//         *
//         * @param setAngle    the button to map.
//         * @param turretAngle the angle to move to.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.TurretBindings withSetAngle(Trigger setAngle, Angle turretAngle) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(setAngle).whileTrue(subsystems.turret.getTurret().setAngle(turretAngle));
//            return this;
//        }
//
//        /**
//         * Runs the turret at preset speed, stopping when finished.
//         *
//         * @param runTurret then button to map.
//         * @param isCCW     whether to turn CCW, aka lefty loosey.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.TurretBindings withRunTurret(Trigger runTurret, boolean isCCW) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(runTurret).whileTrue(subsystems.turret.runTurret(turretSpeed, isCCW))
//                    .onFalse(subsystems.turret.stopTurret());
//            return this;
//        }
//
//        /**
//         * Sets a default command for this subsystem.
//         *
//         * @param defaultCommand the default command to set.
//         * @return this, for chaining.
//         */
//        public InputBuilder.InputStream.TurretBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//            if (!isPresent) {
//                return this;
//            }
//            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.turret.setDefaultCommand(defaultCommand.get())));
//            return this;
//        }
//
//        /**
//         * Leaves TurretBindings going back to the InputStream.
//         *
//         * @return this InputStream.
//         */
//        public InputBuilder.InputStream back() {
//            return InputBuilder.InputStream.this;
//        }
//    }
//}
