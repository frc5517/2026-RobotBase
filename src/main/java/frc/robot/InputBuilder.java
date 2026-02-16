package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.math.AllianceFlipUtil;
import frc.robot.util.math.FieldConstants;
import lombok.*;
import lombok.experimental.Accessors;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.InputBuilder.InputSelections.*;
import static frc.robot.Telemetry.Publishers.Robot.inputOverride;


public class InputBuilder
{
    /// Our subsystems and systems to call to from RobotContainer
    private final Subsystems subsystems;
    // Control chooser for dashboard
    private static final SendableChooser<InputSelections> inputSelector = new SendableChooser<>();
    // Control input devices
    private final CommandXboxController driverXbox;
    private final CommandXboxController operatorXbox;

    public InputBuilder(Subsystems subsystems) {
        /// Stop the console spam from not all buttons showing up.
        DriverStation.silenceJoystickConnectionWarning(true);
        /// Initialize our subsystem calls
        this.subsystems = subsystems;
        /// Initialize Binding Methods here.
        this.driverXbox = new CommandXboxController(0);
        this.operatorXbox = new CommandXboxController(1);
        /// Initialize input publisher
        Telemetry.Publishers.Robot.inputPublisher.accept(() -> inputSelector);

        testing();
    }

    // Control binding type enum
    public enum InputSelections {
        /// Default Input Schema
        SINGLE_XBOX("Single Xbox", false),
        DUAL_XBOX("Dual Xbox"),
        TESTING("Testing", true),

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

    public void testing() {
        // Define constants first, like speeds.
        var stream =
                new InputStream().
                        StartingMethod // Dumb constructor to load a different one.
                        .defaultXboxDrive(TESTING.isMode, driverXbox)
                        /// Initial Constants
                        .normalRotation(.8)
                        .normalTranslation(.8)
                        .slowRotation(.4)
                        .slowTranslation(.4)
                        /// Button Bindings
                        .SwerveBindings                 /// SwerveDrive Bindings
                        .withSlowAtBump()
                        .withSlowDrive(                 driverXbox.rightBumper())
                        .withToggleCentricity(          driverXbox.back(), true)
                        .withResetSimOdometry(          driverXbox.start())
                        .updateSwerveStream()           // Update our stream after making default drive speed changes.
                        .back().HoodBindings            /// Hood
                        .withRunHood(                   driverXbox.pov(0), true)
                        .withRunHood(                   driverXbox.pov(180), false)
                        .back().TurretBindings          /// Turret
                        .withAimAt(                     driverXbox.y())
                        .withRunTurret(                 driverXbox.pov(270), true)
                        .withRunTurret(                 driverXbox.pov(90), false)
                        .back().IndexerBindings
                        .withRunIndexer(                driverXbox.leftBumper(), true)
                        .withRunIndexer(                driverXbox.rightBumper(), false)
                        .back().IntakeBindings          /// Intake
                        .withRunIntake(                 driverXbox.leftBumper(), true)
                        .withRunIntake(                 driverXbox.rightBumper(), false)
                        .back().FlyWheelBindings        /// Flywheel
                        .withSimShoot(                  driverXbox.rightTrigger())
                        .back().MiscBindings            /// Misc Bindings
                        .withChangeInput(SINGLE_XBOX,   driverXbox.a())
                        .resetField(                    driverXbox.start());
    }

    /// ***** Input Stream Below ***** ///

    @Accessors(fluent = true, chain = true)
    private class InputStream {
        /// Stream Constants
        @Setter private Trigger isMode;
        /// Drive Constants
        @Setter private double deadzone = 0.01;
        @Setter private double slowTranslation = 0.3;
        @Setter private double slowRotation = 0.2;
        @Setter private double normalTranslation = .8;
        @Setter private double normalRotation = .8;
        @Setter private double boostTranslation = 1.0;
        @Setter private double boostRotation = 0.75;
        @Setter @Getter private Supplier<Command> driveCommand; // Default drive command.
        /// Intake Constants
        @Setter private double intakeSpeed = .6;
        /// Indexer Constants
        @Setter private double indexSpeed = 0.5;
        /// FlyWheel Constants
        @Setter private double flyWheelSpeed = 0.5;
        /// Hood Constants
        @Setter private double hoodSpeed = 0.5;
        /// Turret Constants
        @Setter private double turretSpeed = 0.5;
        //@Setter private Command turretDefaultCommand = subsystems.turret.getTurret().runTo(TurretSubsystem.ControlConstants.ANGLE_TOLERANCE)

        /// Inner Config Namespaces
        public final StartingMethods StartingMethod = new StartingMethods();
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
        InputStream() {
        }

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
                    .scaleTranslation(normalTranslation)
                    .scaleRotation(normalRotation)
                    .deadband(deadzone)
                    .robotRelative(true)
                    .allianceRelativeControl(false);
            this.driveCommand = () -> subsystems.swerve.driveFieldOriented(() -> swerveInputStream.get());
            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.swerve.setDefaultCommand(driveCommand().get())));
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
                        () -> driverXbox.getLeftY(),
                        () -> driverXbox.getLeftX());
                // Set heading drive.
                xboxDrive.swerveInputStream.withControllerRotationAxis(() -> driverXbox.getRightX());
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

        }


        /// ***** Separate Bindings ***** ///

        /**
         * Intake button bindings.
         */
        private class IntakeBindings {
            private final boolean isPresent;

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
             * Leaves IntakeBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }


        private class IndexerBindings {
            private final boolean isPresent;

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
                isMode.and(runIndexer).whileTrue(subsystems.indexer.runIndexer(hoodSpeed, isIn))
                        .onFalse(subsystems.indexer.stopIndexer());
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
            private final boolean isPresent;
            private KickerBindings() {
                this.isPresent = subsystems.kicker != null;
            }

        }

        /// ***** Scoring Mechanisms ***** ///

        /**
         * FlyWheel Button Bindings
         */
        private class FlyWheelBindings {
            private final boolean isPresent;

            private FlyWheelBindings() {
                this.isPresent = subsystems.flywheel != null;
            }
            /**
             * Simulates the turret shooting fuel.
             *
             * @param shoot the button to map.
             * @return this, for chaining.
             */
            public FlyWheelBindings withSimShoot(Trigger shoot) {
                if (!isPresent) {return this;}
                isMode.and(shoot).onTrue(subsystems.flywheel.simShoot());
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
            private final boolean isPresent;

            private HoodBindings() {
                this.isPresent = subsystems.hood != null;
            }

            /**
             * Runs the hood at preset speed, using PID to hold the angle when finished.
             *
             * @param runHood then button to map.
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
            private final boolean isPresent;
            private TurretBindings() {
                this.isPresent = subsystems.turret != null;
            }

            public TurretBindings withAimAt(Trigger aimAt) {
                if (!isPresent) {return this;}
                isMode.and(aimAt).whileTrue(new ShootOnTheMoveCommand(subsystems, () -> FieldConstants.Hub.topCenterPoint));
                return this;
            }

            /**
             * Runs the turret at preset speed, using PID to hold the angle when finished.
             *
             * @param runTurret then button to map.
             * @param isCCW whether to turn CCW, aka lefty loosey.
             * @return this, for chaining.
             */
            public TurretBindings withRunTurret(Trigger runTurret, boolean isCCW) {
                if (!isPresent) {return this;}
                isMode.and(runTurret).whileTrue(subsystems.turret.runTurret(hoodSpeed, isCCW))
                        .onFalse(subsystems.turret.stopTurret());
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
            private final boolean isPresent;
            private SwerveBindings() {
                this.isPresent = subsystems.swerve != null;
            }
            /**
             * Changes inputs to the given input selection.
             */
            public SwerveBindings withToggleCentricity(Trigger toggleCentricity, boolean fieldDefault) {
                // Set our default.
                swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault);
                // Toggle when pressed.
                isMode.and(toggleCentricity).toggleOnTrue(Commands.runEnd(
                        () -> swerveInputStream.robotRelative(fieldDefault).allianceRelativeControl(!fieldDefault),
                        () -> swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault)));
                return this;
            }

            /**
             * Holding the button slows the drive.
             *
             * @param slowDrive the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withSlowDrive(Trigger slowDrive) {
                isMode.and(slowDrive)
                        .onTrue(Commands.runOnce(() -> swerveInputStream.scaleTranslation(slowTranslation).scaleRotation(slowRotation)))
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
             * Updates the {@link SwerveInputStream} with the latest values.
             *
             * @return this, for chaining.
             */
            public SwerveBindings updateSwerveStream() {
                swerveInputStream
                        .scaleTranslation(normalTranslation)
                        .scaleRotation(normalRotation)
                        .deadband(deadzone);
                return this;
            }

            public SwerveBindings withSlowAtBump() {
                final Telemetry.ZoneTrigger bumpZones = new Telemetry.ZoneTrigger(
                        Telemetry.Publishers.Robot.bumpZonePublisher,
                        Pair.of(new Translation2d(3.75, 1.5), new Translation2d(5.5, 3.5)),
                        Pair.of(new Translation2d(3.75, 4.5), new Translation2d(5.5, 6.5)),
                        Pair.of(new Translation2d(11, 4.5), new Translation2d(12.75, 6.5)),
                        Pair.of(new Translation2d(11, 1.5), new Translation2d(12.75, 3.5)));

                withSlowDrive(isMode.and(bumpZones.getTrigger()));
                return this;
            }

            /**
             * Leaves SwerveBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
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
                isMode.and(() -> RobotBase.isSimulation()).and(resetField).onTrue(
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
             * Sets a default command for this input selection.
             *
             * @param subsystem the subsystem to apply the default command to.
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public MiscBindings withDefaultCommand(SubsystemBase subsystem, Supplier<Command> defaultCommand) {
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystem.setDefaultCommand(defaultCommand.get())));
                return this;
            }
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

    /**
     * Neatly packs our subsystems into a bite size struct.
     */
    public record Subsystems(
            FlyWheelSubsystem flywheel,
            HoodSubsystem hood,
            IndexerSubsystem indexer,
            IntakeSubsystem intake,
            SubsystemBase kicker,
            SwerveSubsystem swerve,
            TurretSubsystem turret) {}
}