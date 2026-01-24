package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import lombok.*;
import lombok.experimental.Accessors;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

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

    public void singleXbox() {
        var stream =
                new InputStream(SINGLE_XBOX.isMode)
                        .deadzone(0.01)
                        .withXboxDrive(driverXbox)
                        .withResetSimOdometry(driverXbox.start());
    }

    public void testing() {
        // Define constants first, like speeds.
        var stream =
                new InputStream(TESTING.isMode)
                        .normalRotation(1)
                        .normalTranslation(.4)
                        .slowRotation(.4)
                        .slowTranslation(1)
                        .withXboxDrive(driverXbox)
                        .withSlowDrive(driverXbox.rightBumper())
                        .withRunIntake(driverXbox.leftBumper())
                        .withToggleCentricity(driverXbox.back(), true)
                        .withChangeInput(SINGLE_XBOX, driverXbox.a())
                        .resetField(driverXbox.start())
                        .updateDriveCommand();
    }

    @Accessors(fluent = true, chain = true, makeFinal = true)
    private class InputStream {
        /// Stream Constants
        @Setter private Trigger isMode;
        /// Drive Constants
        @Setter private double deadzone = 0.05;
        @Setter private double slowTranslation = 0.3;
        @Setter private double slowRotation = 0.2;
        @Setter private double normalTranslation = 1;
        @Setter private double normalRotation = 1.0;
        @Setter private double boostTranslation = 1.0;
        @Setter private double boostRotation = 0.75;
        @Setter private SwerveInputStream swerveInputStream;
        @Setter private Command driveCommand;

        InputStream(Trigger isMode) {
            this.isMode = isMode;
        }

        /**
         * Defaults to regular xbox driver control.
         *
         * @param driverXbox
         * @return
         */
        public InputStream withXboxDrive(CommandXboxController driverXbox) {
            return withSwerveInputStream(new SwerveInputStream(
                    subsystems.swerve.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1,
                    () -> driverXbox.getRightX() * -1)
                    .robotRelative(true)
                    .allianceRelativeControl(false));
        }

        /**
         * Adds all the builder values.
         *
         * @param swerveInputStream
         * @return
         */
        public InputStream withSwerveInputStream(SwerveInputStream swerveInputStream) {
            swerveInputStream(swerveInputStream
                            .deadband(deadzone)
                            .scaleTranslation(normalTranslation)
                            .scaleRotation(normalRotation));
            updateDriveCommand();
            return this;
        }

        //TODO
        public InputStream withSnakeDrive(
                DoubleSupplier transX, DoubleSupplier transY,
                DoubleSupplier headingX, DoubleSupplier headingY, double inputThreshold) {
            // When input is greater than the threshold override heading.
            final Trigger xTrigger = new Trigger(() -> headingY.getAsDouble() > inputThreshold);
            final Trigger yTrigger = new Trigger(() -> headingX.getAsDouble() > inputThreshold);

            withSwerveInputStream(new SwerveInputStream(
                    subsystems.swerve.getSwerveDrive(), transX, transY, transX, transY)
                    //.cubeRotationControllerAxis(true)
                    //.cubeTranslationControllerAxis(true)
                    .robotRelative(false)
                    .allianceRelativeControl(true)
                    .headingWhile(true));

            isMode.and(xTrigger.or(yTrigger)).onTrue(Commands.runOnce(
                    () -> swerveInputStream.withControllerHeadingAxis(headingX, headingY)))
                    .onFalse(Commands.runOnce(
                            () -> swerveInputStream.withControllerHeadingAxis(
                                    () -> transY.getAsDouble() * -1, () -> transX.getAsDouble() * -1)));

            return this;
        }

        /**
         * Runs the intake.
         *
         * @param runIntake the button to bind to.
         * @return this, for chaining.
         */
        public InputStream withRunIntake(Trigger runIntake) {
            isMode.and(runIntake).whileTrue(subsystems.intake.runIntake());
            return this;
        }

        /**
         * Changes inputs to the given input selection.
         */
        public InputStream withToggleCentricity(Trigger toggleCentricity, boolean fieldDefault) {
            isMode.and(toggleCentricity).onTrue(Commands.runOnce(() -> swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault)))
                    .onFalse(Commands.runOnce(() -> swerveInputStream.robotRelative(fieldDefault).allianceRelativeControl(!fieldDefault)));
            return this;
        }

        /**
         * TODO
         * @param slowDrive
         * @return
         */
        public InputStream withSlowDrive(Trigger slowDrive) {
            isMode.and(slowDrive).whileTrue(Commands.runEnd(
                    () -> {
                        System.out.println("Slow Drive");
                        swerveInputStream.scaleTranslation(slowTranslation).scaleRotation(slowRotation);
                    },
                    () -> swerveInputStream.scaleTranslation(normalTranslation).scaleRotation(normalRotation)));
            return this;
        }

        /**
         * Changes inputs to the given input selection.
         */
        public InputStream withChangeInput(InputSelections bindingType, Trigger changeInput) {
            isMode.and(changeInput).onTrue(Commands.runOnce(() -> inputOverride.set(bindingType.name)));
            return this;
        }

        public InputStream withResetSimOdometry(Trigger resetOdometry) {
            isMode.and(resetOdometry).onTrue(Commands.runOnce(() -> subsystems.swerve.getSwerveDrive().resetOdometry(subsystems.swerve.getSwerveDrive().getSimulationDriveTrainPose().get())));
            return this;
        }

        public InputStream resetField(Trigger sendMessage) {
            isMode.and(sendMessage).onTrue(Commands.runOnce(() ->
                    SimulatedArena.getInstance().resetFieldForAuto()));
            return this;
        }

        // TODO
        public InputStream updateDriveCommand() {
            driveCommand(subsystems.swerve.driveFieldOriented(swerveInputStream));
            subsystems.swerve.setDefaultCommand(driveCommand);
            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.swerve.setDefaultCommand(driveCommand)));
            return this;
        }
    }

    /**
     * Just a small helper to group the subsystem parameters together.
     */
    public static class Subsystems {
        public final FlyWheelSubsystem flywheel;
        public final HoodSubsystem hood;
        public final IndexerSubsystem indexer;
        public final IntakeSubsystem intake;
        public final SwerveSubsystem swerve;
        public final TurretSubsystem turret;

        Subsystems(
                FlyWheelSubsystem flywheel,
                HoodSubsystem hood,
                IndexerSubsystem indexer,
                IntakeSubsystem intake,
                SwerveSubsystem swerve,
                TurretSubsystem turret
        ) {
            this.flywheel = flywheel;
            this.hood = hood;
            this.indexer = indexer;
            this.intake = intake;
            this.swerve = swerve;
            this.turret = turret;
        }
    }
}