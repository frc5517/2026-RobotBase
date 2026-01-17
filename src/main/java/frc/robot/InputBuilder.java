package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import lombok.*;
import lombok.experimental.Accessors;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import java.util.function.DoubleSupplier;

import static frc.robot.Telemetry.Publishers.Robot.inputOverride;
import static lombok.AccessLevel.PRIVATE;


public class InputBuilder
{
    /// Our subsystems and systems to call to from RobotContainer
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    // Control chooser for dashboard
    private static final SendableChooser<InputSelections> inputSelector = new SendableChooser<>();
    // Control input devices
    private final CommandXboxController driverXbox;
    private final CommandXboxController operatorXbox;

    public InputBuilder(
            SwerveSubsystem swerve,
            TurretSubsystem turret) {
        /// Initialize our subsystem calls
        this.swerve = swerve;
        this.turret = turret;
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
        InputStream.builder().withIsMode(InputSelections.TESTING.isMode)
                .withNormalRotation(1)
                .withNormalTranslation(1)
                .withSwerve(swerve)
                .build()
                .withXboxDrive(swerve, driverXbox)
                .withChangeInput(InputSelections.SINGLE_XBOX, driverXbox.a())
                .resetField(driverXbox.start());
    }

    @Accessors(fluent = true, chain = true, makeFinal = true)
    @Builder(setterPrefix = "with", toBuilder = true)
    public static class InputStream {
        /// Stream Constants
        private final Trigger isMode;
        /// Drive Constants
        @Builder.Default double deadzone = 0.05;
        @Builder.Default double slowTranslation = 0.3;
        @Builder.Default double slowRotation = 0.2;
        @Builder.Default double normalTranslation = 0.8;
        @Builder.Default double normalRotation = 1.0;
        @Builder.Default double boostTranslation = 1.0;
        @Builder.Default double boostRotation = 0.75;
        @Getter SwerveInputStream swerveInputStream;
        private SwerveSubsystem swerve;
        private Command driveCommand;

        /**
         * Defaults to regular xbox driver control.
         *
         * @param swerve
         * @param driverXbox
         * @return
         */
        public InputStream withXboxDrive(
                SwerveSubsystem swerve,
                CommandXboxController driverXbox) {
            if (swerve != null) {
                return withSwerveInputStream(swerve, new SwerveInputStream(
                        swerve.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1,
                        () -> driverXbox.getRightX() * -1)
                        .robotRelative(true)
                        .allianceRelativeControl(false));
            }
            return this;
        }

        /**
         * Adds all the builder values.
         *
         * @param swerve
         * @param swerveInputStream
         * @return
         */
        public InputStream withSwerveInputStream(SwerveSubsystem swerve, SwerveInputStream swerveInputStream) {
            var updated = this.toBuilder()
                    .withSwerve(swerve)
                    .withSwerveInputStream(swerveInputStream
                            .deadband(deadzone)
                            .scaleTranslation(normalTranslation)
                            .scaleRotation(normalRotation))
                    .withDriveCommand(swerve.driveFieldOriented(swerveInputStream))
                    .build(); // set here just in case it was set out of order.
            updated.isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(updated.driveCommand)));
            return updated;
        }

        /**
         * Changes inputs to the given input selection.
         */
        public InputStream withChangeInput(InputSelections bindingType, Trigger changeInput) {
            isMode.and(changeInput).onTrue(Commands.runOnce(() -> inputOverride.set(bindingType.name)));
            return this;
        }

        public InputStream resetField(Trigger sendMessage) {
            isMode.and(sendMessage).onTrue(Commands.runOnce(() ->
                    SimulatedArena.getInstance().resetFieldForAuto()));
            return this;
        }
        /// External from builder, just in case we need to set things after the builder.
    }
}