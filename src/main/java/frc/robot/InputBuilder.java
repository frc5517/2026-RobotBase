package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import lombok.*;
import lombok.experimental.Accessors;
import swervelib.SwerveInputStream;

import java.util.function.DoubleSupplier;

import static frc.robot.Telemetry.Publishers.Robot.inputOverride;


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
        InputStream.builder(InputSelections.TESTING.isMode)
                .withSwerve(swerve)
                .withSwerveInputStream(new SwerveInputStream(
                        swerve.getSwerveDrive(),
                        () -> driverXbox.getLeftX() * -1,
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getRightX() * 1))
                .withBoostTranslation(1)
                .withChangeInput(InputSelections.SINGLE_XBOX, driverXbox.a())
                .withTestMessage(driverXbox.b())
                .build();
    }

    @Accessors(fluent = true, chain = true, makeFinal = false)
    @Builder(setterPrefix = "with")
    public static class InputStream {
        /// Stream Constants
        @Builder.Default private final Trigger isMode = new Trigger(() -> false);
        /// Drive Constants
        @Builder.Default private final SwerveSubsystem swerve = null;
        @Builder.Default double driveSpeed = 1;
        @Builder.Default double slowTranslation = 0.3;
        @Builder.Default double slowRotation = 0.2;
        @Builder.Default double normalTranslation = 0.8;
        @Builder.Default double normalRotation = 0.6;
        @Builder.Default double boostTranslation = 1.0;
        @Builder.Default double boostRotation = 0.75;
        @Builder.Default SwerveInputStream swerveInputStream = null;
        @Builder.Default Command driveCommand = null;
        /// Elevator Constants

        public static InputStreamBuilder builder(Trigger isMode) {
            return new InputStreamBuilder()
                    .withIsMode(isMode);
        }

        public static class InputStreamBuilder {
            /**
             * Builds.
             * TODO
             *
             * @return
             */
            public InputStream build() {
                if (driveCommand$set) {
                    isMode$value.and(DriverStation::isEnabled).onTrue(
                            Commands.runOnce(() -> swerve$value.setDefaultCommand(driveCommand$value)));
                }
                return new InputStream(
                        isMode$value,
                        swerve$value,
                        driveSpeed$value,
                        slowTranslation$value,
                        slowRotation$value,
                        normalTranslation$value,
                        normalRotation$value,
                        boostTranslation$value,
                        boostRotation$value,
                        swerveInputStream$value,
                        driveCommand$value);
            }

            /**
             * Builds with drive controls.
             * TODO
             *
             * @return
             */
            public InputStream build(
                    SwerveSubsystem swerve,
                    DoubleSupplier translationX,
                    DoubleSupplier translationY,
                    DoubleSupplier rotation) {
                if (!swerveInputStream$set) {
                    withSwerve(swerve)
                            .withSwerveInputStream(new SwerveInputStream(
                                    swerve.getSwerveDrive(), translationX, translationY, rotation)
                            .scaleTranslation(normalTranslation$value)
                            .scaleRotation(normalRotation$value)
                    );
                }
                return build();
            }

            /**
             * Defaults to regular xbox driver control.
             *
             * @param swerve
             * @param driverXbox
             * @return
             */
            public InputStreamBuilder withXboxDrive(
                    SwerveSubsystem swerve,
                    CommandXboxController driverXbox) {
                if (!swerveInputStream$set) {
                    withSwerveInputStream(new SwerveInputStream(
                            swerve.getSwerveDrive(),
                            () -> driverXbox.getLeftX() * -1,
                            () -> driverXbox.getLeftY() * -1,
                            () -> driverXbox.getRightX() * 1)
                            .scaleTranslation(normalTranslation$value)
                            .scaleRotation(normalRotation$value)
                    );
                }
                return this;
            }



            public InputStreamBuilder withSwerve(SwerveSubsystem swerve) {
                if (swerveInputStream$set) {
                    withDriveCommand(swerve$value.driveFieldOriented(swerveInputStream$value));
                }
                return this;
            }

            public InputStreamBuilder withSwerveInputStream(SwerveInputStream swerveInputStream) {
                if (swerve$set) {
                    withDriveCommand(swerve$value.driveFieldOriented(swerveInputStream));
                }
                return this;
            }

            /**
             * Changes inputs to the given input selection.
             */
            public InputStreamBuilder withChangeInput(InputSelections bindingType, Trigger changeInput) {
                isMode$value.and(changeInput).onTrue(Commands.runOnce(() -> inputOverride.set(bindingType.name)));
                return this;
            }

            public InputStreamBuilder withTestMessage(Trigger sendMessage) {
                isMode$value.and(sendMessage).onTrue(Commands.runOnce(() ->
                        System.out.printf(
                            "Input Builder Test - SwerveSubsystem: %s - %s\n SwerveStream: %s - %s\n SwerveCommand: %s - %s\n",
                                swerve$set, swerve$value, swerveInputStream$set, swerveInputStream$value,  driveCommand$set, driveCommand$value)));
                return this;
            }
        }
        /// External from builder, just in case we need to set things after the builder.
    }
}