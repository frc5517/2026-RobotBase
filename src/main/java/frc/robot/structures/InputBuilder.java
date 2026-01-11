package frc.robot.structures;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IndexerSubsystem;
import lombok.*;
import lombok.experimental.Accessors;
import yams.mechanisms.swerve.utility.SwerveInputStream;

import java.util.function.DoubleSupplier;

import static frc.robot.Telemetry.Publishers.Robot.inputOverride;


public class InputBuilder
{
    // Control chooser for dashboard
    private static final SendableChooser<InputSelections> inputSelector = new SendableChooser<>();
    // Control input devices
    private final CommandXboxController driverXbox;
    private final CommandXboxController operatorXbox;

    public InputBuilder() {
        /// Initialize Binding Methods here.
        this.driverXbox = new CommandXboxController(0);
        this.operatorXbox = new CommandXboxController(1);

        testing();
    }

    // Control binding type enum
    public enum InputSelections {
        /// Default Input Schema
        SINGLE_XBOX("Single Xbox", true),
        DUAL_XBOX("Dual Xbox"),
        TESTING("Testing"),

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
        InputStream.driverXboxControls(InputSelections.TESTING.isMode, driverXbox)
                .withArmSpeed(1)
                .withBoostTranslation(0.1)
                .build()
                // Then Bind our actions.
                .withChangeInput(InputStructure.BindingType.SINGLE_XBOX, driverXbox.a());
    }

    @Accessors(fluent = true, chain = true, makeFinal = false)
    @Builder(setterPrefix = "with", builderMethodName = "internalBuilder")
    public static class InputStream {
        /// Drive Constants
        @Builder.Default double slowTranslation = 0.3;
        @Builder.Default double slowRotation = 0.2;
        @Builder.Default double normalTranslation = 0.8;
        @Builder.Default double normalRotation = 0.6;
        @Builder.Default double boostTranslation = 1.0;
        @Builder.Default double boostRotation = 0.75;
        SwerveInputStream swerveInputStream;
        //@Builder.Default private final Command driveCommand = ;
        @Builder.Default double driveSpeed = 1;
        /// Elevator Constants
        @Builder.Default double elevatorSpeed = 0.5;
        @Builder.Default double armSpeed = 0.5;
        /// Stream Constants
        private final Trigger isMode;

        public static InputStreamBuilder builder(Trigger isMode)
        {
            return new InputStreamBuilder()
                    .withIsMode(isMode);
        }

        public static InputStreamBuilder builder(
                Trigger isMode,
                DoubleSupplier translationX,
                DoubleSupplier translationY,
                DoubleSupplier rotation)
        {
            return builder(isMode);
//                    .withSwerveInputStream(new SwerveInputStream(
//                    drive,
//                    translationX,
//                    translationY,
//                    rotation));
        }

        public static InputStreamBuilder driverXboxControls(Trigger isMode, CommandXboxController controller)
        {
            return builder(isMode,
                    () -> -1 * controller.getLeftX(),
                    () -> -1 * controller.getLeftY(),
                    () -> -1 * controller.getRightX());
        }

        /**
         * Changes inputs to the given input selection.
         */
        InputStream withChangeInput(InputStructure.BindingType bindingType, Trigger changeInput) {
            isMode.and(changeInput).onTrue(Commands.runOnce(() -> inputOverride.set(bindingType.name)));
            return this;
        }
    }
}