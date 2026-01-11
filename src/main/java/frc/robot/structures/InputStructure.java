package frc.robot.structures;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;

import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * RobotControlBindings - Centralized control bindings management for robot operation.
 * <p>
 * This class manages all control input bindings for different operator configurations.
 * Each binding method contains its own speed constants to allow for easy customization
 * when creating driver-specific control schemes.
 * <p>
 * Available control schemes:
 * - Single operator with Xbox controller
 * - Dual operator with two Xbox controllers
 * - Single operator with Logitech Extreme 3D Pro joystick
 * - Dual operator with two Logitech Extreme 3D Pro joysticks
 * - Mixed configurations (stick + Xbox)
 * - Test mode bindings
 * <p>
 * The class uses a mode-based binding system where only the selected control scheme
 * is active at any given time, preventing control conflicts between different input devices.
 */
@SuppressWarnings("UnusedReturnValue")
public class InputStructure {

    // Control chooser for dashboard
    private static final SendableChooser<BindingType> inputSelector = new SendableChooser<>();
    // Control input devices
    private final CommandXboxController driverXbox;
    private final CommandXboxController operatorXbox;
    private final CommandJoystick driverRightStick;  // Flight Stick
    private final CommandJoystick driverLeftStick; // Flight Stick again
    // Robot subsystems

    private StringPublisher inputOverride;

    /**
     * Constructs a new RobotControlBindings instance.
     */
    public InputStructure(

    ) {

        this.driverXbox = new CommandXboxController(0);
        this.operatorXbox = new CommandXboxController(1);
        this.driverRightStick = new CommandJoystick(0);
        this.driverLeftStick = new CommandJoystick(1);
    }

    /**
     * Initializes all control bindings and the control chooser.
     * This method should be called once during robot initialization.
     */
    public void init() {
        // Send the inputSelector to the dashboard.
        Telemetry.Publishers.Robot.inputPublisher.setValue(inputSelector);

        // Initialize all standard binding configurations
        singleXboxBindings();
        dualXboxBindings();
        testBindings();

        // Add custom driver bindings here
        // Example: xAndYBindings();

        // Log initialization
        DriverStation.reportWarning("Robot control bindings initialized successfully", false);
    }

    /**
     * Single Xbox controller bindings for solo operation.
     * All robot controls are mapped to one controller with modifier buttons for speed control.
     */
    private void singleXboxBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = BindingType.SINGLE_XBOX.isMode;
        new ControlStream(
                () -> -1 * driverXbox.getLeftX(),
                () -> -1 * driverXbox.getLeftY(),
                () -> -1 * driverXbox.getRightX(),
                isMode);
    }

    /**
     * Dual Xbox controller bindings for driver-operator configuration.
     * Split responsibilities between driver and operator for maximum efficiency.
     */
    private void dualXboxBindings() {
        Trigger isMode = BindingType.DUAL_XBOX.isMode;

        // Configure driver Xbox controls using helper method
        typicalDriverXboxControls(isMode);
        // Configure operator Xbox controls using helper method
        typicalOperatorXboxControls(isMode);
    }

    /**
     * Test mode bindings for debugging and system identification.
     * Provides direct control over subsystems for testing and calibration.
     * <p>
     * WARNING: Test mode bypasses safety interlocks. Use with caution.
     */
    private void testBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = BindingType.TESTING.isMode;
        new ControlStream(
                () -> -1 * driverXbox.getLeftX(),
                () -> -1 * driverXbox.getLeftY(),
                () -> -1 * driverXbox.getRightX(),
                isMode);
    }

    /**  Define Student Binding Methods here  */

    private void newStudentBinding() {
        Trigger isMode = BindingType.NEWSTUDENTBINDINGTYPE.isMode;
        new ControlStream(isMode);
    }

    /**
     * Helper method to configure typical driver Xbox controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private ControlStream typicalDriverXboxControls(Trigger isMode) {
        return new ControlStream(
                () -> -1 * driverXbox.getLeftX(),
                () -> -1 * driverXbox.getLeftY(),
                () -> -1 * driverXbox.getRightX(),
                isMode);
    }

    /**
     * Helper method to configure typical operator Xbox controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private ControlStream typicalOperatorXboxControls(Trigger isMode) {
        return new ControlStream(isMode);
    }

    /**
     * Helper method to configure typical driver single stick controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private ControlStream typicalSingleStickControls(Trigger isMode) {
        return new ControlStream(
                () -> -1 * driverRightStick.getY(),
                () -> -1 * driverRightStick.getX(),
                () -> -1 * driverRightStick.getTwist(),
                isMode);

        // Single stick bindings <!>
    }

    /**
     * Helper method to configure typical driver dual stick controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private ControlStream typicalDualStickControls(Trigger isMode) {
        return new ControlStream(
                () -> -1 * driverRightStick.getY(),
                () -> -1 * driverRightStick.getX(),
                () -> -1 * driverLeftStick.getTwist(),
                isMode);

        // Dual stick bindings <!>
    }

    /**
     * Gets the currently selected binding type.
     *
     * @return The active BindingType
     */
    public BindingType getCurrentBindingType() {
        return inputSelector.getSelected();
    }

    /**
     * Gets the control chooser for external access if needed.
     *
     * @return The SendableChooser for control selection
     */
    public SendableChooser<BindingType> getControlChooser() {
        return inputSelector;
    }

    // Control binding type enum
    public enum BindingType {
        /*
        All controls using a single xbox controller.
         */
        SINGLE_XBOX("Single Xbox", true),
        /*
        Classic driver and operator setup on two xbox controllers.
         */
        DUAL_XBOX("Dual Xbox"),
        /*
        All controls on a single joystick.
         */
        SINGLE_STICK("Single Stick"),
        /*
        All controls on two joysticks one driver.
         */
        DUAL_STICK("Dual Stick"),
        /*
        Classic driver and operator with the driver using a single joystick.
         */
        SINGLE_STICK_XBOX("Single Stick and Xbox"),
        /*
        Classic driver and operator with the driver using dual joysticks.
         */
        DUAL_STICK_XBOX("Dual Stick and Xbox"),
        /*
        Control mode used for testing controls subject to constant change
         */
        TESTING("Testing"),

        /**  Define Student BindingTypes here  */

        NEWSTUDENTBINDINGTYPE("New Student Binding Type");

        // BindType Name
        public final String name;
        public final Trigger isMode;

        BindingType(String name, boolean isDefault) {
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
        BindingType(String name) {
            this(name, false);
        }
    }


    // Warning suppression to clean up, suppressions are as follows.
    // {Optionals always give a warning, Some methods warn when all parts are not used, this is for when a method doesn't chain into another}
    @SuppressWarnings({"OptionalUsedAsFieldOrParameterType", "SameParameterValue", "UnusedReturnValue"})
    private class ControlStream {
        /* Trigger used to enable all controls in this Control Stream class */
        protected Optional<Trigger> isMode;
        /* Drive train control constants */
        protected Optional<Double> SLOW_TRANSLATION;
        protected Optional<Double> SLOW_ROTATION;
        protected Optional<Double> NORMAL_TRANSLATION;
        protected Optional<Double> NORMAL_ROTATION;
        protected Optional<Double> BOOST_TRANSLATION;
        protected Optional<Double> BOOST_ROTATION;
        //protected Optional<SwerveInputStream> inputStream; TODO
        protected Optional<Command> driveCommand;
        /* Subsystem control constants */
        protected Optional<Double> ELEVATOR_SPEED;
        protected Optional<Double> ARM_SPEED;

        /**
         * Input stream used to streamline the construction of custom input profiles.
         *
         * @param isMode {@link Trigger} used to determine when input should be allowed.
         */
        public ControlStream(Trigger isMode) {
            /* Trigger used to enable all controls in this Control Stream class */
            this.isMode = Optional.of(isMode);
            /* Drive train control constants */
            this.SLOW_TRANSLATION = Optional.of(0.3);
            this.SLOW_ROTATION = Optional.of(0.2);
            this.NORMAL_TRANSLATION = Optional.of(0.8);
            this.NORMAL_ROTATION = Optional.of(0.6);
            this.BOOST_TRANSLATION = Optional.of(1.0);
            this.BOOST_ROTATION = Optional.of(0.75);
            /* Drive Controls not defined, I'll still define default drive variables for adding input streams after init. */
            /* Subsystem control constants */
//            this.ELEVATOR_SPEED = Optional.of(Elevator.ControlConstants.kElevatorSpeed); TODO
//            this.ARM_SPEED = Optional.of(Arm.ControlConstants.kArmSpeed);
        }

        /**
         * Input stream used to streamline the construction of custom input profiles.
         *
         * @param isMode {@link Trigger} used to determine when input should be allowed.
         */
        public ControlStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, Trigger isMode) {
            this(isMode);
//            this.inputStream = Optional.of(SwerveInputStream.of( TODO drive stuff
//                            swerve.getSwerveDrive(),
//                            x, y)
//                    .cubeTranslationControllerAxis(true)
//                    .withControllerRotationAxis(rotation)
//                    .deadband(Constants.OperatorConstants.DEADBAND)
//                    .scaleTranslation(NORMAL_TRANSLATION.get())
//                    .scaleRotation(NORMAL_ROTATION.get())
//                    .robotRelative(true)
//                    .allianceRelativeControl(false)
//                    .translationHeadingOffset(Rotation2d.k180deg));
//            updateDriveCommand();
//            // Set default drive command when enabled
//            if (driveCommand.isPresent()) {
//                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(getDriveCommand())));
//            }
        }

        ControlStream withChangeInput(BindingType bindingType, Trigger changeInput) {
            if (isMode.isPresent()) {
                isMode.get().and(changeInput).onTrue(Commands.runOnce(() -> {
                    inputOverride.set(bindingType.name);
                }));
            } else {
                DriverStation.reportWarning("isMode not found, Change Input failed.", true);
            }

            return this;
        }

        /*  Drive Train Controls  */

//        /**
//         * Slows drive train speed.
//         *
//         * @param shouldSlow button mapping {@link Trigger} to use.
//         * @return {@link InputStructure} for chaining.
//         */
//        ControlStream withSlowTranslation(Trigger shouldSlow) {
//            if (isMode.isPresent() && inputStream.isPresent()
//                    && SLOW_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
//                isMode.get().and(shouldSlow).whileTrue(Commands.runEnd(
//                        () -> inputStream.get().scaleTranslation(SLOW_TRANSLATION.get()),
//                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
//            } else {
//                DriverStation.reportWarning("Something not found, Slow Translation failed.", true);
//            }
//            return this;
//        }
//
//        /**
//         * Boosts drive train speed.
//         *
//         * @param shouldBoost button mapping {@link Trigger} to use.
//         * @return {@link InputStructure} for chaining.
//         */
//        ControlStream withBoostTranslation(Trigger shouldBoost) {
//            if (isMode.isPresent() && inputStream.isPresent()
//                    && BOOST_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
//                isMode.get().and(shouldBoost).whileTrue(Commands.runEnd(
//                        () -> inputStream.get().scaleTranslation(BOOST_TRANSLATION.get()),
//                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
//            } else {
//                DriverStation.reportWarning("Something not found, Boost Translation failed.", true);
//            }
//            return this;
//        }
//
//        /**
//         * Method to switch turning the heading offset on and off.
//         * Heading offset can be set with setHeadingOffset(Angle)
//         * {Default : 180 degrees}
//         *
//         * @param shouldOffset button mapping {@link Trigger} to use.
//         * @return {@link InputStructure} for chaining.
//         */
//        ControlStream withHeadingOffset(Trigger shouldOffset) {
//            if (isMode.isPresent() && inputStream.isPresent()
//                    && BOOST_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
//                isMode.get().and(shouldOffset).whileTrue(Commands.runEnd(
//                        () -> inputStream.get().scaleTranslation(BOOST_TRANSLATION.get()),
//                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
//            } else {
//                DriverStation.reportWarning("Something not found, Heading offset failed.", true);
//            }
//            return this;
//        }
//
//        /**
//         * Method to switch between field and robot centric drives.
//         *
//         * @param shouldToggleCentricity button mapping {@link Trigger} to use.
//         * @return {@link InputStructure} for chaining.
//         */
//        ControlStream withToggleCentricity(Trigger shouldToggleCentricity) {
//            if (isMode.isPresent() && inputStream.isPresent()) {
//                isMode.get().and(shouldToggleCentricity).toggleOnTrue(Commands.runEnd(
//                        () -> inputStream.get().robotRelative(false).allianceRelativeControl(true),
//                        () -> inputStream.get().robotRelative(true).allianceRelativeControl(false)
//                ));
//            } else {
//                DriverStation.reportWarning("Something not found, Toggle Centricity failed.", true);
//            }
//            return this;
//        }
//
//        /**
//         * Updates the driveCommand from the latest {@link SwerveInputStream}.
//         *
//         * @return {@link InputStructure} for chaining.
//         */
//        ControlStream updateDriveCommand() {
//            if (inputStream.isPresent()) {
//                this.driveCommand = Optional.of(swerve.driveFieldOriented(inputStream.get()));
//                swerve.setDefaultCommand(driveCommand.get());
//            } else {
//                DriverStation.reportWarning("Input stream not found, updateDriveCommand failed.", false);
//            }
//            return this;
//        }
//
//        /**
//         * Method to set the heading offset.
//         * Offset is then enabled by using withHeadingOffset(Trigger)
//         *
//         * @param headingOffset heading offset angle.
//         * @return {@link InputStructure} for chaining.
//         */
//        ControlStream setHeadingOffset(Angle headingOffset) {
//            if (inputStream.isPresent()) {
//                inputStream.get().translationHeadingOffset(new Rotation2d(headingOffset));
//            } else {
//                DriverStation.reportWarning("Input Stream not found, setting Heading Offset failed.", true);
//            }
//            return this;
//        }
//
//        /**
//         * Gets the drive command.
//         *
//         * @return the {@link SwerveInputStream} or if not found will report a warning and return null.
//         */
//        Command getDriveCommand() {
//            Command command = driveCommand.orElse(null);
//            if (command == null) {
//                DriverStation.reportWarning("Drive Command is null", false);
//                return null;
//            } else {
//                return command;
//            }
//        }
//
//        /**
//         * Changes the default drive command.
//         * This just sets the commands as the SwerveSubsystem default command.
//         *
//         * @param driveCommand {@link Command} to use.
//         * @return {@link InputStructure} for chaining.
//         */
//        ControlStream setDriveCommand(Command driveCommand) {
//            this.driveCommand = Optional.of(driveCommand);
//            updateDriveCommand();
//            return this;
//        }

//        /**
//         * Gets the input stream.
//         *
//         * @return the {@link SwerveInputStream} or if not found will report a warning and return null.
//         */
//        SwerveInputStream getInputStream() {
//            SwerveInputStream stream = inputStream.orElse(null);
//            if (stream == null) {
//                DriverStation.reportWarning("Input stream is null", false);
//                return null;
//            } else {
//                return stream;
//            }
//        }
//
//        /**
//         * Changes the {@link ControlStream}'s {@link SwerveInputStream} for controlling the drive train.
//         *
//         * @param inputStream {@link SwerveInputStream} to use.
//         * @return {@link InputStructure} for chaining.
//         */
//        ControlStream setInputStream(SwerveInputStream inputStream) {
//            this.inputStream = Optional.of(inputStream);
//            return this;
//        }


        /**
         * Speed to use to control the elevator when not defining a speed.
         *
         * @param elevatorSpeed Duty cycle speed to use. {-0.0, 1.0}
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setElevatorSpeed(double elevatorSpeed) {
            if (ELEVATOR_SPEED.isPresent()) {
                this.ELEVATOR_SPEED = Optional.of(MathUtil.clamp(elevatorSpeed, -0.0, 1.0));
            } else {
                DriverStation.reportWarning("Elevator Speed is invalid.", true);
            }
            return this;
        }

        /**
         * Speed to use to control the arm when not defining a speed.
         *
         * @param armSpeed Duty cycle speed to use. {-0.0, 1.0}
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setArmSpeed(double armSpeed) {
            if (ARM_SPEED.isPresent()) {
                this.ARM_SPEED = Optional.of(MathUtil.clamp(armSpeed, -0.0, 1.0));
            } else {
                DriverStation.reportWarning("Arn Speed is invalid.", true);
            }
            return this;
        }
    }
}