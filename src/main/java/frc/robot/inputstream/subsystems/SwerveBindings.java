//package frc.robot.inputstream.subsystems;
//
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.inputstream.InputStream;
//import frc.robot.util.borrowed.math.AllianceFlipUtil;
//import frc.robot.util.borrowed.math.FieldConstants;
//import lombok.Getter;
//import lombok.Setter;
//
//import java.util.Objects;
//import java.util.function.Supplier;
//
//import static edu.wpi.first.units.Units.Degrees;
//import static frc.robot.RobotContainer.subsystems;
//
///**
// * Swerve Drive Button Bindings
// */
//public class SwerveBindings {
//    /**
//     * Checks if this subsystem is present, if not, don't bind anything.
//     */
//    @Getter
//    private final boolean isPresent;
//    /**
//     * Controller Dead Zone
//     */
//    @Setter
//    private double deadzone = 0.001;
//    /**
//     * Move Speed Scalar when slowing drive speed.
//     */
//    @Setter
//    private double slowTranslation = 0.3;
//    /**
//     * Rotation Speed Scalar when slowing drive speed.
//     */
//    @Setter
//    private double slowRotation = 0.4;
//    /**
//     * Move Speed Scalar when driving normally.
//     */
//    @Setter
//    private double normalTranslation = .6;
//    /**
//     * Rotation Speed Scalar when driving normally.
//     */
//    @Setter
//    private double normalRotation = .6;
//    /**
//     * Move Speed Scalar when boosting drive speed.
//     */
//    @Setter
//    private double boostTranslation = 0.8;
//    /**
//     * Rotation Speed Scalar when boosting drive speed.
//     */
//    @Setter
//    private double boostRotation = 0.85;
//
//    /**
//     * The {@link InputStream} to attach to.
//     */
//    private final InputStream inputStream;
//
//    public SwerveBindings(InputStream stream) {
//        this.inputStream = Objects.requireNonNull(stream);
//        this.isPresent = subsystems.swerve() != null;
//    }
//
//    /**
//     * Changes inputs to the given input selection.
//     */
//    public SwerveBindings withToggleCentricity(Trigger toggleCentricity, boolean fieldDefault) {
//        if (!isPresent) {
//            return this;
//        }
//        // Set our default.
//        inputStream.getSwerveInputStream().robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault);
//        // Toggle when pressed.
//        inputStream.getIsMode().and(toggleCentricity).toggleOnTrue(Commands.runEnd(
//                () -> inputStream.getSwerveInputStream().robotRelative(fieldDefault).allianceRelativeControl(!fieldDefault),
//                () -> inputStream.getSwerveInputStream().robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault)));
//        return this;
//    }
//
//    /**
//     * Automatically swap between the Neutral and Alliance zone crossing tbe bump.
//     *
//     * @param toggleZones the button to map.
//     * @return this, for chaining.
//     */
//    public SwerveBindings withToggleZones(Trigger toggleZones) {
//        if (!isPresent) {
//            return this;
//        }
//        inputStream.getIsMode().and(toggleZones).whileTrue(subsystems.swerve().toggleZones());
//        return this;
//    }
//
//    // TODO Remove and put into 2026 bindings
//    /**
//     * Looks at the hub with the drive. Doesn't account for distance.
//     *
//     * @param autoAimDriveThenFireWhile the button to map.
//     * @return this, for chaining.
//     */
//    public SwerveBindings withLookAtHub(Trigger autoAimDriveThenFireWhile) {
//        if (!isPresent) {
//            return this;
//        }
//        // Aim at the hub with the drive
//        this.withAimWhile(autoAimDriveThenFireWhile,
//                new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero)); // Leave look ahead time alone.
//        return this;
//    }
//
//    /**
//     * Aims at a pose while held.
//     *
//     * @param aimWhile     the button to map.
//     * @param aimWhilePose the pose to aim at.
//     * @return this, for chaining.
//     */
//    public SwerveBindings withAimWhile(Trigger aimWhile, Pose2d aimWhilePose) {
//        if (!isPresent) {
//            return this;
//        }
//        // Update Telemetry Continuously
//        inputStream.getIsMode().and(DriverStation::isEnabled).whileTrue(Commands.run(() -> {
//            SmartDashboard.putBoolean("Telemetry/RobotTelemetry/Swerve/Drive is Aimed", inputStream.getSwerveInputStream().aimLock(Degrees.of(1)).getAsBoolean());
//        }));
//        // Aim while held
//        inputStream.getIsMode().and(aimWhile).onTrue(Commands.runOnce(() -> {
//            // Update our pose and aim supplier when inputStream.getIsMode() and aimWhile.
//            inputStream.getSwerveInputStream().aim(AllianceFlipUtil.ifShouldFlip(aimWhilePose));
//            inputStream.getSwerveInputStream().aimWhile(inputStream.getIsMode().and(aimWhile));
//        }));
//        // Return this for chaining.
//        return this;
//    }
//
//    /**
//     * Holding the button slows the drive.
//     *
//     * @param slowDrive the button to map.
//     * @return this, for chaining.
//     */
//    public SwerveBindings withSlowDrive(Trigger slowDrive) {
//        if (!isPresent) {
//            return this;
//        }
//        inputStream.getIsMode().and(slowDrive)
//                .onTrue(Commands.runOnce(() -> inputStream.getSwerveInputStream().scaleTranslation(slowTranslation).scaleRotation(slowRotation)))
//                .onFalse(Commands.runOnce(() -> inputStream.getSwerveInputStream().scaleTranslation(normalTranslation).scaleRotation(normalRotation)));
//        return this;
//    }
//
//    /**
//     * Holding the button speeds up the drive.
//     *
//     * @param boostDrive the button to map.
//     * @return this, for chaining.
//     */
//    public SwerveBindings withBoostDrive(Trigger boostDrive) {
//        if (!isPresent) {
//            return this;
//        }
//        inputStream.getIsMode().and(boostDrive)
//                .onTrue(Commands.runOnce(() -> inputStream.getSwerveInputStream().scaleTranslation(boostTranslation).scaleRotation(boostRotation)))
//                .onFalse(Commands.runOnce(() -> inputStream.getSwerveInputStream().scaleTranslation(normalTranslation).scaleRotation(normalRotation)));
//        return this;
//    }
//
//    /**
//     * Resets the robot odometry. Zeros the gyro. Updates pose to real pose in sim.
//     *
//     * @param resetOdometry the button to map.
//     * @return this, for chaining.
//     */
//    public SwerveBindings withResetSimOdometry(Trigger resetOdometry) {
//        if (!isPresent) {
//            return this;
//        }
//        inputStream.getIsMode().and(resetOdometry).onTrue(Commands.runOnce(() -> subsystems.swerve().getSwerveDrive().resetOdometry(subsystems.swerve().getSwerveDrive().getSimulationDriveTrainPose().get())));
//        return this;
//    }
//
//    /**
//     * Sets a default command for this subsystem.
//     *
//     * @param defaultCommand the default command to set.
//     * @return this, for chaining.
//     */
//    public SwerveBindings withDefaultCommand(Supplier<Command> defaultCommand) {
//        if (!isPresent) {
//            return this;
//        }
//        inputStream.getIsMode().and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystems.swerve().setDefaultCommand(defaultCommand.get())));
//        return this;
//    }
//
//    /**
//     * Leaves SwerveBindings going back to the InputStream.
//     *
//     * @return this InputStream.
//     */
//    public InputStream back() {
//        /// Update things here, also runs on binding entry.
//        if (Objects.nonNull(inputStream.getSwerveInputStream())) {
//            inputStream.getSwerveInputStream()
//                    .scaleTranslation(normalTranslation)
//                    .scaleRotation(normalRotation)
//                    .deadband(deadzone);
//        }
//        return this.inputStream;
//    }
//}
