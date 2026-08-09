package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.systems.ScoringSystem;
import frc.robot.util.borrowed.math.AllianceFlipUtil;
import frc.robot.util.borrowed.math.FieldConstants;

public class AutonSelector {
    private final SendableChooser<Command> autonSelector;
    private final InputBuilder.Subsystems subsystems;
    private final ScoringSystem scoring;

    private static class StartingPoses {
        private static final Pose2d RIGHT_BUMP = new Pose2d(FieldConstants.LeftBump.nearLeftCorner.interpolate(FieldConstants.LeftBump.nearRightCorner, 0.5)
                .plus(new Translation2d(-.15, -3.15)), new Rotation2d(45)); // Right Bump Constants are broken
        private static final Pose2d LEFT_BUMP = new Pose2d(FieldConstants.LeftBump.nearLeftCorner.interpolate(FieldConstants.LeftBump.nearRightCorner
                .plus(new Translation2d(-0.15, 0)), 0.5), new Rotation2d(-45));
    }

    AutonSelector(InputBuilder.Subsystems subsystems, ScoringSystem scoring) {
        NamedCommands.registerCommand("Shoot Average", scoring.shootAverage());
        this.autonSelector = AutoBuilder.buildAutoChooser();
        this.subsystems = subsystems;
        this.scoring = scoring;

        autonSelector.setDefaultOption("Right to Left - Collect then Outpost", rightToLeftOutpost());
        autonSelector.addOption("Left to Right - Collect then Outpost", leftToRightOutpost());
        autonSelector.addOption("Do Nothing", doNothing());
        autonSelector.addOption("Other", other());

        SmartDashboard.putData("AutonSelector", autonSelector);
    }

    /**
     * Gets the selected auton {@link Command}.
     *
     * @return the auton {@link Command} to run.
     */
    public Command getSelected() {
        return autonSelector.getSelected();
    }

    public Command doNothing() {
        return Commands.none();
    }

    public Command other() {
        return Commands.runOnce(
                        // Update Odometry
                        () -> subsystems.swerve().getSwerveDrive().resetOdometry(AllianceFlipUtil.ifShouldFlip(StartingPoses.RIGHT_BUMP)))
                // Then move to outpost while firing.
                .andThen(subsystems.swerve().pathfindToPath("Left to Hub Front"))
                // While firing
                .alongWith(subsystems.flywheel().getFlyWheel().run(RotationsPerSecond.of(50)))
                .alongWith(subsystems.hood().getHood().setAngle(Degrees.of(15)))
                .alongWith(subsystems.kicker().runKicker(.75, true))
                // Start feeding after a second
                .alongWith(Commands.waitSeconds(1).andThen(
                    //subsystems.agitator().runAgitator(0.75, true)
                    subsystems.indexer().runIndexer(.75, true)
                    .alongWith(subsystems.intake().runIntake(.75, true))));
    }

    /**
     * Start in front of the right bump then transfer to collect.
     * Then to the outpost while scoring.
     *
     * @return a Right side Auton.
     */
    public Command rightToLeftOutpost() {
        return Commands.runOnce(
                        // Update Odometry
                        () -> subsystems.swerve().getSwerveDrive().resetOdometry(AllianceFlipUtil.ifShouldFlip(StartingPoses.RIGHT_BUMP)))
                // Cross bump
                .andThen(subsystems.swerve().pathfindToPath("Right Bump Neutral"))
                // Fast Collect
                .andThen(subsystems.swerve().pathfindToPath("Right to Left Centerline")
                        // While intaking, stopping when path is over.
                        .deadlineFor(subsystems.intake().intake(0.75, true)))
                // Then move to outpost while firing.
                .andThen(subsystems.swerve().pathfindToPath("Left to Hub Front"))
                .alongWith(subsystems.kicker().runKicker(.75, true))
                // Start feeding after a second
                .alongWith(Commands.waitSeconds(1).andThen(
                    //subsystems.agitator().runAgitator(0.75, true)
                    subsystems.indexer().runIndexer(.75, true)
                    .alongWith(subsystems.intake().runIntake(.75, true))));
    }

    /**
     * Start in front of the left bump then transfer to collect.
     * Then to the outpost while scoring.
     *
     * @return a Right side Auton.
     */
    public Command leftToRightOutpost() {
        return Commands.runOnce(
                        // Update Odometry
                        () -> subsystems.swerve().getSwerveDrive().resetOdometry(AllianceFlipUtil.ifShouldFlip(StartingPoses.LEFT_BUMP)))
                // Cross bump
                .andThen(subsystems.swerve().pathfindToPath("Left Bump Neutral"))
                // Fast Collect
                .andThen(subsystems.swerve().pathfindToPath("Left to Right Centerline")
                        // While intaking, stopping when path is over.
                        .deadlineFor(subsystems.intake().intake(0.75, true)))
                // Then move to outpost while firing.
                .andThen(subsystems.swerve().pathfindToPath("Right to Hub Front"))
                .alongWith(subsystems.kicker().runKicker(.75, true))
                // Start feeding after a second
                .alongWith(Commands.waitSeconds(1).andThen(
                    //subsystems.agitator().runAgitator(0.75, true)
                    subsystems.indexer().runIndexer(.75, true)
                    .alongWith(subsystems.intake().runIntake(.75, true))));
    }
}