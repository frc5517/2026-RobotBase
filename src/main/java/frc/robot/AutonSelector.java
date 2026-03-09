package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.systems.ScoringSystem;
import frc.robot.util.borrowed.math.AllianceFlipUtil;
import frc.robot.util.borrowed.math.FieldConstants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.SwerveSubsystem.ControlConstants.INITIAL_SIM_POSE;

public class AutonSelector {
    private final SendableChooser<Command> autonSelector;
    private final InputBuilder.Subsystems subsystems;
    private final ScoringSystem scoring;

    AutonSelector(InputBuilder.Subsystems subsystems, ScoringSystem scoring) {
        this.autonSelector = new SendableChooser<>();
        this.subsystems = subsystems;
        this.scoring = scoring;

        autonSelector.setDefaultOption("Score, Collect, Outpost Right", scoreBumpCollectScoreOutPostRight());
        autonSelector.addOption("Score, Collect, Outpost Left", scoreBumpCollectScoreOutPostLeft());

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

    /**
     * Start in front of the right bump then transfer to collect.
     * Then to the outpost while scoring.
     * @return a Right side Auton.
     */
    public Command scoreBumpCollectScoreOutPostRight() {
        // The Starting pose is interpolated between right and left of bump
        Pose2d startingPose = new Pose2d(FieldConstants.LeftBump.nearLeftCorner.interpolate(FieldConstants.LeftBump.nearRightCorner, 0.5)
                .plus(new Translation2d(-.15, -3.15)), new Rotation2d(45)); // Right Bump Constants are broken
        Pose2d outpostPose = new Pose2d(FieldConstants.Outpost.centerPoint, Rotation2d.kCCW_90deg);

        return Commands.runOnce(() -> subsystems.swerve().getSwerveDrive().resetOdometry(AllianceFlipUtil.ifShouldFlip(startingPose)))
                .andThen(scoring.shootOnTheMove().withTimeout(Seconds.of(1.5)))
                .andThen(subsystems.swerve().pathfindToPath("Right Bump"))
                .andThen(subsystems.swerve().driveToNearestFuel()
                        .alongWith(subsystems.intake().intake(0.75, true))
                        .withTimeout(Seconds.of(2)))
                .andThen(subsystems.swerve().driveToPose(() -> AllianceFlipUtil.apply(outpostPose))
                        .alongWith(scoring.shootOnTheMove())
                        .andThen(scoring.shootOnTheMove()));
    }
    /**
     * Start in front of the right bump then transfer to collect.
     * Then to the outpost while scoring.
     * @return a Right side Auton.
     */
    public Command scoreBumpCollectScoreOutPostLeft() {
        // The Starting pose is interpolated between right and left of bump
        Pose2d startingPose = new Pose2d(FieldConstants.LeftBump.nearLeftCorner.interpolate(FieldConstants.LeftBump.nearRightCorner
                .plus(new Translation2d(-0.15, 0)), 0.5), new Rotation2d(-45));
        Pose2d outpostPose = new Pose2d(FieldConstants.Outpost.centerPoint, Rotation2d.kCCW_90deg);

        return Commands.runOnce(() -> subsystems.swerve().getSwerveDrive().resetOdometry(AllianceFlipUtil.ifShouldFlip(startingPose)))
                .andThen(scoring.shootOnTheMove().withTimeout(Seconds.of(1.5)))
                .andThen(subsystems.swerve().pathfindToPath("Left Bump"))
                .andThen(subsystems.swerve().driveToNearestFuel()
                        .alongWith(subsystems.intake().intake(0.75, true))
                        .withTimeout(Seconds.of(2)))
                .andThen(subsystems.swerve().driveToPose(() -> AllianceFlipUtil.apply(outpostPose))
                        .alongWith(scoring.shootOnTheMove())
                        .andThen(scoring.shootOnTheMove()));
    }
}
