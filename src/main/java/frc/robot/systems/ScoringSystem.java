package frc.robot.systems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import frc.robot.subsystems.FlyWheelSubsystem.FlyWheelState;
import frc.robot.subsystems.SwerveSubsystem.SwerveState;
import frc.robot.util.math.AllianceFlipUtil;

import java.awt.geom.Rectangle2D;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.systems.ScoringSystem.ControlConstants.*;
import static frc.robot.systems.ScoringSystem.HardwareConstants.*;

public class ScoringSystem {
    public static class HardwareConstants {
        public static Angle HUB_ENTRY_ANGLE = Degrees.of(35);

        /// Zones
        public static final Rectangle2D SCORING_ZONE = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
                ? new Rectangle2D.Double(1.5, 0.5, 2, 7)
                : new Rectangle2D.Double(14.53 - 1.5, 8.05 -0.5, 2, 7);

    }
    public static final class ControlConstants {
        public static final Trigger SCORING_ZONE_TRIGGER = new Trigger(() -> SCORING_ZONE.contains(SwerveState.CurrentPose.getX(), SwerveState.CurrentPose.getY())); // Tells the robot when we should aim at the hub.
    }

    public ScoringSystem() {
        Telemetry.Publishers.Robot.scoringZonePublisher.accept(new Pose2d[]{ // Publishes a bounding box for the zone.
                new Pose2d(SCORING_ZONE.getMinX(), SCORING_ZONE.getMinY(), Rotation2d.kZero),
                new Pose2d(SCORING_ZONE.getMaxX(), SCORING_ZONE.getMinY(), Rotation2d.kZero),
                new Pose2d(SCORING_ZONE.getMaxX(), SCORING_ZONE.getMaxY(), Rotation2d.kZero),
                new Pose2d(SCORING_ZONE.getMinX(), SCORING_ZONE.getMaxY(), Rotation2d.kZero),});
    }

    public Trigger autoScore(BooleanSupplier enable) {
        return SCORING_ZONE_TRIGGER
                .and(enable)
                .whileTrue(aimAtHub()
                        .alongWith(scoreHub().onlyWhile(FlyWheelState.atDesiredVelocity()))
        );
    }

    public Command aimAtHub() {
        return null;
    }
    public Command scoreHub() {
        return null;
    }
}