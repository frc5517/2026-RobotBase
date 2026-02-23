package frc.robot.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.InputBuilder;
import frc.robot.Robot;
import frc.robot.util.borrowed.math.AllianceFlipUtil;
import frc.robot.util.borrowed.math.FieldConstants;

import java.util.Map;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.systems.ScoringSystem.ControlConstants.*;

public class ScoringSystem {
    public static class HardwareConstants {
        public static Angle HUB_ENTRY_ANGLE = Degrees.of(35);
    }
    public static final class ControlConstants {

        public static final class DoubleInterpolatingMaps {
            // meters, seconds
            public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT = InterpolatingDoubleTreeMap.ofEntries(
                    Map.entry(1.0, .3),
                    Map.entry(4.0, .75));
            // TODO: add more data points here.
            // CLOSE: NEED
            // MID: maybe good enough
            // FAR: NEED

            // meters, RPM
            public static final InterpolatingDoubleTreeMap SHOOT_SPEED = InterpolatingDoubleTreeMap.ofEntries(
                    Map.entry(2.0, 2500.0),
                    Map.entry(3.0, 2700.0),
                    Map.entry(4.0, 3000.0),
                    Map.entry(4.86, 3500.0));

            // meters, degrees
            public static final InterpolatingDoubleTreeMap HOOD_ANGLE = InterpolatingDoubleTreeMap.ofEntries(
                    Map.entry(1.0, 10.0),
                    Map.entry(2.0, 25.0),
                    Map.entry(3.0, 40.0));
        }
    }

    private final InputBuilder.Subsystems subsystems;

    public ScoringSystem(InputBuilder.Subsystems subsystems) {
        this.subsystems = subsystems;
    }

    public Command shootOnTheMove() {
        Angle[] turretGoal = {subsystems.turret().getTurret().getAngle()};
        Angle[] hoodGoal = {subsystems.hood().getHood().getAngle()};
        AngularVelocity[] flyWheelGoal = {subsystems.flywheel().getFlyWheel().getSpeed()};
        return Commands.run(() -> calculateSOTMGoals(
                AllianceFlipUtil.ifShouldFlip(new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero)).getTranslation(),
                (newAngle) -> turretGoal[0] = newAngle,
                (newAngle) -> hoodGoal[0] = newAngle,
                (newSpeed) -> flyWheelGoal[0] = newSpeed))
                .alongWith(subsystems.turret().getTurret().run(() -> turretGoal[0]))
                .alongWith(subsystems.hood().getHood().run(() -> hoodGoal[0]))
                .alongWith(Robot.isSimulation()
                ? subsystems.flywheel().simShoot(() -> flyWheelGoal[0]).andThen(Commands.waitSeconds(0.2)).repeatedly()
                : subsystems.flywheel().getFlyWheel().run(() -> flyWheelGoal[0]));
    }

    public void calculateSOTMGoals(Translation2d target, Consumer<Angle> turretGoal, Consumer<Angle> hoodGoal, Consumer<AngularVelocity> flyWheelGoal) {
        // 2d location of the shooter
        Translation2d shooterOnGround = subsystems.swerve().getSwerveDrive().getPose().getTranslation()
                        .plus(subsystems.turret().getPose3D().toPose2d().getTranslation());
        // Distance to the target
        Distance distanceToTarget = Meters.of(shooterOnGround.getDistance(target));
        // Time that the fuel is in the air.
        double timeOfFlight = DoubleInterpolatingMaps.TIME_OF_FLIGHT.get(distanceToTarget.in(Meters));
        // Chassis speeds times the time of flight.
        ChassisSpeeds speedsAtTime = subsystems.swerve().getSwerveDrive().getFieldVelocity().times(timeOfFlight);
        // Subtract the current speed times the time of flight. Adjusting our goal based on the chassis speeds.
        Translation2d targetAdjustedForSpeed = target.minus(new Translation2d(speedsAtTime.vxMetersPerSecond, speedsAtTime.vyMetersPerSecond));
        // Distance from the updated target based on robot speeds.
        Distance adjustedDistance = Meters.of(targetAdjustedForSpeed.getNorm());
        /// Turret goal is straightforward, angle from origin from our translation with the robot as the origin.
        turretGoal.accept(subsystems.turret().angleToPose(subsystems.swerve().getSwerveDrive().getPose(), targetAdjustedForSpeed).get());
        /// Hood and Flywheel use the Double Interpolating Maps.
        hoodGoal.accept(Degrees.of(DoubleInterpolatingMaps.HOOD_ANGLE.get(adjustedDistance.in(Meters))));
        flyWheelGoal.accept(RPM.of(DoubleInterpolatingMaps.SHOOT_SPEED.get(adjustedDistance.in(Meters))));
    }
}