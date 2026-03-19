package frc.robot.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.InputBuilder;
import frc.robot.Robot;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.systems.ScoringSystem.ControlConstants.DoubleInterpolatingMaps;
import frc.robot.util.borrowed.math.AllianceFlipUtil;
import frc.robot.util.borrowed.math.FieldConstants;
import lombok.Getter;
import lombok.Setter;

import javax.swing.*;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ScoringSystem {
    public static class HardwareConstants {
        public static Angle HUB_ENTRY_ANGLE = Degrees.of(35);
    }
    public static final class ControlConstants {

        public static final class DoubleInterpolatingMaps {
            // meters, seconds
            public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT = InterpolatingDoubleTreeMap.ofEntries(
                    Map.entry(1.0, .4),
                    Map.entry(3.0, .55));

            // meters, RPM
            public static final InterpolatingDoubleTreeMap SHOOT_SPEED = InterpolatingDoubleTreeMap.ofEntries(
                    Map.entry(2.0, 2500.0),
                    Map.entry(3.0, 2750.0),
                    Map.entry(4.0, 3500.0));

            // meters, degrees
            public static final InterpolatingDoubleTreeMap HOOD_ANGLE = InterpolatingDoubleTreeMap.ofEntries(
                    Map.entry(1.0, 8.0),
                    Map.entry(2.0, 24.0),
                    Map.entry(3.0, 35.0));
        }

        public static final class SOTMTargets {
            public static final Pose2d HUB = new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero);
        }
    }

    public static class SOTMLatestGoals {
        @Getter
        private static Supplier<Angle> TurretGoal = () -> Degrees.of(0);
        @Getter
        private static Supplier<Angle> HoodGoal = () -> Degrees.of(0);
        @Getter
        private static Supplier<AngularVelocity> FlyWheelGoal = () -> RPM.of(0);
        @Getter
        private static Supplier<Time> TimeOfFlight = () -> Seconds.of(0);
        @Getter
        private static Supplier<Distance> DistanceToTarget = () -> Meters.of(0);
    }

    private final InputBuilder.Subsystems subsystems;

    public ScoringSystem(InputBuilder.Subsystems subsystems) {
        this.subsystems = subsystems;
    }

    public Command shootOnTheMove(Supplier<Pose2d> target) {
        // Goal values
        Angle[] turretGoal = {subsystems.turret().getTurret().getAngle()};
        Angle[] hoodGoal = {subsystems.hood().getHood().getAngle()};
        AngularVelocity[] flyWheelGoal = {subsystems.flywheel().getFlyWheel().getSpeed()};
        return Commands.run(() -> calculateSOTMGoals(
                // Calculate goals based on a flipped target
                AllianceFlipUtil.ifShouldFlip(target.get()).getTranslation(),
                (newAngle) -> turretGoal[0] = newAngle,
                (newAngle) -> hoodGoal[0] = newAngle,
                (newSpeed) -> flyWheelGoal[0] = newSpeed))
                // Run goals
                .alongWith(subsystems.turret().getTurret().run(() -> turretGoal[0]))
                .alongWith(subsystems.hood().getHood().run(() -> hoodGoal[0]))
                .alongWith(RobotBase.isSimulation()
                ? subsystems.flywheel().simShoot(subsystems, () -> flyWheelGoal[0]).andThen(Commands.waitSeconds(.2)).repeatedly()
                        : subsystems.flywheel().setGoal(() -> flyWheelGoal[0]));
    }

    public void calculateSOTMGoals(Translation2d target, Consumer<Angle> turretGoal, Consumer<Angle> hoodGoal, Consumer<AngularVelocity> flyWheelGoal) {
        // Velocity measurement delay that affects precise calculations.
        double phaseDelay = 0.05;
        Pose2d rawPose = subsystems.swerve().getSwerveDrive().getPose();
        // 2d location of the shooter
        Translation2d shooterOnGround = rawPose.getTranslation()
                        .plus(subsystems.turret().getPose3D().toPose2d().getTranslation());
        // Distance to the target
        Distance distanceToTarget = Meters.of(shooterOnGround.getDistance(target));
        // Time that the fuel is in the air.
        double timeOfFlight = DoubleInterpolatingMaps.TIME_OF_FLIGHT.get(distanceToTarget.in(Meters));
        // Chassis speeds times the time of flight.
        ChassisSpeeds speedsAtTime = subsystems.swerve().getSwerveDrive().getFieldVelocity().times(timeOfFlight);
        // Estimate the robot pose based on phase delay
        Pose2d estimatedPose = rawPose.exp(new Twist2d(
            speedsAtTime.vxMetersPerSecond * phaseDelay, 
            speedsAtTime.vyMetersPerSecond * phaseDelay, 
            speedsAtTime.omegaRadiansPerSecond * phaseDelay));
        // Adjust the speeds to be turret relative
        ChassisSpeeds turretSpeedsAtTime = subsystems.turret().getVelocity(speedsAtTime, estimatedPose.getRotation().getMeasure());
        // Subtract the current speed times the time of flight. Adjusting our goal based on the chassis speeds. 
        Translation2d targetAdjustedForSpeed = target.minus(new Translation2d(turretSpeedsAtTime.vxMetersPerSecond, turretSpeedsAtTime.vyMetersPerSecond));
        // Distance from the updated target based on robot speeds.
        Distance adjustedDistance = Meters.of(targetAdjustedForSpeed.getNorm());
        /// Set Latest SOTMGoals
        /// Turret goal is straightforward, angle from origin from our translation with the robot as the origin.
        SOTMLatestGoals.TurretGoal = () -> subsystems.turret().angleToPose(subsystems.swerve().getSwerveDrive().getPose(), targetAdjustedForSpeed).get();
        /// Hood and Flywheel use the Double Interpolating Maps.
        SOTMLatestGoals.HoodGoal = () -> Degrees.of(DoubleInterpolatingMaps.HOOD_ANGLE.get(adjustedDistance.in(Meters)));
        SOTMLatestGoals.FlyWheelGoal = () -> RPM.of(DoubleInterpolatingMaps.SHOOT_SPEED.get(adjustedDistance.in(Meters)));
        /// Add other good data
        SOTMLatestGoals.TimeOfFlight = () -> Seconds.of(timeOfFlight);
        SOTMLatestGoals.DistanceToTarget = () -> adjustedDistance;
        // They want consumers I guess.
        turretGoal.accept(SOTMLatestGoals.getTurretGoal().get());
        hoodGoal.accept(SOTMLatestGoals.getHoodGoal().get());
        flyWheelGoal.accept(SOTMLatestGoals.getFlyWheelGoal().get());
    }

    public Trigger isReadyToFire() {
        return new Trigger(subsystems.turret().isNear(SOTMLatestGoals.getTurretGoal())).and(subsystems.hood().isNear(SOTMLatestGoals.getHoodGoal()));
    }
}