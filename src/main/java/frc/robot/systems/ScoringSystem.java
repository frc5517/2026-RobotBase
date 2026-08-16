package frc.robot.systems;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.InputBuilder;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.systems.ScoringSystem.ControlConstants.DoubleInterpolatingMaps;
import frc.robot.util.borrowed.math.AllianceFlipUtil;
import frc.robot.util.borrowed.math.FieldConstants;
import lombok.Getter;
import swervelib.SwerveInputStream;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ScoringSystem {
    public static class HardwareConstants {
        public static Angle HUB_ENTRY_ANGLE = Degrees.of(35);
    }

    public static final class ControlConstants {

        public static final class DoubleInterpolatingMaps {
            private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngle =
                    new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
            private static final InterpolatingDoubleTreeMap flyWheelSpeed =
                    new InterpolatingDoubleTreeMap();
            private static final InterpolatingDoubleTreeMap timeOfFlight =
                    new InterpolatingDoubleTreeMap();

            static {
                /// TODO : THESE ARE JUST STOLEN VALUES
                hoodAngle.put(1.34, Rotation2d.fromDegrees(11.0));
                hoodAngle.put(1.78, Rotation2d.fromDegrees(12.0));
                hoodAngle.put(2.17, Rotation2d.fromDegrees(22.0));
                hoodAngle.put(2.81, Rotation2d.fromDegrees(23.0));
                hoodAngle.put(3.82, Rotation2d.fromDegrees(24.0));
                hoodAngle.put(4.09, Rotation2d.fromDegrees(25.0));
                hoodAngle.put(4.40, Rotation2d.fromDegrees(26.0));
                hoodAngle.put(4.77, Rotation2d.fromDegrees(27.0));
                hoodAngle.put(5.57, Rotation2d.fromDegrees(28.0));
                hoodAngle.put(5.60, Rotation2d.fromDegrees(30.0));

                flyWheelSpeed.put(1.5, 2725.0);
                flyWheelSpeed.put(2.504222, 2800.0);
                flyWheelSpeed.put(2.889, 2950.0);
                flyWheelSpeed.put(3.254686, 3100.0);
                flyWheelSpeed.put(3.695324, 3200.0);
                flyWheelSpeed.put(3.983757, 3300.0);
                flyWheelSpeed.put(4.498437, 3400.0);
                flyWheelSpeed.put(4.986071, 3500.0);
                flyWheelSpeed.put(5.410986, 3700.0);

                timeOfFlight.put(1.50, 0.8);
                timeOfFlight.put(2.00, 0.8);
                timeOfFlight.put(2.50, 0.85);
                timeOfFlight.put(3.50, 0.9);
                timeOfFlight.put(4.00, 0.9);
                timeOfFlight.put(4.50, 1.0);
                timeOfFlight.put(5.00, 1.0);
                timeOfFlight.put(5.50, 1.1);
                }
        }

        public static final class SOTMTargets {
            public static final Pose2d HUB = new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero);
        }
    }

    public static class SOTMLatestGoals {
        @Getter
        private static Angle HeadingGoal = Degrees.of(0);
        @Getter
        private static Angle HoodGoal = Degrees.of(0);
        @Getter
        private static AngularVelocity FlyWheelGoal = RPM.of(0);
        @Getter
        private static Time TimeOfFlight = Seconds.of(0);
        @Getter
        private static Distance DistanceToTarget = Meters.of(0);
    }

    private final InputBuilder.Subsystems subsystems;

    public ScoringSystem(InputBuilder.Subsystems subsystems) {
        this.subsystems = subsystems;
    }

    public Command shootAverage() {
        return subsystems.hood().getHood().setAngle(Degrees.of(15))
                .alongWith(subsystems.flywheel().getFlyWheel().run(RotationsPerSecond.of(48)))
                .alongWith(subsystems.kicker().runKicker(.65, true))
                .alongWith(
                        Commands.waitSeconds(1)
                                .andThen(
                                        subsystems.indexer().runIndexer(.65, true))
                                        .alongWith(subsystems.intake().intake(.65, true)))
                .withTimeout(20);
    }

    public Command shootOnTheMove(SwerveInputStream stream) {
        double[] goals = new double[4];
        return Commands.run(() -> calculateSOTMGoals(FieldConstants.Hub.topCenterPoint.toTranslation2d(), goals))
                // Run goals
                .alongWith(Commands.run(() -> stream.aimLookahead(Seconds.of(goals[0]).unaryMinus())))
                .alongWith(subsystems.hood().getHood().run(() -> Degrees.of(goals[1])))
                .alongWith(subsystems.flywheel().getFlyWheel().run(() -> RPM.of(goals[2])))
                .alongWith((Commands.waitUntil(() ->
                                stream.aimLock(Degrees.of(3)).getAsBoolean()
                                        && subsystems.flywheel().isNear(() -> RPM.of(goals[2]), RPM.of(50)).getAsBoolean()
                                        && subsystems.hood().isNear(() -> Degrees.of(goals[1]), Degrees.of(1)).getAsBoolean())
                        .andThen(Commands.runOnce(() -> subsystems.flywheel().simShoot(subsystems)))
                        .andThen(Commands.waitSeconds(.15)))
                        .repeatedly());
    }

    public void calculateSOTMGoals(Translation2d target, double[] goals) {
        target = AllianceFlipUtil.ifShouldFlip(new Pose2d(target, Rotation2d.kZero)).getTranslation();
        // Velocity measurement delay that affects precise calculations.
        double phaseDelay = 0.05;

        Pose2d rawPose = subsystems.swerve().getSwerveDrive().getPose();
        ChassisSpeeds fieldVel = subsystems.swerve().getSwerveDrive().getFieldVelocity();

        // Shooter mounting offset, rotated into field coordinates.
        Translation2d shooterOffset =
                TurretSubsystem.HardwareConstants.TURRET_POSITION.toTranslation2d()
                        .rotateBy(rawPose.getRotation());
        Pose2d shooterPose =
                new Pose2d(rawPose.getTranslation().plus(shooterOffset), rawPose.getRotation());

        // Velocity of the shooter point: chassis translation plus omega x r.
        double omega = fieldVel.omegaRadiansPerSecond;
        Translation2d shooterVel = new Translation2d(
                fieldVel.vxMetersPerSecond + (-omega * shooterOffset.getY()),
                fieldVel.vyMetersPerSecond + (omega * shooterOffset.getX()));

        // Where the shooter will actually be once the measurement delay is paid off.
        Pose2d estimatedPose = new Pose2d(
                shooterPose.getTranslation().plus(shooterVel.times(phaseDelay)),
                shooterPose.getRotation().plus(Rotation2d.fromRadians(omega * phaseDelay)));

        double timeOfFlight = 0;
        Pose2d lookAheadPose = estimatedPose;
        double lookAheadDistance = target.getDistance(lookAheadPose.getTranslation());

        // Iterate time of flight to converge on an accurate distance value.
        for (int i = 0; i < 5; i++) {
            timeOfFlight = DoubleInterpolatingMaps.timeOfFlight.get(lookAheadDistance);
            lookAheadPose = new Pose2d(
                    estimatedPose.getTranslation().plus(shooterVel.times(timeOfFlight)),
                    estimatedPose.getRotation());
            lookAheadDistance = target.getDistance(lookAheadPose.getTranslation());
        }
        SOTMLatestGoals.HoodGoal =
                DoubleInterpolatingMaps.hoodAngle.get(lookAheadDistance).getMeasure();
        SOTMLatestGoals.FlyWheelGoal =
                RPM.of(DoubleInterpolatingMaps.flyWheelSpeed.get(lookAheadDistance));
        SOTMLatestGoals.TimeOfFlight = Seconds.of(timeOfFlight);
        SOTMLatestGoals.DistanceToTarget = Meters.of(lookAheadDistance);

        goals[0] = timeOfFlight;
        goals[1] = SOTMLatestGoals.HoodGoal.in(Degrees);
        goals[2] = SOTMLatestGoals.FlyWheelGoal.in(RPM);
    }

//    public Trigger isReadyToFire() {
//        return new Trigger(subsystems.turret().isNear(SOTMLatestGoals.getTurretGoal())).and(subsystems.hood().isNear(SOTMLatestGoals.getHoodGoal()));
//    }
}