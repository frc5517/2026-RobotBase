package frc.robot.systems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.InputBuilder;
import frc.robot.systems.ScoringSystem.ControlConstants.DoubleInterpolatingMaps;
import frc.robot.util.borrowed.math.AllianceFlipUtil;
import frc.robot.util.borrowed.math.FieldConstants;
import lombok.Getter;

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
                hoodAngle.put(1.34, Rotation2d.fromDegrees(19.0));
                hoodAngle.put(1.78, Rotation2d.fromDegrees(19.0));
                hoodAngle.put(2.17, Rotation2d.fromDegrees(24.0));
                hoodAngle.put(2.81, Rotation2d.fromDegrees(27.0));
                hoodAngle.put(3.82, Rotation2d.fromDegrees(29.0));
                hoodAngle.put(4.09, Rotation2d.fromDegrees(30.0));
                hoodAngle.put(4.40, Rotation2d.fromDegrees(31.0));
                hoodAngle.put(4.77, Rotation2d.fromDegrees(32.0));
                hoodAngle.put(5.57, Rotation2d.fromDegrees(32.0));
                hoodAngle.put(5.60, Rotation2d.fromDegrees(35.0));

                flyWheelSpeed.put(2.12923, 2725.0);
                flyWheelSpeed.put(2.504222, 2800.0);
                flyWheelSpeed.put(2.889, 2950.0);
                flyWheelSpeed.put(3.254686, 3085.0);
                flyWheelSpeed.put(3.695324, 3200.0);
                flyWheelSpeed.put(3.983757, 3320.0);
                flyWheelSpeed.put(4.498437, 3475.0);
                flyWheelSpeed.put(4.986071, 3675.0);
                flyWheelSpeed.put(5.410986, 4000.0);

                timeOfFlight.put(5.68, 1.16);
                timeOfFlight.put(4.55, 1.12);
                timeOfFlight.put(3.15, 1.11);
                timeOfFlight.put(1.88, 1.09);
                timeOfFlight.put(1.38, 0.90);
                }
        }

        public static final class SOTMTargets {
            public static final Pose2d HUB = new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero);
        }
    }

    public static class SOTMLatestGoals {
        @Getter
        private static Angle TurretGoal = Degrees.of(0);
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

    public Command shootOnTheMove(Supplier<Pose2d> target) {
        // Goal values
        //Angle[] turretGoal = {subsystems.turret().getTurret().getAngle()};
        Angle[] hoodGoal = {subsystems.hood().getHood().getAngle()};
        AngularVelocity[] flyWheelGoal = {subsystems.flywheel().getFlyWheel().getSpeed()};
        return Commands.run(() -> calculateSOTMGoals(
                        // Calculate goals based on a flipped target
                        AllianceFlipUtil.ifShouldFlip(target.get()).getTranslation(),
                        //(newAngle) -> newAngle,
                        (newAngle) -> hoodGoal[0] = newAngle,
                        (newSpeed) -> flyWheelGoal[0] = newSpeed))
                // Run goals
                //.alongWith(subsystems.turret().getTurret().run(() -> turretGoal[0]))
                .alongWith(subsystems.hood().getHood().run(() -> hoodGoal[0]))
                .alongWith(RobotBase.isSimulation()
                        ? subsystems.flywheel().simShoot(subsystems, () -> flyWheelGoal[0]).andThen(Commands.waitSeconds(.2)).repeatedly()
                        : subsystems.flywheel().setGoal(() -> flyWheelGoal[0]));
    }

    public void calculateSOTMGoals(Translation2d target, Consumer<Angle> hoodGoal, Consumer<AngularVelocity> flyWheelGoal) {
        // Velocity measurement delay that affects precise calculations.
        double phaseDelay = 0.05;
        Pose2d rawPose = subsystems.swerve().getSwerveDrive().getPose();
        // 2d location of the shooter
        Pose2d turretPose = subsystems.turret().getPose3D().toPose2d().relativeTo(rawPose);
        // Distance to the target
        Distance distanceToTarget = Meters.of(turretPose.getTranslation().getDistance(target));
        // Chassis speeds adjusted for phase delay
        ChassisSpeeds robotSpeeds = subsystems.swerve().getSwerveDrive().getFieldVelocity().times(phaseDelay);
        // Estimate the pose based on current speeds adjusted for phase delay
        Pose2d estimatedPose = rawPose.exp(new Twist2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, robotSpeeds.omegaRadiansPerSecond));
        // Chassis speeds of the turret
        ChassisSpeeds turretSpeeds = subsystems.turret().getVelocity(robotSpeeds, estimatedPose.getRotation().getMeasure());
        // Time that the fuel is in the air.
        double timeOfFlight = 100; // Set to 100 for the IDEs sake.
        Pose2d lookAheadPose = turretPose;
        // Distance from the shooter to the target based on no calculations
        double lookAheadDistance = target.getDistance(lookAheadPose.getTranslation());
        /// Iterate time of flight to converge on accurate distance value.
        for (int i = 0; i < 10; i++) {
            // Interpolate through our data map to estimate how long the fuel will fly for.
            timeOfFlight = DoubleInterpolatingMaps.timeOfFlight.get(distanceToTarget.in(Meters));
            // Velocity imparted on the ball from the chassis over time
            double lookAheadOffsetX = turretSpeeds.vxMetersPerSecond * timeOfFlight;
            double lookAheadOffsetY = turretSpeeds.vyMetersPerSecond * timeOfFlight;
            // new target based on the calculated offset.
            lookAheadPose = turretPose.plus(new Transform2d(lookAheadOffsetX, lookAheadOffsetY, Rotation2d.kZero));
            // Distance from the shooter to the target based on the previous calculations.
            lookAheadDistance = target.getDistance(lookAheadPose.getTranslation());
        }
        /// Set Latest SOTMGoals
        /// Turret goal is straightforward, angle from origin from our translation with the robot as the origin.
        SOTMLatestGoals.TurretGoal = subsystems.turret().angleToPose(estimatedPose, target);
        /// Hood and Flywheel use the Double Interpolating Maps.
        SOTMLatestGoals.HoodGoal = DoubleInterpolatingMaps.hoodAngle.get(lookAheadDistance).getMeasure();
        SOTMLatestGoals.FlyWheelGoal = RPM.of(DoubleInterpolatingMaps.flyWheelSpeed.get(lookAheadDistance));
        /// Add other good data
        SOTMLatestGoals.TimeOfFlight = Seconds.of(timeOfFlight);
        SOTMLatestGoals.DistanceToTarget = Meters.of(lookAheadDistance);
        // They want consumers I guess.
        //turretGoal.accept(SOTMLatestGoals.getTurretGoal());
        hoodGoal.accept(SOTMLatestGoals.getHoodGoal());
        flyWheelGoal.accept(SOTMLatestGoals.getFlyWheelGoal());
    }

//    public Trigger isReadyToFire() {
//        return new Trigger(subsystems.turret().isNear(SOTMLatestGoals.getTurretGoal())).and(subsystems.hood().isNear(SOTMLatestGoals.getHoodGoal()));
//    }
}