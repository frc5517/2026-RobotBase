package frc.robot.systems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import frc.robot.subsystems.SwerveSubsystem.SwerveState;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.systems.ScoringSystem.ControlConstants.SCORING_ZONE_TRIGGER;
import static frc.robot.systems.ScoringSystem.HardwareConstants.SCORING_ZONE;

public class ScoringSystem {
    public static class HardwareConstants {
        public static Angle HUB_ENTRY_ANGLE = Degrees.of(35);

        public static final Telemetry.ZoneTrigger SCORING_ZONE = new Telemetry.ZoneTrigger(Telemetry.Publishers.Robot.scoringZonePublisher,
                Pair.of(new Translation2d(1.5, 0.5), new Translation2d(3.5, 7.5)));
    }
    public static final class ControlConstants {
        public static final Trigger SCORING_ZONE_TRIGGER = SCORING_ZONE.getTrigger();
    }

    public ScoringSystem() {
    }

    public Trigger autoScore(BooleanSupplier enable) {
        return SCORING_ZONE_TRIGGER
                .and(enable)
                .whileTrue(aimAtHub());
    }

    public Command aimAtHub() {
        return null;
    }
    public Command scoreHub() {
        return null;
    }
}