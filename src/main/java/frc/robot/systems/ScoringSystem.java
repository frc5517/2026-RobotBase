package frc.robot.systems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.InputBuilder;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;

public class ScoringSystem {
    public static class HardwareConstants {
        public static Angle HUB_ENTRY_ANGLE = Degrees.of(35);
    }
    public static final class ControlConstants {
    }

    public ScoringSystem() {
    }

    public Trigger autoScore(BooleanSupplier enable) {
        return InputBuilder.CustomTriggers.scoringZone.getTrigger()
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