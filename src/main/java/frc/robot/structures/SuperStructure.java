package frc.robot.structures;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class SuperStructure {
    /**
     * Setpoint enum, used to group the setpoints required to do something.
     */
    public enum Setpoints
    {

        Collect (Meters.of(1), Degrees.of(-30));

        public final Distance elevatorHeight;
        public final Angle armAngle;

        Setpoints(Distance elevatorHeight, Angle armAngle) {
            this.elevatorHeight = elevatorHeight;
            this.armAngle = armAngle;
        }
    }

    public Command setManipulatorPose(Setpoints setpoint)
    {
        Angle value = Setpoints.Collect.armAngle;
        return Commands.none();
    }

}
