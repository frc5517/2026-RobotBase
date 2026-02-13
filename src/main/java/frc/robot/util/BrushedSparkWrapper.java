package frc.robot.util;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BrushedSparkWrapper {

    private final SparkMax motor;

    public BrushedSparkWrapper(int canID) {
        this.motor = new SparkMax(canID, SparkLowLevel.MotorType.kBrushed);
    }

    public Command set(double dutycycle) {
        return Commands.run(() -> motor.set(dutycycle));
    }

}
