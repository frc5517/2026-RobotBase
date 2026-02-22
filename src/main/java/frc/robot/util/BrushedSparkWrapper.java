package frc.robot.util;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Telemetry;
import yams.motorcontrollers.SmartMotorControllerConfig;

import static edu.wpi.first.units.Units.Amps;

public class BrushedSparkWrapper {
    // Grabs the telemetry path.
    private final String telemetryPath = Telemetry.Publishers.Robot.smartDashboardBrushedPath;

    public final SparkMax motor;
    private final DCMotor dcMotor;
    private final SmartMotorControllerConfig config;
    private final DCMotorSim physSim;
    private final SparkMaxSim simMotor;
    private final boolean isSim;

    public BrushedSparkWrapper(int canID, SmartMotorControllerConfig config) {
        // SparkMax Instance
        this.motor = new SparkMax(canID, SparkLowLevel.MotorType.kBrushed);
        // Motor Physics Characteristics
        this.dcMotor = DCMotor.getVex775Pro(1);
        // YAMS SMC
        this.config = config;
        // Cached sim bool
        this.isSim = RobotBase.isSimulation();
        // If sim, setup sim.
        if (isSim) {
            /// Basic Physics sim motor
            this.physSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            dcMotor,
                            config.getMOI(),
                            config.getGearing().getRotorToMechanismRatio()),
                    dcMotor, 0.01, 0.01);
            // Rev SparkMaxSim
            this.simMotor = new SparkMaxSim(motor, dcMotor);
        } else  {
            // Else null
            this.physSim = null;
            this.simMotor = null;
        }
        // Publish our one timers.
        publishStaticTelemetry();
    }

    /**
     * Published one time data like CAN ID.
     */
    private void publishStaticTelemetry() {
        if (config.getTelemetryName().isPresent()) {
            SmartDashboard.putNumber(telemetryPath + config.getTelemetryName().get() + "/CAN ID", motor.getDeviceId());
            SmartDashboard.putNumber(telemetryPath + config.getTelemetryName().get() + "/Firmware Version", motor.getFirmwareVersion());
            SmartDashboard.putString(telemetryPath + config.getTelemetryName().get() + "/Motor Type", motor.getMotorType().toString());
        }
    }

    /**
     * Publish periodic telemetry
     */
    public void updateTelemetry() {
        if (config.getTelemetryName().isPresent()) {
            SmartDashboard.putNumber(telemetryPath + config.getTelemetryName().get() + "/Output Current", motor.getOutputCurrent());
            SmartDashboard.putNumber(telemetryPath + config.getTelemetryName().get() + "/Applied Output", motor.getAppliedOutput());
            SmartDashboard.putNumber(telemetryPath + config.getTelemetryName().get() + "/Bus Voltage", motor.getBusVoltage());
            SmartDashboard.putNumber(telemetryPath + config.getTelemetryName().get() + "/Motor Temp", motor.getMotorTemperature());
        }
    }

    /**
     * Simulation Iteration.
     */
    public void simIterate() {
        if (isSim) {
            // Iterate physics sim
            physSim.setInput(simMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
            physSim.update(0.02);
            // Iterate Rev SparkSim
            simMotor.iterate(
                    Units.radiansPerSecondToRotationsPerMinute(physSim.getAngularVelocityRadPerSec()),
                    RoboRioSim.getVInVoltage(),
                    0.02);
            // Simulate battery draw
            RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(physSim.getCurrentDrawAmps()));
        }
    }

    /**
     * Loops back to this wrapper to handle YAMS compatible getters.
     *
     * @return this.
     */
    public BrushedSparkWrapper getMotorController() {
        return this;
    }

    /**
     * Get the supply current of the motor controller.
     *
     * @return The supply current of the motor controller.
     */
    public Current getStatorCurrent() {
        return Amps.of(motor.getOutputCurrent());
    }

    /**
     * Set the DutyCycle of the SmartMotorController.
     * Params:
     * duty cycle – [-1,1] to set.
     *
     * @return {@link Command}
     */
    public Command set(double dutycycle) {
        return Commands.run(() -> motor.set(dutycycle));
    }
}
