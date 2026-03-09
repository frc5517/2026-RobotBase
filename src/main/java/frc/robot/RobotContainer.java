// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.systems.ScoringSystem;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

public class RobotContainer {
    /// Bite-sized Subsystems record class for easy packaging of all the subsystems.
    private final InputBuilder.Subsystems subsystems;
    /// Our Systems
    private final ScoringSystem scoring;
    /// Input Builder
    private final InputBuilder inputBuilder;
    ///
    private final AutonSelector autonSelector;

    /**
     * This is an unconventional FRC RobotContainer layout.
     * ConfigureBindings has been pushed to {@link InputBuilder}.
     * This allows creation, manipulation, selection, and layered complexity of bindings with ease.
     */
    public RobotContainer() {
        /// Disable not hardware finished subsystems unless it is sim.
        if (RobotBase.isSimulation()) {
            ///  If in Simulation enable all
            final var turretSubsystem = new TurretSubsystem();
            subsystems = new InputBuilder.Subsystems(new SwerveSubsystem(), new FlyWheelSubsystem(), new HoodSubsystem(turretSubsystem), new IndexerSubsystem(), new IntakeSubsystem(), new KickerSubsystem(), turretSubsystem);
        } else {
            ///  If not in sim, disable not complete systems.
            final TurretSubsystem turretSubsystem = null;
            subsystems = new InputBuilder.Subsystems(new SwerveSubsystem(), new FlyWheelSubsystem(), new HoodSubsystem(turretSubsystem), new IndexerSubsystem(), new IntakeSubsystem(), new KickerSubsystem(), turretSubsystem);
        }
        scoring = new ScoringSystem(subsystems);
        inputBuilder = new InputBuilder(subsystems, scoring);
        autonSelector = new AutonSelector(subsystems, scoring);
        inputBuilder.configureBindings();

        if (RobotBase.isSimulation()) {
            // Enable Simulated Scoring Data
            SimulatedArena.getInstance().enableBreakdownPublishing();
        }
    }

    public void periodic()
    {
        SimulatedArena.getInstance().simulationPeriodic();
        SmartDashboard.putNumber("Red Score", SimulatedArena.getInstance().getScore(false));
        Telemetry.updateTelemetry();
    }

    public Command getAutonomousCommand() {
    return autonSelector.getSelected();
  }
}
