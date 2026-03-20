// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.systems.ScoringSystem;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.opponents.EmptyOpponent;

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
        final var turretSubsystem = new TurretSubsystem();
        subsystems = new InputBuilder.Subsystems(new SwerveSubsystem(), new FlyWheelSubsystem(), new HoodSubsystem(turretSubsystem), new IndexerSubsystem(), new IntakeSubsystem(), new AgitatorSubsystem(), new KickerSubsystem(), turretSubsystem);

        scoring = new ScoringSystem(subsystems);
        inputBuilder = new InputBuilder(subsystems, scoring);
        autonSelector = new AutonSelector(subsystems, scoring);
        inputBuilder.configureBindings();

        if (RobotBase.isSimulation()) {
            // Enable Simulated Scoring Data
            SimulatedArena.getInstance().enableBreakdownPublishing();

            DriverStation.Alliance opponentAlliance = DriverStation.Alliance.Blue;
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                    opponentAlliance = DriverStation.Alliance.Red;
                }
            }
            // A Dumb Smart opponent to play defense. More of an annoyance.
            new EmptyOpponent("Defense Bot 1", opponentAlliance).withDefense(() -> subsystems.swerve().getSwerveDrive().getPose());
        }
    }

    public void periodic() {
        Telemetry.updateTelemetry(subsystems);
    }

    public Command getAutonomousCommand() {
        return autonSelector.getSelected();
    }
}
