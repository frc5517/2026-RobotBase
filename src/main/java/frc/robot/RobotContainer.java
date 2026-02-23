// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.systems.ScoringSystem;

public class RobotContainer {
    /// Subsystems
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final FlyWheelSubsystem  flyWheel = new FlyWheelSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    /// Bite-sized Subsystems record class for easy packaging of all the subsystems.
    private final InputBuilder.Subsystems subsystems = new InputBuilder.Subsystems(flyWheel, hood, indexer, intake, null, swerve, turret);
    /// Our Systems
    private final ScoringSystem scoring = new ScoringSystem(subsystems);
    /// Input Builder
    private final InputBuilder inputBuilder = new InputBuilder(subsystems, scoring);

    /**
     * This is an unconventional FRC RobotContainer layout.
     * ConfigureBindings has been pushed to {@link InputBuilder}.
     * This allows creation, manipulation, selection, and layered complexity of bindings with ease.
     */
    public RobotContainer() {
        inputBuilder.configureBindings();
    }

    public void periodic()
    {
        Telemetry.updateTelemetry();
    }

    public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
