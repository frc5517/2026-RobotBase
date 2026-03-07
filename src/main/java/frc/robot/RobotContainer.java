// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.systems.ScoringSystem;

public class RobotContainer {
    /// Subsystems
    private final FlyWheelSubsystem  flyWheel;
    private final HoodSubsystem hood;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final KickerSubsystem kicker;
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    /// Bite-sized Subsystems record class for easy packaging of all the subsystems.
    private final InputBuilder.Subsystems subsystems;
    /// Our Systems
    private final ScoringSystem scoring;
    /// Input Builder
    private final InputBuilder inputBuilder;

    /**
     * This is an unconventional FRC RobotContainer layout.
     * ConfigureBindings has been pushed to {@link InputBuilder}.
     * This allows creation, manipulation, selection, and layered complexity of bindings with ease.
     */
    public RobotContainer() {
        swerve = new SwerveSubsystem(); // Swerve MUST come first. Compile issues.
        flyWheel = new FlyWheelSubsystem();
        hood = new HoodSubsystem();
        indexer = new IndexerSubsystem();
        intake = new IntakeSubsystem();
        kicker = new KickerSubsystem();
        turret = new TurretSubsystem();

        /// Disable not hardware finished subsystems unless it is sim.
        if (RobotBase.isSimulation()) {
            subsystems = new InputBuilder.Subsystems(flyWheel, hood, indexer, intake, kicker, swerve, turret);
        } else {
            subsystems = new InputBuilder.Subsystems(null, null, indexer, intake, kicker, swerve, null);
        }
        scoring = new ScoringSystem(subsystems);
        inputBuilder = new InputBuilder(subsystems, scoring);
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
