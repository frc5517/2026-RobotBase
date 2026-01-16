// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

public class RobotContainer {
    /// Drive Base
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    /// Scoring System
    private final TurretSubsystem turret = new TurretSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final FlyWheelSubsystem  flyWheel = new FlyWheelSubsystem();
    /// Loading System
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    /// Make our input builder, thus creating all of our inputs.
    private final InputBuilder inputBuilder = new InputBuilder(
            swerve, turret);
  public RobotContainer() {
  }

  public void periodic()
  {
      Telemetry.updateTelemetry();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
