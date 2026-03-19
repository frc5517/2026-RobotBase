// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import frc.robot.InputBuilder;
import frc.robot.Telemetry;
import frc.robot.util.borrowed.math.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SwerveSubsystem.HardwareConstants.*;
import static frc.robot.subsystems.SwerveSubsystem.ControlConstants.*;

public class SwerveSubsystem extends SubsystemBase
{
    public static class HardwareConstants {
        /// This is the max achievable speed of the physical chassis.
        /// Do not change this. It is not an arbitrary speed limiter.
        public static final double MAX_SPEED = 14.5;
        public static final File CONFIG_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve");
        /// Max accelerations are calculated in the Pathplanner GUI
        public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(11.7);
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(1749);
    }
    public static class ControlConstants {
        /// Enable vision odometry updates while driving.
        public static final boolean RUN_VISION = true; // Run only in sim until real vision is ready.
        // Simulation Starting Pose, flipping is handled by YAGSL.
        public static final Pose2d INITIAL_SIM_POSE = new Pose2d(new Translation2d(
                Meter.of(2),
                Meter.of(6)),
                Rotation2d.fromDegrees(0));
    }
  /// The Swerve drive object.
  @Getter
  private final SwerveDrive swerveDrive;
  /// PhotonVision class to keep an accurate odometry.
  @Getter
  private PhotonSubsystem vision;

  public static class SwerveState {
      @Setter
      public static Supplier<SwerveDrive> swerveDrive = () -> null;
      @Setter @Getter
      public static Pose2d CurrentPose = Pose2d.kZero;
      @Setter
      public static ChassisSpeeds CurrentSpeeds = new ChassisSpeeds();
  }

  /**
   * Initialize {@link SwerveDrive}.
   */
   public SwerveSubsystem() {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = Telemetry.telemetryVerbosity.yagslVerbosity;
    try
    {
      swerveDrive = new SwerveParser(CONFIG_DIRECTORY).createSwerveDrive(MAX_SPEED, INITIAL_SIM_POSE);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setCosineCompensator(true);
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.15); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                3); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    if (RUN_VISION)
    {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way, we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    setupPathPlanner();

    new Trigger(RobotBase::isSimulation).and(DriverStation::isTeleopEnabled).onTrue(Commands.runOnce(() -> swerveDrive.resetOdometry(AllianceFlipUtil.ifShouldFlip(INITIAL_SIM_POSE))));

    /// Set our SwerveState Suppliers
    SwerveState.setSwerveDrive(this::getSwerveDrive);
  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision()
  {
    vision = new PhotonSubsystem(swerveDrive::getPose, swerveDrive.field);
  }

  @Override
  public void periodic()
  {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (RUN_VISION)
    {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
      //vision.updateGamePieceSim();
    }
    SwerveState.setCurrentPose(swerveDrive.getPose());
  }

  @Override
  public void simulationPeriodic()
  {
      if (RUN_VISION) {
          Telemetry.updateCameraPoses(new Pose3d(swerveDrive.getPose()));
      }
  }

    /**
     * Automatically decides how to get to the other zone.
     *
     * @return a command to toggle zones.
     */
  public Command toggleZones() {
      return Commands.defer(() ->
              changeZones(
                      InputBuilder.CustomTriggers.allianceZone.getTrigger().getAsBoolean(), // If in allianceZone go to neutral.
                      swerveDrive.getPose().getMeasureY().gte(Meters.of(4))), // If closer to left field go left.
              Set.of(this));
  }

    /**
     * Changes between our zone and the neutral zone automatically and safely.
     * Uses booleans to determine the path.
     *
     * @param toNeutral
     * @param isRightBump
     * @return
     */
  public Command changeZones(boolean toNeutral, boolean isRightBump) {
      String pathName;
      if (toNeutral) {
          if (isRightBump) {
              pathName = "Right Bump Neutral";
          } else {
              pathName = "Left Bump Neutral";
          }
      } else {
          if (isRightBump) {
              pathName = "Right Bump Alliance";
          } else  {
              pathName = "Left Bump Alliance";
          }
      }
      return pathfindToPath(pathName);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try
    {
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built-in path-following controller for holonomic drive trains
              new PIDConstants(4.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(4.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          RobotConfig.fromGUISettings(), // The pathplanner configuration
          AllianceFlipUtil::shouldFlip, // Whether to flip the path
          this); // Reference to this subsystem to set requirements
    } catch (Exception e)
    {
      // Handle exception as needed
      DriverStation.reportWarning("Error during pathplanner setup:\n" + e.getMessage(), true);
    }
    //Preload PathPlanner Path finding
    PathfindingCommand.warmupCommand();
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path-following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Supplier<Pose2d> pose)
  {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return Commands.defer(() ->
            AutoBuilder.pathfindToPose(
                    pose.get(),
                    constraints,
                    edu.wpi.first.units.Units.MetersPerSecond.of(0)),
            Set.of(this)); // Goal end velocity in meters/sec
  }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param path Target {@link PathPlannerPath} to go find and follow.
     * @return PathFinding command
     */
    public Command pathfindToPath(PathPlannerPath path)
    {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(), 4.0,
                swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pathName The path name. Automatically hangles Exception.
     * @return PathFinding command
     */
    public Command pathfindToPath(String pathName)
    {
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return pathfindToPath(path);
        } catch (Exception e) {
            DriverStation.reportError("PathPlannerPath " + pathName + " not found: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
  throws IOException, ParseException
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                            swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint
        = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                   swerveDrive.getStates(),
                                                   DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                    () -> {
                      double newTime = Timer.getFPGATimestamp();
                      SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                      robotRelativeChassisSpeed.get(),
                                                                                      newTime - previousTime.get());
                      swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                        newSetpoint.moduleStates(),
                                        newSetpoint.feedforwards().linearForces());
                      prevSetpoint.set(newSetpoint);
                      previousTime.set(newTime);
                    });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), swerveDrive.getOdometryHeading());

      });
    } catch (Exception e)
    {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }

    /**
    * Drive, according to the chassis robot-oriented velocity.
    *
    * @param velocity Robot oriented {@link ChassisSpeeds}
    */
    public Command drive(Supplier<ChassisSpeeds> velocity) {
      return runEnd(
              () -> swerveDrive.drive(velocity.get()),
              () -> swerveDrive.drive(new ChassisSpeeds()));
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, swerveDrive, 12, true),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, swerveDrive),
                3.0, 5.0, 3.0);
    }
}