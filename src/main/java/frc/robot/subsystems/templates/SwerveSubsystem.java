package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

/**
 * Swerve Drive Subsystem, using MapleSim for physics simulation.
 */
public class SwerveSubsystem extends SubsystemBase
{
  /// Telemetry Options
  private final String driveMotorTelemetryName = "driveMotor";
  private final String angleMotorTelemetryName = "angleMotor";
  private final SmartMotorControllerConfig.TelemetryVerbosity telemetryVerbosity = SmartMotorControllerConfig.TelemetryVerbosity.HIGH;
  /// Module Configuration Options
  private final DCMotor driveMotor = DCMotor.getNEO(1);
  private final DCMotor azimuthMotor = DCMotor.getNEO(1);
  private final MechanismGearing driveGearing = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
  private final Distance wheelDiameter = Inches.of(4);
  private final MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromStages("21:1"));
  private final PIDController drivePIDController = new PIDController(50, 0, 4);
  private final PIDController azimuthPIDController = new PIDController(50, 0, 4);
  private final Current driveStatorCurrentLimit = Amps.of(40);
  private final Current azimuthStatorCurrentLimit = Amps.of(20);
  /// Module Locations
  // Track Width and height can be used to quickly set locations on symmetric robots. If overriding, make sure to update the MapleSim configuration.
  private final Distance trackHeight = Inches.of(30);
  private final Distance trackWidth = Inches.of(30);
  private final Translation2d flLocation = new Translation2d(trackHeight.div(2), trackWidth.div(2));
  private final Translation2d frLocation = new Translation2d(trackHeight.div(2), trackWidth.div(2).unaryMinus());
  private final Translation2d blLocation = new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2));
  private final Translation2d brLocation = new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2).unaryMinus());
  /// Drivetrain Configuration Options
  private final LinearVelocity maximumChassisSpeedsLinearVelocity = MetersPerSecond.of(10);
  private final AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(360);
  private final PIDController translationController = new PIDController(1, 0, 0);
  private final PIDController rotationController = new PIDController(1, 0, 0);
  private final Pose2d startingPose = new Pose2d(1.5, 3, Rotation2d.fromDegrees(0));
  /// MapleSim Integration Options
  private final Distance wheelCenterToBumper = Inches.of(5);
  private final Mass robotMass = Pounds.of(100);


  private final SwerveDrive drive;
  private final Field2d     field = new Field2d();

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationXScalar,
                                                    DoubleSupplier translationYScalar,
                                                    DoubleSupplier rotationScalar)
  {
    return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
        .withMaximumAngularVelocity(maximumChassisSpeedsAngularVelocity)
        .withMaximumLinearVelocity(maximumChassisSpeedsLinearVelocity)
        .withDeadband(0.01)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl();
  }

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public Supplier<ChassisSpeeds> getSimpleChassisSpeeds(DoubleSupplier translationXScalar,
                                                        DoubleSupplier translationYScalar,
                                                        DoubleSupplier rotationScalar)
  {
    return () -> new ChassisSpeeds(maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble())
                                                                      .in(RadiansPerSecond));
  }

  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, AnalogEncoder encoder, String moduleName,
                                   Translation2d location)
  {
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(wheelDiameter)
        .withClosedLoopController(drivePIDController)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(driveStatorCurrentLimit)
        .withTelemetry(driveMotorTelemetryName, telemetryVerbosity);
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(azimuthPIDController)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(driveStatorCurrentLimit)
        .withTelemetry(angleMotorTelemetryName, telemetryVerbosity);
    SmartMotorController driveSMC   = new SparkWrapper(drive, driveMotor, driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, azimuthMotor, azimuthCfg);
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(encoder)
        .withTelemetry(moduleName, telemetryVerbosity)
        .withLocation(location)
        .withOptimization(true);
    return new SwerveModule(moduleConfig);
  }

  public SwerveSubsystem()
  {
    final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    var fl = createModule(new SparkMax(1, MotorType.kBrushless),
                          new SparkMax(2, MotorType.kBrushless),
                          new AnalogEncoder(3),
                          "frontleft",
                          new Translation2d(Inches.of(24), Inches.of(24)));
    var fr = createModule(new SparkMax(4, MotorType.kBrushless),
                          new SparkMax(5, MotorType.kBrushless),
                          new AnalogEncoder(6),
                          "frontright",
                          new Translation2d(Inches.of(24), Inches.of(-24)));
    var bl = createModule(new SparkMax(7, MotorType.kBrushless),
                          new SparkMax(8, MotorType.kBrushless),
                          new AnalogEncoder(9),
                          "backleft",
                          new Translation2d(Inches.of(-24), Inches.of(24)));
    var br = createModule(new SparkMax(10, MotorType.kBrushless),
                          new SparkMax(11, MotorType.kBrushless),
                          new AnalogEncoder(12),
                          "backright",
                          new Translation2d(Inches.of(-24), Inches.of(-24)));
//    DriveTrainSimulationConfig mapleConfig = DriveTrainSimulationConfig.Default()
//            .withBumperSize(trackHeight.plus(wheelCenterToBumper), trackWidth.plus(wheelCenterToBumper))
//            .withRobotMass(robotMass)
//            .withCustomModuleTranslations(new Translation2d[] { flLocation, frLocation, blLocation, brLocation})
//            .withGyro(COTS.ofPigeon2());
    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(() -> Degrees.of(gyro.getAngle()))
        .withTranslationController(translationController)
        .withRotationController(rotationController);
//        .withMapleSim(mapleConfig, startingPose);
    drive = new SwerveDrive(config);

    SmartDashboard.putData("Field", field);
  }

  /**
   * Drive the {@link SwerveDrive} object with robot relative chassis speeds.
   *
   * @param speedsSupplier Robot relative {@link ChassisSpeeds}.
   * @return {@link Command} to run the drive.
   */
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  public Command driveToPose(Pose2d pose)
  {
    return drive.driveToPose(pose);
  }

  public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command lock()
  {
    return run(drive::lockPose);
  }

  @Override
  public void periodic()
  {
//    SimulatedArena.getInstance().simulationPeriodic();
    drive.updateTelemetry();
//    field.setRobotPose(RobotBase.isSimulation() ? drive.getMapleSimPose() : drive.getPose());
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
    field.getObject("odometry").setPose(drive.getPose());
  }
}

