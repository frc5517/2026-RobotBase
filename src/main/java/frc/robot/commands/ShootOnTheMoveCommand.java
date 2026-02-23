package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.InputBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootOnTheMoveCommand extends Command {
  private final InputBuilder.Subsystems subsystems;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle latestHoodAngle;
  private Angle latestTurretAngle;

  public ShootOnTheMoveCommand(InputBuilder.Subsystems subsystems, Supplier<Translation3d> aimPointSupplier) {
    this.subsystems = subsystems;
    this.aimPointSupplier = aimPointSupplier;
    this.addRequirements(subsystems.hood(), subsystems.turret(), subsystems.flywheel());

    // We use the drivetrain to determine linear velocity, but don't require it for
    // control. We
    // will be using the superstructure to control the shooting mechanism so it's a
    // requirement.
    // addRequirements(superstructure);

    // TODO: figure out if the above is actually required. Right now, when you start
    // some other command, the auto aim can't start back up again
  }

  @Override
  public void initialize() {
    super.initialize();
    System.out.println("***** SOTM Init *****");
    latestHoodAngle = subsystems.hood().getHood().getAngle();
    latestTurretAngle = subsystems.turret().getTurret().getAngle();
    latestShootSpeed = subsystems.flywheel().getFlyWheel().getSpeed();

    /// Dynamically sets all goals until finished.
    subsystems.hood().getHood().run(() -> latestHoodAngle)
            .alongWith(subsystems.turret().getTurret().run(() -> latestTurretAngle))
                    .alongWith(subsystems.flywheel().getFlyWheel().run(() -> latestShootSpeed))
            .until(this::isFinished).asProxy();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate trajectory to aimPoint
    var target = aimPointSupplier.get();

    var shooterLocation = subsystems.swerve().getSwerveDrive().getPose().getTranslation()
        .plus(subsystems.turret().getPose3D().getTranslation().toTranslation2d());

    // Ignore this parameter for now, the range tables will account for it :/
    // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
    var targetOnGround = new Translation2d(target.getX(), target.getY());

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

    // Get time of flight. We could try to do this analytically but for now it's
    // easier and more realistic
    // to use a simple linear approximation based on empirical data.
    double timeOfFlight = getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time
    // of flight.
    // If we're stationary, this should be zero. If we're backing up, this will be
    // "ahead" of the target, etc.
    var updatedPosition = subsystems.swerve().getSwerveDrive().getFieldVelocity().times(timeOfFlight);
    var correctiveVector = new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond)
        .unaryMinus();
    var correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);

    var correctedTarget = targetOnGround.plus(correctiveVector);

    var vectorToTarget = subsystems.swerve().getSwerveDrive().getPose().getTranslation().minus(correctedTarget);

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    var calculatedHeading = vectorToTarget.getAngle()
        .rotateBy(subsystems.swerve().getSwerveDrive().getPose().getRotation().unaryMinus())
        .getMeasure();


    latestTurretAngle = calculatedHeading;
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);

    System.out.printf(
            "Current Turret: %f, Calc Turret: %f, " +
                    "Current Hood: %f, Calc Hood: %f, " +
                    "Current Flywheel: %f, Calc Flywheel: %f\n",
            subsystems.turret().getTurret().getAngle().in(Degrees), latestTurretAngle.in(Degrees),
            subsystems.hood().getHood().getAngle().in(Degrees), latestHoodAngle.in(Degrees),
            subsystems.flywheel().getFlyWheel().getSpeed().in(RPM), latestShootSpeed.in(RPM));


  }

  private double getFlightTime(Distance distanceToTarget) {
    // Simple linear approximation based on empirical data.
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    return RPM.of(SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  private Angle calculateRequiredHoodAngle(Distance distanceToTarget) {
    return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  // meters, seconds
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.0, 1.0),
      Map.entry(4.86, 1.5));
  // TODO: add more data points here.
  // CLOSE: NEED
  // MID: maybe good enough
  // FAR: NEED

  // meters, RPS
  private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(2.0, 2700.0),
      Map.entry(3.0, 3000.0),
      Map.entry(4.0, 3300.0),
      Map.entry(4.86, 3750.0));

  // meters, degrees
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.0, 15.0),
      Map.entry(2.0, 30.0),
      Map.entry(3.0, 45.0));
}
