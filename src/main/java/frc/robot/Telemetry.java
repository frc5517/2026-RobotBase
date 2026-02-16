package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.ScoringSystem;
import frc.robot.util.math.AllianceFlipUtil;
import lombok.Getter;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.motorcontrollers.SmartMotorControllerConfig;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static frc.robot.systems.ScoringSystem.HardwareConstants.SCORING_ZONE;

public class Telemetry
{
    /// Current Telemetry Setting
    public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;
    /// Telemetry Defaults
    public static final String telemetryPath = "SmartDashboard/Telemetry/"; // Make access public for other telemetry.
    public static final String smartDashboardPath = "Telemetry/";
    public static final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable(telemetryPath);

    /// Publishers
    public static class Publishers
    {
        /// Robot Publishers
        public static class Robot
        {
            /// Telemetry Paths
            private static final NetworkTable robotTable = telemetryTable.getSubTable("RobotTelemetry");
            // Zones
            private static final NetworkTable zoneTable = robotTable.getSubTable("Zones");
            private static final NetworkTable zoneTriggerTable = zoneTable.getSubTable("Triggers");
            private static final String smartDashboardRobotPath = smartDashboardPath + "RobotTelemetry/";
            // Input Selection
            public static final SmartDashboardPublisher inputPublisher = new SmartDashboardPublisher(smartDashboardRobotPath + "Input Selector");
            public static final StringPublisher inputOverride = robotTable.getSubTable("Input Selector").getStringTopic("selected").publish();
            // Zone Trigger Bounding Boxes
            public static final StructArrayPublisher<Translation2d> scoringZonePublisher = zoneTable.getStructArrayTopic("Scoring Zone", Translation2d.struct).publish();
            public static final StructArrayPublisher<Translation2d> bumpZonePublisher = zoneTable.getStructArrayTopic("Bump Zone", Translation2d.struct).publish();
            // Zone Trigger Publishers
            public static final BooleanPublisher scoringZoneTriggerPublisher = zoneTriggerTable.getBooleanTopic("In Scoring Zone").publish();
            public static final BooleanPublisher bumpZoneTriggerPublisher = zoneTriggerTable.getBooleanTopic("In Bump Zone").publish();

            /// Mech3d Publishers
            public static class Mech3D
            {
                public static final NetworkTable mechTable = robotTable.getSubTable("Mech3DTelemetry");
                public static final StructPublisher<Pose3d> hoodPose = mechTable.getStructTopic("Hood Pose", Pose3d.struct).publish();
                public static final StructPublisher<Pose3d> turretPose = mechTable.getStructTopic("Turret Pose", Pose3d.struct).publish();
            }

        }
        /// MapleSim Publishers
        public static class MapleSim
        {
            // Table for maple sim publishers.
            private static final NetworkTable mapleTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim");
            // Generic Game Piece Publisher.
            public static final StructArrayPublisher<Pose3d> elementPublisher = mapleTable.getStructArrayTopic("Fuel", Pose3d.struct).publish();
        }
    }

    /// Telemetry Verbosity Settings
    public enum TelemetryVerbosity {
        /// No telemetry data is sent to the dashboard.
        NONE(
                SmartMotorControllerConfig.TelemetryVerbosity.LOW,
                SwerveDriveTelemetry.TelemetryVerbosity.NONE),
        /// Only basic telemetry data is sent to the dashboard.
        LOW(
                SmartMotorControllerConfig.TelemetryVerbosity.LOW,
                SwerveDriveTelemetry.TelemetryVerbosity.LOW),
        /// All telemetry data is sent to the dashboard.
        HIGH(
                SmartMotorControllerConfig.TelemetryVerbosity.HIGH,
                SwerveDriveTelemetry.TelemetryVerbosity.HIGH),;

        // Telemetry verbosity for YAMS at this verbosity level.
        public final SmartMotorControllerConfig.TelemetryVerbosity yamsVerbosity;
        // Telemetry verbosity for YAGSL at this verbosity level.
        public final SwerveDriveTelemetry.TelemetryVerbosity yagslVerbosity;

        /**
         * Robot Telemetry Options
         *
         * @param yamsVerbosity Verbosity to use for YAMS at this level.
         */
        TelemetryVerbosity(
                SmartMotorControllerConfig.TelemetryVerbosity yamsVerbosity,
                SwerveDriveTelemetry.TelemetryVerbosity yagslVerbosity) {
            this.yamsVerbosity = yamsVerbosity;
            this.yagslVerbosity = yagslVerbosity;
        }
    }

    /// Initializes any need data. Called statically.
    Telemetry() {
        // Add all the default pieces.
        if (RobotBase.isSimulation()) {
            ((Arena2026Rebuilt) SimulatedArena.getInstance()).setEfficiencyMode(true); // Spawn more or less.
            SimulatedArena.getInstance().resetFieldForAuto(); // Reset the field.
        }
    }

    /// Updates all of our custom telemetry
    public static void updateTelemetry()
    {
        // Input
        Publishers.Robot.inputPublisher.update();
        // Zone Triggers
        Publishers.Robot.scoringZoneTriggerPublisher.accept(ScoringSystem.ControlConstants.SCORING_ZONE_TRIGGER.getAsBoolean());


        if (RobotBase.isSimulation()) {
            // MapleSim
        Publishers.MapleSim.elementPublisher.accept(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
    }

    /**
     * Zone Helper class to handle flipping n such.
     */
    public static class ZoneTrigger
    {
        private final List<Rectangle2D> rectangles;
        private final List<Pair<Translation2d, Translation2d>> zones;
        /**
         * Gets the ZoneTrigger.
         */
        @Getter
        private Trigger trigger;

        /**
         * Creates a new auto flipping zone trigger.
         *
         * @param zones all the zone bounding boxes.
         *              The first is bottom left, Second is bottom right.
         *              Each box is checked separately.
         */
        @SafeVarargs
        public ZoneTrigger(Pair<Translation2d, Translation2d>... zones) {
            // Setup our zones.
            this.rectangles = new ArrayList<>();
            this.zones = List.of(zones);
            // Setup our trigger.
            loadRectangles();
            this.trigger = new Trigger(() -> containsPose(() -> SwerveSubsystem.SwerveState.CurrentPose));
        }

        /**
         *
         * @param zonePublisher
         * @param zones
         */
        @SafeVarargs
        public ZoneTrigger(StructArrayPublisher<Translation2d> zonePublisher, Pair<Translation2d, Translation2d>... zones) {
            this(zones);
            Telemetry.ModeTriggers.isEnabled().onTrue(Commands.runOnce(() -> publishZone(zonePublisher)));
        }

        /**
         * Checks if the given pose is held within the zones.
         *
         * @param poseSupplier the position to check.
         * @return whether the pose is in the defined zone.
         */
        public boolean containsPose(Supplier<Pose2d> poseSupplier) {
            boolean[] contains = {false};
            final Pose2d pose = poseSupplier.get();
            rectangles.forEach(rect -> {
                if (rect.contains(pose.getX(), pose.getY())) {
                    contains[0] = true;
                }});
            return contains[0];
        }

        /**w
         * Flips the pose list if robot is red alliance.
         *
         * @param zones the given zones.
         * @return our zones at their expected side.
         */
        private List<Pair<Translation2d, Translation2d>> ifShouldFlip(List<Pair<Translation2d, Translation2d>> zones) {
            System.out.println(DriverStation.getAlliance().toString());
            // If should flip
            if (AllianceFlipUtil.shouldFlip()) {
                // Flip
                final List<Pair<Translation2d, Translation2d>> updatedZones = new ArrayList<>();
                zones.forEach(pair -> {
                    updatedZones.add(Pair.of(AllianceFlipUtil.apply(pair.getSecond()), AllianceFlipUtil.apply(pair.getFirst()))); // Flip corners as well. First becomes second.
                });
                return updatedZones;
            } else {
                // Else, don't.
                return zones;
            }
        }

        private void loadRectangles() {
            this.rectangles.clear(); // Clear any old alliance poses.
            // Makes a Rect2d from the start and end poses.
            final List<Pair<Translation2d, Translation2d>> list = ifShouldFlip(zones);
            list.forEach(zone ->
                    this.rectangles.add(new Rectangle2D.Double(
                            zone.getFirst().getX(),
                            zone.getFirst().getY(),
                            zone.getSecond().getX() - zone.getFirst().getX(),
                            zone.getSecond().getY() - zone.getFirst().getY())));
        }

        /**
         * Publishes a Translation in every bounding box corner.
         */
        private void publishZone(StructArrayPublisher<Translation2d> zonePublisher) {
            loadRectangles(); // Update our rectangles first.
            final ArrayList<Translation2d> conePoses = new ArrayList<>();
            rectangles.forEach(zone -> {
                conePoses.add(new Translation2d(zone.getMinX(), zone.getMinY()));
                conePoses.add(new Translation2d(zone.getMaxX(), zone.getMinY()));
                conePoses.add(new Translation2d(zone.getMaxX(), zone.getMaxY()));
                conePoses.add(new Translation2d(zone.getMinX(), zone.getMaxY()));
                conePoses.add(new Translation2d(zone.getMinX(), zone.getMinY())); // Close the Box Line
            });
            zonePublisher.set(conePoses.toArray(Translation2d[]::new));
        }
    }

    /**
     * SmartDashboard wrapper to match NT4 Publishers.
     */
    public static class SmartDashboardPublisher
    {
        @Getter
        private Sendable value;
        private Supplier<Sendable> supplier;
        private final String path;

        /**
         * Small SmartDashboard Publisher Wrapper.
         *
         * @param path the telemetry path. Use smartDashboardPath + name.
         */
        public SmartDashboardPublisher(String path)
        {
            this.path = path;

        }

        /**
         * Puts the value onto the dashboard.
         *
         * @param value to publish.
         */
        public void setValue(Sendable value)
        {
            this.value = value;
            SmartDashboard.putData(path, value);
        }

        /**
         * Accepts sendable to be updated later.
         * Also sets when ran.
         *
         * @param supplier the value supplier.
         */
        public void accept(Supplier<Sendable> supplier)
        {
            this.supplier = supplier;
            setValue(supplier.get());
        }

        /**
         * Updates the published value from the current supplier.
         */
        public void update()
        {
            setValue(supplier.get());
        }
    }

    /**
     * Quick Helper since RobotModeTriggers only has a few triggers.
     */
    public static class ModeTriggers {
        private static final Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);

        public static Trigger isEnabled() {
            return isEnabledTrigger;
        }
    }
}

