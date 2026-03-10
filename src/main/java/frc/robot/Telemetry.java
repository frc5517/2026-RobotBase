package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import lombok.Getter;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.motorcontrollers.SmartMotorControllerConfig;

import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

public class Telemetry
{
    /// Current Telemetry Setting
    public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;
    /// Telemetry Defaults
    public static final String telemetryPath = "SmartDashboard/Telemetry"; // Make access public for other telemetry.
    public static final String smartDashboardPath = "Telemetry/";
    public static final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable(telemetryPath);
    public static final String yamsMechPath = "Telemetry/Mechanisms/";

    /// Publishers
    public static class Publishers
    {

        public static class Field {
            public static final NetworkTable fieldTable = telemetryTable.getSubTable("Field");
            public static final BooleanPublisher activeHub = fieldTable.getBooleanTopic("Hub Active").publish(); // True when our hub is active
            public static final BooleanPublisher blinkTimer = fieldTable.getBooleanTopic("Blink Timer").publish(); // Flashes whenever close to a time change. Auton, shift change, end game.
            public static final DoublePublisher shiftTime = fieldTable.getDoubleTopic("Shift Timer").publish(); // Relays time until shift change
            public static final DoublePublisher matchTime = fieldTable.getDoubleTopic("Match Timer").publish(); // Match timer.
        }

        /// Robot Publishers
        public static class Robot
        {
            /// Telemetry Paths
            // Zones
            public static final NetworkTable zoneTable = telemetryTable.getSubTable("Zones");
            public static final String smartDashboardBrushedPath = yamsMechPath + "Brushed Telemetry/";
            // Input Selection
            public static final SmartDashboardPublisher inputPublisher = new SmartDashboardPublisher(smartDashboardPath + "Input Selector");
            public static final StringPublisher inputOverride = zoneTable.getSubTable("Input Selector").getStringTopic("selected").publish();

            /// Mech3d Publishers
            public static class Mech3D
            {
                public static final NetworkTable mechTable = zoneTable.getSubTable("Visualize Telemetry");
                public static final StructPublisher<Pose2d> robotPose = mechTable.getStructTopic("Robot Pose", Pose2d.struct).publish();
                public static final StructPublisher<Pose3d> hoodPose = mechTable.getStructTopic("Hood Pose", Pose3d.struct).publish();
                public static final StructPublisher<Pose3d> turretPose = mechTable.getStructTopic("Turret Pose", Pose3d.struct).publish();
                public static final StructArrayPublisher<Pose3d> fuelTrajectory = mechTable.getStructArrayTopic("Shot Trajectory", Pose3d.struct).publish();
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

    /// Initializes any need data. Called statically.
    Telemetry() {
        // Add all the default pieces.
        if (RobotBase.isSimulation()) {
            ((Arena2026Rebuilt) SimulatedArena.getInstance()).setEfficiencyMode(true); // Spawn more or less.
            SimulatedArena.getInstance().resetFieldForAuto(); // Reset the field.
        }
    }

    /// Updates all of our custom telemetry
    public static void updateTelemetry(InputBuilder.Subsystems subsystem)
    {
        // Input
        Publishers.Robot.inputPublisher.update();
        Publishers.Robot.Mech3D.robotPose.set(subsystem.swerve().getSwerveDrive().getPose());

        if (RobotBase.isSimulation()) {
            SmartDashboard.putBoolean("ALLIANCE ZONE", InputBuilder.CustomTriggers.allianceZone.getTrigger().getAsBoolean());
            // MapleSim
        Publishers.MapleSim.elementPublisher.accept(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
    }

    /**
     * Quick Helper since RobotModeTriggers only has a few triggers.
     */
    public static class FMSTriggers {
        public static final Trigger isHubActive = new Trigger(FMSTriggers::isHubActive);

        private static boolean isHubActive() {
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            // If we have no alliance, we cannot be enabled, therefore no hub.
            if (alliance.isEmpty()) {
                return false;
            }
            // Hub is always enabled in autonomous.
            if (DriverStation.isAutonomousEnabled()) {
                return true;
            }
            // At this point, if we're not teleop enabled, there is no hub.
            if (!DriverStation.isTeleopEnabled()) {
                return false;
            }
            // We're teleop enabled, compute.
            double matchTime = DriverStation.getMatchTime();
            String gameData = DriverStation.getGameSpecificMessage();
            // If we have no game data, we cannot compute, assume hub is active, as It's likely early in teleop.
            if (gameData.isEmpty()) {
                return true;
            }
            boolean redInactiveFirst;
            switch (gameData.charAt(0)) {
                case 'R' -> redInactiveFirst = true;
                case 'B' -> redInactiveFirst = false;
                default -> {
                    // If we have invalid game data, assume hub is active.
                    return true;
                }
            }
            // Shift was active for blue if red won auto, or red if blue won auto.
            boolean shift1Active = switch (alliance.get()) {
                case Red -> !redInactiveFirst;
                case Blue -> redInactiveFirst;
            };
            final int preEmptiveScoringTime = 1;
            if (matchTime > 130) { // Match time left greater than 130sec
                // Transition shift, hub is active.
                return true;
            } else if (matchTime > 105 + preEmptiveScoringTime) {
                // Shift 1
                return shift1Active;
            } else if (matchTime > 80 + preEmptiveScoringTime) {
                // Shift 2
                return !shift1Active;
            } else if (matchTime > 55 + preEmptiveScoringTime) {
                // Shift 3
                return shift1Active;
            } else if (matchTime > 30 + preEmptiveScoringTime) {
                // Shift 4
                return !shift1Active;
            } else {
                // End game, hub always active.
                return true;
            }
        }
    }

    /**
     * SmartDashboard wrapper to match NT4 Publishers.
     */
    public static class SmartDashboardPublisher
    {
        /**
         * The value published to the NT4.
         */
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
}

