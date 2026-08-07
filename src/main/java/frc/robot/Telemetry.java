package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.motorcontrollers.SmartMotorControllerConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

public class Telemetry {
    /// Current Telemetry Setting
    public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;
    /// Telemetry Defaults
    public static final String telemetryPath = "SmartDashboard/Telemetry"; // Make access public for other telemetry.
    public static final String smartDashboardPath = "Telemetry/";
    public static final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable(telemetryPath);
    public static final String yamsMechPath = "Telemetry/Mechanisms/";

    /// Publishers
    public static class Publishers {

        public static class Field {
            public static final NetworkTable fieldTable = telemetryTable.getSubTable("Field");
            public static final BooleanPublisher wasActiveFirst = fieldTable.getBooleanTopic("Was Active First").publish(); // True when our hub is active
            public static final BooleanPublisher activeHub = fieldTable.getBooleanTopic("Hub Active").publish(); // True when our hub is active
            //public static final BooleanPublisher blinkTimer = fieldTable.getBooleanTopic("Blink Timer").publish(); // Flashes whenever close to a time change. Auton, shift change, end game.
            public static final DoublePublisher shiftTime = fieldTable.getDoubleTopic("Shift Timer").publish(); // Relays time until shift change
            public static final DoublePublisher matchTime = fieldTable.getDoubleTopic("Match Timer").publish(); // Match timer.
        }

        /// Robot Publishers
        public static class Robot {
            /// Telemetry Paths
            // Zones
            public static final NetworkTable zoneTable = telemetryTable.getSubTable("Zones");
            public static final String smartDashboardBrushedPath = yamsMechPath + "Brushed Telemetry/";
            // Input Selection
            public static final SmartDashboardPublisher inputPublisher = new SmartDashboardPublisher(smartDashboardPath + "Input Selector");
            public static final StringPublisher inputOverride = zoneTable.getSubTable("Input Selector").getStringTopic("selected").publish();

            /// Mech3d Publishers
            public static class Mech3D {
                public static final NetworkTable mechTable = zoneTable.getSubTable("Visualize Telemetry");
                public static final StructPublisher<Pose2d> robotPose = mechTable.getStructTopic("Robot Pose", Pose2d.struct).publish();
                public static final StructPublisher<Pose3d> hoodPose = mechTable.getStructTopic("Hood Pose", Pose3d.struct).publish();
                public static final StructPublisher<Pose3d> hopperPose = mechTable.getStructTopic("Hopper Pose", Pose3d.struct).publish();
                public static final StructPublisher<Pose3d> turretPose = mechTable.getStructTopic("Turret Pose", Pose3d.struct).publish();
                public static final StructArrayPublisher<Pose3d> fuelTrajectory = mechTable.getStructArrayTopic("Shot Trajectory", Pose3d.struct).publish();
            }

            public static class Vision {
                private static final NetworkTable visionTable = telemetryTable.getSubTable("Vision");
                private static final List<Pair<Pose3d, StructPublisher<Pose3d>>> cameraPosePublishers = new ArrayList<>();

                public static void publishCameraPose(String camName, Rotation3d camAngle, Translation3d camPose) {
                    cameraPosePublishers.add(Pair.of(new Pose3d(camPose, camAngle), visionTable.getStructTopic(camName + " Pose", Pose3d.struct).publish()));
                }
            }

        }

        /// MapleSim Publishers
        public static class MapleSim {
            // Table for maple sim publishers.
            private static final NetworkTable mapleTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim");
            // Generic Game Piece Publisher.
            public static final StructArrayPublisher<Pose3d> elementPublisher = mapleTable.getStructArrayTopic("Fuel", Pose3d.struct).publish();
            // Generic Game Piece Publisher.
            public static final StructArrayPublisher<Pose3d> elementInRobotPublisher = mapleTable.getStructArrayTopic("Fuel in Robot", Pose3d.struct).publish();
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
    public static void updateTelemetry(InputBuilder.Subsystems subsystem) {
        // Input
        Publishers.Robot.inputPublisher.update();
        Publishers.Robot.Mech3D.robotPose.set(subsystem.swerve().getSwerveDrive().getPose());

        // Field
        Publishers.Field.activeHub.accept(FMSTriggers.getWasActiveFirst());
        Publishers.Field.activeHub.accept(FMSTriggers.isHubActive());
        Publishers.Field.shiftTime.accept(FMSTriggers.getCurrentShift().getNextShift().timeUntilThisShift());
        Publishers.Field.matchTime.accept((int) (DriverStation.getMatchTime()));

        if (RobotBase.isSimulation()) {
            // MapleSim
            Publishers.MapleSim.elementPublisher.accept(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
    }

    /**
     * Updates camera 3D poses based on the swerve position.
     *
     * @param swervePose the swerve pose.
     */
    public static void updateCameraPoses(Pose3d swervePose) {
        for (Pair<Pose3d, StructPublisher<Pose3d>> pair : Publishers.Robot.Vision.cameraPosePublishers) {
            // Publish the camera pose relative to the robot.
            pair.getSecond().accept(swervePose.plus(new Transform3d(pair.getFirst().getTranslation(), pair.getFirst().getRotation())));
        }
    }

    /**
     * Time Constants
     * The event happens when the match timer is at this integer.
     */
    public enum Shifts {
        AUTONOMOUS(160),
        TRANSITION_SHIFT(140),
        FIRST_SHIFT(130),
        SECOND_SHIFT(105),
        THIRD_SHIFT(80),
        FOURTH_SHIFT(55),
        END_GAME(30);

        /// Match timer when this shift starts
        @Getter
        private final int matchTimeLeft;

        /**
         * Shift time management
         *
         * @param matchTimeLeft match timer when this shift starts.
         */
        Shifts(int matchTimeLeft) {
            this.matchTimeLeft = matchTimeLeft;
        }

        /**
         * A boolean check for whether the shift change has passed minus* this time.
         * *Match timer counts down, this adds seconds to the time left value.
         *
         * @param timeBefore time to add to the matchTimeLeft timer
         * @return whether the timeBefore the shift has passed.
         */
        public boolean timeUntilWithin(int timeBefore) {
            return DriverStation.getMatchTime() <= timeBefore + matchTimeLeft;
        }

        /**
         * Time until this shift will start.
         *
         * @return time until this shift will change.
         */
        public int timeUntilThisShift() {
            return (int) (DriverStation.getMatchTime() - matchTimeLeft);
        }

        /**
         * Checks if the hub was active given wasActiveFirst.
         *
         * @param wasActiveFirst whether our hub was active first.
         * @return whether the hub is active during this shift.
         */
        public boolean isHubActive(boolean wasActiveFirst) {
            return switch (this) {
                case FIRST_SHIFT, THIRD_SHIFT -> wasActiveFirst;
                case SECOND_SHIFT, FOURTH_SHIFT -> !wasActiveFirst;
                default -> true;
            };
        }

        /**
         * Whether this shift is active.
         *
         * @return whether this shift is active.
         */
        public boolean isCurrentShift() {
            final int matchTime = (int) (DriverStation.getMatchTime());
            // start time left is greater than current time left.
            return matchTimeLeft >= matchTime
                    // and match time is less then next shift start.
                    && matchTime <= getNextShift().getMatchTimeLeft();
        }

        /**
         * Whether this shift is active.
         *
         * @param timeBefore time to add to the matchTimeLeft timer
         * @return whether this shift is active.
         */
        public boolean isCurrentShift(int timeBefore) {
            final int matchTime = (int) (DriverStation.getMatchTime());
            // start time left is greater than current time left.
            return timeBefore + matchTimeLeft >= matchTime
                    // and match time is less then next shift start.
                    && matchTime <= getNextShift().getMatchTimeLeft();
        }

        /**
         * Get the next shift
         *
         * @return the next shift.
         */
        public Shifts getNextShift() {
            return Shifts.values()[ordinal() + 1];
        }
    }

    /**
     * Quick Helper since RobotModeTriggers only has a few triggers.
     */
    public static class FMSTriggers {
        private static final int preemptiveHubActiveTime = 1;
        public static final Trigger isHubActive = new Trigger(FMSTriggers::isHubActive);
        public static int currentShift;
        public static int nextShift;
        public static Boolean wasActiveFirst = null; // Fancy boolean for null

        static {
            // Update whether we were the first active shift after the first shift starts.
            new Trigger(() -> Shifts.FIRST_SHIFT.getMatchTimeLeft() >= DriverStation.getMatchTime())
                    .onTrue(Commands.runOnce(FMSTriggers::updateWasActiveFirst));
        }

        /**
         * Safely gets wasActiveFirst.
         *
         * @return whether our hub was active during the first shift.
         */
        public static boolean getWasActiveFirst() {
            return Objects.requireNonNullElse(wasActiveFirst, false);
        }

        /**
         * Checks to see which shift is currently active.
         *
         * @return the current shift.
         */
        public static Shifts getCurrentShift() {
            for (Shifts shift : Shifts.values()) {
                if (shift.isCurrentShift()) {
                    return shift;
                }
            }
            return Shifts.AUTONOMOUS;
        }

        /**
         * Checks to see which shift is currently active using the preemptiveHubActiveTime
         *
         * @return the current shift.
         */
        public static Shifts getPreemptiveCurrentShift() {
            for (Shifts shift : Shifts.values()) {
                if (shift.isCurrentShift(preemptiveHubActiveTime)) {
                    return shift;
                }
            }
            return Shifts.AUTONOMOUS;
        }

        /**
         * Updates the was active first static boolean.
         *
         * @return a fresh wasActiveFirst
         */
        public static boolean updateWasActiveFirst() {
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
            wasActiveFirst = switch (alliance.get()) {
                case Red -> !redInactiveFirst;
                case Blue -> redInactiveFirst;
            };
            return wasActiveFirst;
        }

        /**
         * Checks whether the hub is active.
         * Adds the preemptive scoring delay.
         *
         * @return whether the hub is active.
         */
        public static boolean isHubActive() {
            // If not set, it's before the First Shift.
            if (wasActiveFirst == null) {
                return true;
            }
            return getPreemptiveCurrentShift().isHubActive(wasActiveFirst);
        }
    }

    /**
     * SmartDashboard wrapper to match NT4 Publishers.
     */
    public static class SmartDashboardPublisher {
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
        public SmartDashboardPublisher(String path) {
            this.path = path;
        }

        /**
         * Puts the value onto the dashboard.
         *
         * @param value to publish.
         */
        public void setValue(Sendable value) {
            this.value = value;
            SmartDashboard.putData(path, value);
        }

        /**
         * Accepts sendable to be updated later.
         * Also sets when ran.
         *
         * @param supplier the value supplier.
         */
        public void accept(Supplier<Sendable> supplier) {
            this.supplier = supplier;
            setValue(supplier.get());
        }

        /**
         * Updates the published value from the current supplier.
         */
        public void update() {
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
                SwerveDriveTelemetry.TelemetryVerbosity.HIGH),
        ;

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

