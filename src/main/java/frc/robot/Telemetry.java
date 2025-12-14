package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

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
            private static final NetworkTable robotTable = telemetryTable.getSubTable("RobotTelemetry");
            private static final String smartDashboardRobotPath = smartDashboardPath + "RobotTelemetry/";
            public static final SmartDashboardPublisher inputPublisher = new SmartDashboardPublisher(smartDashboardRobotPath + "Input Selector");
            public static final StringPublisher inputOverride = robotTable.getSubTable("Input Selector").getStringTopic("selected").publish();
        }
        /// MapleSim Publishers
        public static class MapleSim
        {
            // Table for maple sim publishers.
            private static final NetworkTable mapleTable = telemetryTable.getSubTable("MapleSim");
            // Generic Game Piece Publisher.
            public static final StructArrayPublisher<Pose3d> elementPublisher = mapleTable.getStructArrayTopic("GameElement", Pose3d.struct).publish();
        }
    }

    /// Telemetry Verbosity Settings
    public enum TelemetryVerbosity {
        /// No telemetry data is sent to the dashboard.
        NONE,
        /// Only basic telemetry data is sent to the dashboard.
        LOW,
        /// All telemetry data is sent to the dashboard.
        HIGH
    }


    public static class SmartDashboardPublisher
    {
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

        public Sendable getValue()
        {
            return value;
        }

        public void set(Sendable value)
        {
            this.value = value;
            SmartDashboard.putData(path, value);
        }

        public void accept(Supplier<Sendable> supplier)
        {
            this.supplier = supplier;
            set(supplier.get());
        }

        public void update()
        {
            set(supplier.get());
        }
    }
}

