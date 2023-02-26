package org.team1540.robot2023.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Arrays;

public class Limelight {
    private final NetworkTable table;
    public final String name;
    private double[] data;
    private final PoseZeroFilter zeroFilter = new PoseZeroFilter(50, 48);
    private final PoseMedianFilter medianFilter = new PoseMedianFilter(10);
    private double latency;

    public Limelight() {
        this("limelight");
    }

    public Limelight(String tablename) {
        name = tablename;
        table = NetworkTableInstance.getDefault().getTable(tablename);
    }

    public void periodic() {
        String key = DriverStation.getAlliance() == DriverStation.Alliance.Red ? "botpose_wpired" : "botpose_wpiblue";
        double[] rawData = table.getEntry(key).getDoubleArray(new double[0]);
        latency = rawData[6];
        data = Arrays.copyOf(rawData, 6); 
        zeroFilter.add(data);

        Translation2d pose;
        if (data.length < 2) {
            pose = new Translation2d();
        } else {
            pose = new Translation2d(data[0], data[1]);
        }
        medianFilter.add(pose);
    }

    public Pose2d getFilteredBotPose() {
        // check if data is zero or empty
        if (data.length == 0) return null;
        if (Arrays.equals(data, new double[data.length])) return null;
        if (!zeroFilter.isNonZero()) return null;

        Pose2d pose = new Pose2d(data[0], data[1], new Rotation2d(Math.toRadians(data[5])));
        return medianFilter.checkOutlier(pose).orElse(null);
    }


    public Pose2d getBotPose() {
        if (data.length == 0 || Arrays.equals(data, new double[6])) {
            return null;
        }
        return new Pose2d(data[0], data[1], new Rotation2d(Math.toRadians(data[5])));
    }

    public enum LEDMode {
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);

        private final int value;

        LEDMode(int value) {
            this.value = value;
        }
    }

    public enum Pipeline {
        APRIL_TAGS(0),
        GAME_PIECE(1);

        private final int value;
        Pipeline(int value) {this.value = value;}
    }

    public void setLedState(LEDMode mode) {
        table.getEntry("ledMode").setNumber(mode.value);
    }
    public void setDriverMode(boolean isDriverMode) {
        table.getEntry("camMode").setNumber(isDriverMode ? 1 : 0);
    }
    public void setPipeline(Pipeline pipeline) {
        table.getEntry("pipeline").setNumber(pipeline.value);
    }

    public Pipeline getPipeline() {
        int currentPipeline = (int) table.getEntry("pipeline").getInteger(0);
        for (Pipeline pipeline : Pipeline.values()) {
            if (pipeline.value == currentPipeline) {
                return pipeline;
            }
        }
        return Pipeline.APRIL_TAGS;
    }

    /**
     * Latency in ms of the pipeline
     *
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image
     * capture latency.
     */
    public double getDeltaTime() {
        return latency;
    }
}
