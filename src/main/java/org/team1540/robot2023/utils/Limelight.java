package org.team1540.robot2023.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Arrays;

public class Limelight {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private static final PoseZeroFilter poseFilter = new PoseZeroFilter(50,48);
    private static final PoseMedianFilter medianFilter = new PoseMedianFilter(10);
    public static Pose2d getFilteredBotPose() {
        String key = DriverStation.getAlliance() == DriverStation.Alliance.Red ? "botpose_wpired" : "botpose_wpiblue";
        double[] data = table.getEntry(key).getDoubleArray(new double[7]);
//        System.out.println(key+":"+ Arrays.toString(data));
        if (data.length == 0 || Arrays.equals(data, new double[6])) {
            return null;
        }
        poseFilter.add(data);
        if (!poseFilter.isNonZero()) {
            return null;
        }
        Pose2d pose =  new Pose2d(data[0], data[1], new Rotation2d(Math.toRadians(data[5])));

        medianFilter.add(pose.getTranslation());

        return medianFilter.checkOutlier(pose).orElse(null);
//        String key = DriverStation.getAlliance() == DriverStation.Alliance.Red ? "botpose_wpired" : "botpose_wpiblue";
//        double[] data = table.getEntry(key).getDoubleArray(new double[0]);
//        poseFilter.add(data);
//
//        if (data.length == 0 || Arrays.equals(data, new double[6]) || !poseFilter.isNonZero()) {
//            return null;
//        }
//
//        return new Pose2d(data[0], data[1], new Rotation2d(Math.toRadians(data[5])));
    }


    public static Pose2d getBotPose() {
        String key = DriverStation.getAlliance() == DriverStation.Alliance.Red ? "botpose_wpired" : "botpose_wpiblue";
        double[] data = table.getEntry(key).getDoubleArray(new double[7]);

        if (data.length == 0 || Arrays.equals(data, new double[6])) {
            return null;
        }
        return new Pose2d(data[0], data[1], new Rotation2d(Math.toRadians(data[5])));
    }

    public enum LEDMode {
        OFF(1),
        BLINK(2),
        ON(3);

        private final int value;
        LEDMode(int value) {
            this.value = value;
        }
    }
    public static void setLedState(LEDMode mode) {
        table.getEntry("ledMode").setNumber(mode.value);
    }

    /**
     * Latency in ms of the pipeline
     *
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image
     *         capture latency.
     */
    public static double getDeltaTime() {
        return table.getEntry("tl").getDouble(0);
    }
}
