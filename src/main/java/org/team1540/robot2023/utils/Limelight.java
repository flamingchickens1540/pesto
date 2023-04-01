package org.team1540.robot2023.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

public class Limelight {
    private double tv, tx, ty, ta;
    private final NetworkTable table;
    //private final AHRS navx; 
    public final String name;
    private double[] data;
    private final PoseZeroFilter zeroFilter = new PoseZeroFilter(50, 48);
    private final PoseMedianFilter medianFilter = new PoseMedianFilter(10);
    private double latency;
    private static  double HORIZONTAL_FOV = Math.toRadians(63.3);
    private static  double VERTICAL_FOV = Math.toRadians(49.7);

    public Limelight(){
        this("limelight");
    }

    public Limelight(String tablename) {
        name = tablename;
        table = NetworkTableInstance.getDefault().getTable(name);
    }

    public void periodic() {
        String key = DriverStation.getAlliance() == DriverStation.Alliance.Red ? "botpose_wpired" : "botpose_wpiblue";
        double[] rawData = table.getEntry(key).getDoubleArray(new double[7]);
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
        if (data == null) return null;
        if (data.length == 0) return null;
        if (Arrays.equals(data, new double[data.length])) return null;
        if (!zeroFilter.isNonZero()) return null;

        Pose2d pose = new Pose2d(data[0], data[1], new Rotation2d(Math.toRadians(data[5])));
        return medianFilter.checkOutlier(pose).orElse(null);
    }


    public Pose2d getBotPose() {
        if (data == null) return null;
        if (data.length == 0) return null;
        if (Arrays.equals(data, new double[data.length])) return null;
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
    public double getLedState(){
        return table.getEntry("ledMode").getDouble(0); 
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

    public void setPipelineBad(){
       table.getEntry("pipeline").setDouble(1); 
    }

    public double getPipelineBad(){
        return  table.getEntry("pipeline").getDouble(3);  
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

    public String getNetworkTable() {
        return table.toString();
    }

    //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public double getTx(){
         tx = table.getEntry("tx").getDouble(0.0);
        SmartDashboard.putNumber("LimelightX", tx);
        return tx;
    }
    public NetworkTableEntry getTxEntry(){
        NetworkTableEntry txEntry = table.getEntry("tx");
       SmartDashboard.putNumber("vision/LimelightX", tx);
       return txEntry;
   }
   
    //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public double getTy(){
         ty = table.getEntry("ty").getDouble(0.0);
        SmartDashboard.putNumber("vision/LimelightY", ty);
        return ty;
    }

    //Target Area (0% of image to 100% of image)
    public double getTa(){
         ta = table.getEntry("ta").getDouble(0.0);
        SmartDashboard.putNumber("vision/LimelightArea", ta);
        return ta;
    }

    //Whether the limelight has any valid targets (0 or 1)
    public double getTv() {
        tv = table.getEntry("tv").getDouble(0.0);
        SmartDashboard.putNumber("vision/LimelightTargets", tv);
        return tv;
    }

    //	Class ID of primary neural detector result
    public String getTclass(){
        String classID = table.getEntry("tclass").getString("nothing");
        SmartDashboard.putString("vision/LimelightClassID", classID);
        return classID;
    }
    public Translation2d getTargetAngles() {
        double x = table.getEntry("tx").getDouble(0.0);
        double y = table.getEntry("ty").getDouble(0);
        return new Translation2d(x, y);
    }

    public boolean isTargetFound(){
        double angle = Math.abs(getTargetAngles().getX());
        return angle > 0.001;
    }

    public boolean isTargetAligned() {
        double distance = Math.abs(getTargetAngles().getX());
        return distance > 0 && distance < 8;
    }


    public double getHorizontalFov() {
        return HORIZONTAL_FOV;
    }

    public double getVerticalFov() {
        return VERTICAL_FOV;
    }
}
