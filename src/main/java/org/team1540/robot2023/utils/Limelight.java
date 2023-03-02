package org.team1540.robot2023.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

import org.team1540.robot2023.commands.drivetrain.Drivetrain;

public class Limelight {
    private double tv, tx, ty, ta;
    public boolean stopSteer = false;
    private double prev_tx = 1.0;
    private double backlashOffset = 0.0;
    private double tolerance = 0.01;
    private double adjustment = 0.0;
    private double prevHeading = 0;
    private PIDController pidController;
    private boolean newPIDLoop = false;
    DoubleArraySubscriber areasSub;
    double limelightMountAngleDegrees = 25.0; //change
    double limelightLensHeightInches = 20.0;
    private final NetworkTable table;
    public final String name;
    private double[] data;
    private final PoseZeroFilter zeroFilter = new PoseZeroFilter(50, 48);
    private final PoseMedianFilter medianFilter = new PoseMedianFilter(10);
    private double latency;
    private static final double HORIZONTAL_FOV = Math.toRadians(63.3);
    private static final double VERTICAL_FOV = Math.toRadians(49.7);

    public Limelight() {
        this("limelight");
    }

    public Limelight(String tablename) {
        name = tablename;
        table = NetworkTableInstance.getDefault().getTable(tablename);
        SmartDashboard.putNumber("Area Threshold", 0.02);
        SmartDashboard.putNumberArray("AutoAlign: PID Values", new double[]{0.015,0,0});
        SmartDashboard.putNumber("AutoAlign: Tolerance", tolerance);
        SmartDashboard.putNumber("AutoAlign: Backlash Offset", backlashOffset);
        SmartDashboard.setPersistent("Area Threshold");
        SmartDashboard.setPersistent("AutoAlign: PID Values");
        SmartDashboard.setPersistent("AutoAlign: Tolerance");
        SmartDashboard.setPersistent("AutoAlign: Backlash Offset");
        
        double[] pidValues = SmartDashboard.getNumberArray("AutoAlign: PID Values", new double[]{0.02,0,0});
        pidController = new PIDController(pidValues[0],pidValues[1],pidValues[2],1D/90D);
        pidController.setSetpoint(0);
        pidController.setTolerance(SmartDashboard.getNumber("AutoAlign: Tolerance", 0.01));
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
    
    //Sana's code 
    public double getDistanceFromLimelightToGoalInches(){
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        // distance from the target to the floor
        double goalHeightInches = 0.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        System.out.println("LL to Goal =" + distanceFromLimelightToGoalInches);
        return distanceFromLimelightToGoalInches;
    }

    public NetworkTable getLimelightTable() {
        return table;
    }
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public NetworkTable getNetworkTable() {
        return table;
    }

    //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public double getTx(){
         tx = table.getEntry("tx").getDouble(0.1234);
        SmartDashboard.putNumber("LimelightX", tx);
        return tx;
    }

    //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public double getTy(){
         ty = table.getEntry("ty").getDouble(0.1234);
        SmartDashboard.putNumber("LimelightY", ty);
        return ty;
    }

    //Target Area (0% of image to 100% of image)
    public double getTa(){
         ta = table.getEntry("ta").getDouble(0.1234);
        SmartDashboard.putNumber("LimelightArea", ta);
        return ta;
    }

    //Whether the limelight has any valid targets (0 or 1)
    public double getTv() {
        tv = table.getEntry("tv").getDouble(0.1234);
        SmartDashboard.putNumber("LimelightTargets", tv);
        return tv;
    }

    //	Class ID of primary neural classifier result
    public double getTclass(){
        double classID = table.getEntry("tclass").getDouble(0.0);
        SmartDashboard.putNumber("LimelightClassID", classID);
        return classID;
    }
    public Translation2d getTargetAngles() {
        double x = table.getEntry("tx").getDouble(0.1234);
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

    public double distanceAssist() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
        SmartDashboard.putNumber("Crosshair Vertical Offset", ty);
        double adjustment = 0.0;
        double area_threshold = 0;//1.75;
        double Kp = 0.225;
    
        if (tv == 1.0) {
          adjustment = (area_threshold - ta) * Kp;
        }
        adjustment = Math.signum(adjustment) * Math.min(Math.abs(adjustment), 0.5);
        return adjustment;
      }

    /*public double steeringAssist(Drivetrain dt) {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
        SmartDashboard.putNumber("Crosshair Horizontal Offset", tx);
        SmartDashboard.putNumber("Found Vision Target", tv);
        SmartDashboard.putNumber("Prev_tx", prev_tx);
        tx = Double.isNaN(tx) ? 0 : tx;
        double[] pidValues = SmartDashboard.getNumberArray("AutoAlign: PID Values", new double[]{0.015, 0, 0});
        pidController.setPID(pidValues[0], pidValues[1], pidValues[2]);
        pidController.setTolerance(SmartDashboard.getNumber("AutoAlign: Tolerance", 0.01));
      
        if (tv == 1.0 && !stopSteer) {
          if (ta > SmartDashboard.getNumber("Area Threshold", 0.02)) {
            adjustment = pidController.calculate(tx);
            prev_tx = tx;
            
            if (!newPIDLoop) {
              newPIDLoop = true;
              pidController.setSetpoint(Math.signum(prev_tx) * SmartDashboard.getNumber("AutoAlign: Backlash Offset", backlashOffset));
            }
          }
        } else {
          newPIDLoop = false;
          pidController.reset();
          adjustment = Math.signum(prev_tx); //* steering_factor;
        }
    
        if (Math.abs(tx) < 1.0 && Math.abs(prev_tx) < 1.0 && Math.abs(dt.getHeading() - prevHeading) < 1) stopSteer = true;
        else stopSteer = false;
        if(stopSteer) {
          adjustment = 0;
        }
        prevHeading = dt.getHeading();
    
        SmartDashboard.putBoolean("Stop Auto Steering", stopSteer);
    
        adjustment = Math.signum(tx) * Math.min(Math.abs(adjustment), 0.5);
        SmartDashboard.putNumber("Adjustment", adjustment);
        return adjustment;
    }

    public boolean isAligned() {
        return pidController.atSetpoint();
    }
    */

   

}
