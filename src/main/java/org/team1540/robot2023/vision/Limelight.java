package org.team1540.robot2023.vision;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Limelight extends SubsystemBase {
    DoubleArraySubscriber areasSub;
    double limelightMountAngleDegrees = 25.0; //change
    double limelightLensHeightInches = 20.0;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight( String limelight){
    }

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
        double x = table.getEntry("tx").getDouble(0.1234);
        SmartDashboard.putNumber("LimelightX", x);
        return x;
    }

    //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public double getTy(){
        double y = table.getEntry("ty").getDouble(0.1234);
        SmartDashboard.putNumber("LimelightY", y);
        return y;
    }

    //Target Area (0% of image to 100% of image)
    public double getTa(){
        double area = table.getEntry("ta").getDouble(0.1234);
        SmartDashboard.putNumber("LimelightArea", area);
        return area;
    }

    //Whether the limelight has any valid targets (0 or 1)
    public boolean getTv() {
        double target = table.getEntry("tv").getDouble(0.1234);
        SmartDashboard.putNumber("LimelightTargets", target);
        return target != 0;
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
        return distance > 0 && distance < SmartDashboard.getNumber("limelight/custom/targetAlignedRange", 5);
    }

    /*public void setLeds(boolean isOn) {
        setLeds(isOn ? LEDMode.ON : LEDMode.OFF);

    }
    */

}