package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.AverageFilter;

public class ResetArmPositionCommand extends CommandBase {

    Arm arm;
    private boolean shouldZero;
    ArmState setpoint;

    private final AverageFilter extensionFilter = new AverageFilter(10);
    private final AverageFilter rotationFilter = new AverageFilter(5);
    private final double extensionThreshold = 0.25;
    private final double rotationThreshold = 0.5;
    private boolean isRotating;
    private long pivotStartTime;
    private double extensionDelay;
    private long endTime;

    public ResetArmPositionCommand(Arm arm) {
        this(arm, true);
    }
    public ResetArmPositionCommand(Arm arm, boolean shouldZero) {
        this.arm = arm;
        this.shouldZero = shouldZero;
        this.setpoint = ArmState.fromRotationExtension(Rotation2d.fromDegrees(-15), Constants.ArmConstants.ARM_BASE_LENGTH);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (shouldZero) {
            arm.resetToEncoder();
        }
       System.out.println("command start");
//        System.out.println(arm.timeToExtension(Constants.ArmConstants.ARM_BASE_LENGTH));
        pivotStartTime = (long) (System.currentTimeMillis() + arm.timeToExtension(setpoint.getExtension())/5);
        endTime = (long) (System.currentTimeMillis() + 0.9*arm.timeToExtension(setpoint.getExtension()));
        arm.setExtension(setpoint.getExtension());
        arm.setRotation(arm.getArmState().getRotation2d());
        isRotating = false;
        extensionFilter.clear();
        rotationFilter.clear();
    }

    @Override
    public void execute() {
        if(!isRotating){
            if(System.currentTimeMillis() >= pivotStartTime){
                isRotating = true;
                arm.setRotation(setpoint.getRotation2d());
                endTime = (long) Math.max(endTime, System.currentTimeMillis() + arm.timeToRotation(setpoint.getRotation2d()));
            }
        }
        extensionFilter.add(Math.abs(setpoint.getExtension() - arm.getArmState().getExtension()));
        rotationFilter.add(Math.abs(setpoint.getRotation2d().getDegrees() - arm.getArmState().getRotation2d().getDegrees()));
    }

    @Override
    public boolean isFinished() {
//        System.out.println(rotationFilter.getAverage() + "      " + Math.abs(arm.getArmState().getRotation2d().getDegrees() - setpoint.getRotation2d().getDegrees()));
        return (
                ((setpoint.getExtension() <= Constants.ArmConstants.ARM_BASE_LENGTH && arm.getLimitSwitch()) ||
                        extensionFilter.getAverage() < extensionThreshold) &&
                        (rotationFilter.getAverage() < rotationThreshold &&
                                Math.abs(arm.getArmState().getRotation2d().getDegrees() - setpoint.getRotation2d().getDegrees()) < rotationThreshold)
        ) || System.currentTimeMillis() > endTime;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.stopAll();
        }
        if (shouldZero) {
            arm.resetToEncoder();
        }
        System.out.println("Command end");
    }
}