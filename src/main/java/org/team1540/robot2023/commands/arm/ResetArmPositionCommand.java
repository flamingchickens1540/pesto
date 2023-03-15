package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.AverageFilter;

public class ResetArmPositionCommand extends CommandBase {

    Arm arm;
    ArmState setpoint;

    private final AverageFilter extensionFilter = new AverageFilter(10);
    private final AverageFilter rotationFilter = new AverageFilter(5);
    private final double extensionThreshold = 0.25;
    private final double rotationThreshold = 0.5;
    private boolean isRotating;
    private long pivotStartTime;
    private double extensionDelay;

    public ResetArmPositionCommand(Arm arm){
        this.arm = arm;
        this.setpoint = ArmState.fromRotationExtension(Rotation2d.fromDegrees(0), Constants.ArmConstants.ARM_BASE_LENGTH);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.resetToEncoder();
        System.out.println("command start");
        System.out.println(arm.timeToExtension(Constants.ArmConstants.ARM_BASE_LENGTH));
        pivotStartTime = (long) (System.currentTimeMillis() + arm.timeToExtension(setpoint.getExtension())/5);
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
            }
        }
        extensionFilter.add(Math.abs(setpoint.getExtension() - arm.getArmState().getExtension()));
        rotationFilter.add(Math.abs(setpoint.getRotation2d().getDegrees() - arm.getArmState().getRotation2d().getDegrees()));
    }

    @Override
    public boolean isFinished() {
        return (
                ((setpoint.getExtension() <= Constants.ArmConstants.ARM_BASE_LENGTH && arm.getLimitSwitch()) ||
                        extensionFilter.getAverage() < extensionThreshold) &&
                        (rotationFilter.getAverage() < rotationThreshold &&
                                Math.abs(arm.getArmState().getRotation2d().getDegrees() - setpoint.getRotation2d().getDegrees()) < rotationThreshold)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.holdPivot();
            arm.holdExtension();
        } else arm.resetToEncoder();
    }
}