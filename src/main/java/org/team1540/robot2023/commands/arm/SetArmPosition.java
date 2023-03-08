package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.AverageFilter;

public class SetArmPosition extends CommandBase {

    Arm arm;
    ArmState setpoint;

    private final AverageFilter extensionFilter = new AverageFilter(10);
    private final AverageFilter rotationFilter = new AverageFilter(5);
    private final double extensionThreshold = 0.25;
    private final double rotationThreshold = 0.5;
    private boolean isExtending;
    private long extensionStartTime;

    public SetArmPosition(Arm arm, ArmState setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
//        arm.setExtension(Math.max(setpoint.getExtension(), Constants.ArmConstants.ARM_BASE_LENGTH));
        extensionStartTime = (long) (System.currentTimeMillis() + arm.timeToRotation(setpoint.getRotation2d()) - arm.timeToExtension(setpoint.getExtension()));
        arm.setExtension(arm.getArmState().getExtension());
        arm.setRotation(setpoint.getRotation2d());
        isExtending = false;
        extensionFilter.clear();
        rotationFilter.clear();
    }

    @Override
    public void execute() {
        if(!isExtending){
            System.out.println("Extension Start: " + extensionStartTime + " Now: " + System.currentTimeMillis());
            System.out.println(extensionStartTime - System.currentTimeMillis());
            if(System.currentTimeMillis() >= extensionStartTime){
                isExtending = true;
                arm.setExtension(setpoint.getExtension());
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
        arm.stopAll();
    }
}