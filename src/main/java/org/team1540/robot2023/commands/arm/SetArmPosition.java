package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.Constants.ArmConstants;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.AverageFilter;

public class SetArmPosition extends CommandBase {

    private final double pivotAccel;
    Arm arm;
    ArmState setpoint;

    private final AverageFilter extensionFilter = new AverageFilter(10);
    private final AverageFilter rotationFilter = new AverageFilter(5);
    private final double extensionThreshold = 0.25;
    private final double rotationThreshold = 0.5;
    private boolean isExtending;
    private long extensionStartTime;
    private long extensionFinishTime;
    private double extensionDelay;
    private boolean shouldZero;


    public SetArmPosition(Arm arm, ArmState setpoint) {
        this(arm, setpoint, 0, true, 20000*(1+((ArmConstants.ARM_LENGTH_EXT-ArmConstants.ARM_BASE_LENGTH)-(setpoint.getExtension()-ArmConstants.ARM_BASE_LENGTH))/(ArmConstants.ARM_LENGTH_EXT-ArmConstants.ARM_BASE_LENGTH)));
    }


    public SetArmPosition(Arm arm, ArmState setpoint, double extensionDelay, boolean shouldZero, double pivotAccel) {
        this.arm = arm;
        this.setpoint = setpoint;
        this.extensionDelay = extensionDelay;
        this.shouldZero = shouldZero;
        this.pivotAccel = pivotAccel;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (shouldZero) {
            arm.resetToEncoder();
        }
        arm.setPivotAccel(pivotAccel);
        extensionStartTime = (long) (System.currentTimeMillis() + arm.timeToRotation(setpoint.getRotation2d()) - arm.timeToExtension(setpoint.getExtension()) + extensionDelay);
        extensionFinishTime = (long) (System.currentTimeMillis() + Math.max(arm.timeToRotation(setpoint.getRotation2d()) , (arm.timeToExtension(setpoint.getExtension())) + extensionDelay));
        arm.setExtension(arm.getArmState().getExtension());
        arm.setRotation(setpoint.getRotation2d());
        isExtending = false;
        extensionFilter.clear();
        rotationFilter.clear();
    }

    @Override
    public void execute() {
        if(!isExtending){
            if (System.currentTimeMillis() >= extensionStartTime){
                isExtending = true;
                arm.setExtension(setpoint.getExtension());
            }
        }
        extensionFilter.add(Math.abs(setpoint.getExtension() - arm.getArmState().getExtension()));
        rotationFilter.add(Math.abs(setpoint.getRotation2d().getDegrees() - arm.getArmState().getRotation2d().getDegrees()));
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > extensionFinishTime;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.stopAll();
        }
//        if (shouldZero) {
//            arm.resetToEncoder();
//        }
    }
}