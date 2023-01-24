package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmPosition extends CommandBase {
    private final Arm arm;
    private final double angle;
    private final double extension;

    private double angleThreshold = 10;
    private double extensionThreshold = 10;
    private int settle = 10;
    private int withinThreshold = 0;
    private boolean notSet = false;

    public SetArmPosition(Arm arm, double x, double y){
        this.arm = arm;
        this.angle = Math.atan(x/y);
        this.extension = Math.sqrt(x*x+y*y);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setAngleRadians(angle);
        arm.setExtension(extension);
    }

    @Override
    public void execute() {
        if(arm.getMaxExtension() < extension){
            arm.setExtension(arm.getMaxExtension());
            notSet = true;
        }
        else if(notSet){
            arm.setExtension(extension);
            notSet = false;
        }


        if(Math.abs(arm.getAngleRadians() - angle) < angleThreshold &&
                Math.abs(arm.getExtension() - extension) < extensionThreshold){
            withinThreshold += 1;
        }
        else {
            withinThreshold = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return withinThreshold > settle;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}
