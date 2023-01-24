package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.RollingAverage;
import org.team1540.robot2023.Constants.ArmConstants;

public class SetArmPosition extends CommandBase {
    private final Arm arm;
    private final double angle;
    private final double extension;

    private double rotThresh = 10;
    private double extThresh = 10;
    private boolean notSet = false;
    private final RollingAverage rotAvg = new RollingAverage(10);
    private final RollingAverage extAvg = new RollingAverage(10);

    public SetArmPosition(Arm arm, double x, double y){
        this.arm = arm;
        this.angle = Math.atan(x/y);

        if(x > ArmConstants.MAX_DISTANCE && x < ArmConstants.MAX_POINT_DISTANCE){
            this.extension = ArmConstants.ARM_LENGTH + 1;
        }
        else this.extension = Math.sqrt(x*x+y*y);

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

        rotAvg.add(arm.getAngleRadians() - angle);
        extAvg.add(arm.getExtension() - extension);
    }

    @Override
    public boolean isFinished() {
        return extAvg.getAverageAbs() < extThresh && rotAvg.getAverageAbs() < rotThresh;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}
