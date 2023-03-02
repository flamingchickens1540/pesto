package org.team1540.robot2023.vision;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.Limelight;


public class Vision extends CommandBase {
    private Limelight limelight = new Limelight("");
    private Drivetrain drivetrain = new Drivetrain(); 

    public Vision(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain; 
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        limelight.getDistanceFromLimelightToGoalInches();
        limelight.getTa();
        System.out.println("target area = " + limelight.getTa());
        double y = limelight.getTy();
        double x = limelight.getTx();
        System.out.println("horizontal offset = " + x);
        System.out.println("vertical offset = " + y);
        limelight.isTargetAligned();
    }

}