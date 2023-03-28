package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberAggressiveCommand extends CommandBase {
    private final WheeledGrabber wheeledGrabber;
    private int count;
    public GrabberAggressiveCommand(WheeledGrabber wheeledGrabber) {
        this.wheeledGrabber = wheeledGrabber;
        addRequirements(wheeledGrabber);
        count = 0;
        SmartDashboard.putNumber("grabber/aggressiveIn", 1);
        SmartDashboard.putNumber("grabber/aggressiveOut", 0);
    }

    @Override
    public void initialize() {
        wheeledGrabber.setCurrentLimit(30);
    }

    @Override
    public void execute() {
        int in = (int) SmartDashboard.getNumber("grabber/aggressiveIn", 1);
        int out = (int) SmartDashboard.getNumber("grabber/aggressiveOut", 0);
        count += 1;
        if(count <= in){
            wheeledGrabber.setSpeed(1.0);
        }
        else if(count <= in + out){
            wheeledGrabber.setSpeed(-0.3);
        }
        else count = 0;
    }

    @Override
    public void end(boolean interrupted) {
        wheeledGrabber.setCurrentLimit(10);
    }
}
