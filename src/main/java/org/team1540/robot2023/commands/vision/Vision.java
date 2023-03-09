package org.team1540.robot2023.commands.vision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.Limelight;


public class Vision extends SubsystemBase {
    private Limelight limelight = new Limelight();
    private Drivetrain drivetrain = new Drivetrain(); 
    private CommandXboxController controller; 


    public Vision( Drivetrain drivetrain ) {
        this.drivetrain = drivetrain; 
    }

    @Override
    public void periodic() {
        limelight.getTa();
        System.out.println("target area = " + limelight.getTa());
        System.out.println("horizontal offset = " + limelight.getTx());
        System.out.println("vertical offset = " + limelight.getTy());
        } 

}