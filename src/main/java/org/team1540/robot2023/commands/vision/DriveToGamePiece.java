package org.team1540.robot2023.commands.vision;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.DoubleSupplier;

import org.team1540.robot2023.commands.auto.AutoDrive;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;

public class DriveToGamePiece extends CommandBase {
    Drivetrain drivetrain;     
    DoubleSupplier hypotenuseSupplier; 
   

    public DriveToGamePiece(Drivetrain drivetrain, DoubleSupplier hypotenuseSupplier){
        this.drivetrain = drivetrain; 
        this.hypotenuseSupplier = hypotenuseSupplier; 
        System.out.println("IN DriveToGamePiece");         
        //  addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule( AutoDrive.driveToPoints(drivetrain, 0.5, 1, calculateEndPoint())); 
        
    }
    
     /**
     * @param Hypotenuse Hypotenuse to game piece in meters 
     * @return PathPoint of gamepiece 
     */
    public PathPoint calculateEndPoint() {
        System.out.println("DRIVE TO GAME PIECE "); 
       double hypotenuseOutput = hypotenuseSupplier.getAsDouble(); 
       double x = Math.sin(drivetrain.getYaw().getDegrees()) * hypotenuseOutput; 
       double y = Math.cos(drivetrain.getYaw().getDegrees()) * hypotenuseOutput; 
       SmartDashboard.putNumber("autos/width", x); 
       SmartDashboard.putNumber("autos/height", y); 
       System.out.println("width = " + x + "length" + y);
       Translation2d currentTranslation = drivetrain.getPose().getTranslation(); 
       System.out.println("curTranX = " + currentTranslation.getX() + "curTranY" + currentTranslation.getY());
       SmartDashboard.putNumber("autos/curTranX", currentTranslation.getX()); 
       SmartDashboard.putNumber("autos/curTranY", currentTranslation.getY()); 
       Translation2d pathTranslation = currentTranslation.plus(new Translation2d(x, y));
       PathPoint point = new PathPoint(pathTranslation, drivetrain.getYaw()); 
       System.out.println("newTranX = " + pathTranslation.getX() + "newTranY" + pathTranslation.getY());
       SmartDashboard.putNumber("autos/newTranx", pathTranslation.getX()); 
       SmartDashboard.putNumber("autos/newTranY",  pathTranslation.getY()); 

       return point; 
    }

    @Override
    public void execute() {
       
    }

    @Override
    public boolean isFinished() {
        return true; 
    }
}
