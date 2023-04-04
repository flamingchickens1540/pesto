package org.team1540.robot2023.commands.vision;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.DoubleSupplier;

import org.team1540.robot2023.commands.auto.AutoDrive;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;

public class DriveToGamePiece extends SequentialCommandGroup {
    Drivetrain drivetrain;     
    DoubleSupplier hypotenuseSupplier; 


    public DriveToGamePiece(Drivetrain drivetrain, DoubleSupplier hypotenuseSupplier){
        this.drivetrain = drivetrain; 
        this.hypotenuseSupplier = hypotenuseSupplier; 
        addCommands(
            new ProxyCommand(
                () -> AutoDrive.driveToPoints(drivetrain, 0.5, 1, calculateEndPoint())
            )
        );
        //addRequirements(drivetrain);
    }

     /**
     * @param Hypotenuse Hypotenuse to game piece in meters 
     * @return PathPoint of gamepiece 
     */
    public PathPoint calculateEndPoint() {
        System.out.println("DRIVE TO GAME PIECE "); 
        double hypotenuseOutput = hypotenuseSupplier.getAsDouble(); 
        double y = Math.sin(drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(180)).getRadians()) * hypotenuseOutput; 
        double x = Math.cos(drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(180)).getRadians()) * hypotenuseOutput; 
        SmartDashboard.putNumber("autos/width", x); 
        SmartDashboard.putNumber("autos/height", y); 
        System.out.println("width = " + x + "length" + y);
        Translation2d currentTranslation = drivetrain.getPose().getTranslation(); 
        System.out.println("curTranX = " + currentTranslation.getX() + "curTranY" + currentTranslation.getY());
        SmartDashboard.putNumber("autos/curTranX", currentTranslation.getX()); 
        SmartDashboard.putNumber("autos/curTranY", currentTranslation.getY()); 
        Translation2d pathTranslation = currentTranslation.plus(new Translation2d(x, y));
        PathPoint point = new PathPoint(pathTranslation, drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(180)),drivetrain.getPose().getRotation()); 
        System.out.println("newTranX = " + pathTranslation.getX() + "newTranY" + pathTranslation.getY());
        SmartDashboard.putNumber("autos/newTranx", pathTranslation.getX()); 
        SmartDashboard.putNumber("autos/newTranY",  pathTranslation.getY()); 
 
        return point; 
    }


}