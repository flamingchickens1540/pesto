package org.team1540.robot2023.commands.vision;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
                AutoDrive.driveToPoints(drivetrain, 0.5, 1, calculateEndPoint())
            )
        );
        addRequirements(drivetrain);
    }
    
     /**
     * @param Hypotenuse Hypotenuse to game piece in meters 
     * @return PathPoint of gamepiece 
     */
    public PathPoint calculateEndPoint() {
       double hypotenuseOutput = hypotenuseSupplier.getAsDouble(); 
       double height = Math.sin(drivetrain.getYaw().getDegrees()) * hypotenuseOutput; 
       double width = Math.cos(drivetrain.getYaw().getDegrees()) * hypotenuseOutput; 
       Translation2d currentTranslation = drivetrain.getPose().getTranslation(); 
       PathPoint point = new PathPoint(currentTranslation.plus(new Translation2d(width, height)), drivetrain.getYaw()); 
       return point; 
    }
}
