package org.team1540.robot2023.commands.vision;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import org.team1540.robot2023.commands.auto.AutoDrive;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;

public class DriveToGamePiece extends CommandBase{
    Drivetrain drivetrain;     
    DoubleSupplier hypotenuse; 
    double hypotenuseOutput = hypotenuse.getAsDouble(); 

    public DriveToGamePiece(Drivetrain drivetrain, DoubleSupplier hypotenus){
        this.drivetrain = drivetrain; 
        this.hypotenuse = hypotenus; 
    }
    
     /**
     * @param Hypotenuse Hypotenuse to game piece in meters 
     * @return PathPoint of gamepiece 
     */
    public PathPoint calculateLength() {
       double length = Math.sin(drivetrain.getYaw().getDegrees()) * hypotenuseOutput; 
       double width = Math.cos(drivetrain.getYaw().getDegrees()) * hypotenuseOutput; 
       Translation2d currentTranslation = drivetrain.getPose().getTranslation(); 
       PathPoint point = new PathPoint(currentTranslation.plus(new Translation2d(width, length)), drivetrain.getYaw()); 
       return point; 
    }

    @Override
    public void execute() {
        calculateLength(); 
    }
}
