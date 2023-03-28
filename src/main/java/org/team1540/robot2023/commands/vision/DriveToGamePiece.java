package org.team1540.robot2023.commands.vision;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;

public class DriveToGamePiece extends CommandBase{
    Drivetrain drivetrain;     

    public DriveToGamePiece(Drivetrain drivetrain){
        this.drivetrain = drivetrain; 
    }
    
     /**
     * @param Hypotenuse Hypotenuse to game piece in meters 
     * @return PathPoint of gamepiece 
     */
    public PathPoint calculateLength(double hypotenuse) {
       double length = Math.sin(drivetrain.getYaw().getDegrees()) * hypotenuse; 
       double width = Math.cos(drivetrain.getYaw().getDegrees()) * hypotenuse; 
       Translation2d currentTranslation = drivetrain.getPose().getTranslation(); 
       PathPoint point = new PathPoint(currentTranslation.plus(new Translation2d(width, length)), drivetrain.getYaw()); 
       return point; 
    }

    @Override
    public void execute() {
        calculateLength(3); 
    }
}
