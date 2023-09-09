package org.team1540.robot2023.commands.vision;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.LimelightManager;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.*;

import java.util.function.DoubleSupplier;

public class TurnToGamePiece extends CommandBase{
    
    Drivetrain drivetrain;
    CommandXboxController controller; 
    DoubleSupplier angleSupplier;
    private final PIDController pid = new PIDController(Constants.Vision.kP, Constants.Vision.kI, Constants.Vision.kD);
    private boolean hasFoundTarget;
    private final GamePiece gamepiece;
    private double angleXOffset; 
    private AverageFilter averageFilter = new AverageFilter(10); 
    private double startTime; 

    //  if(controller != null){
        Limelight limelight = LimelightManager.getInstance().rearLimelight; 
    //}
    // else{
        //Limelight limelight = LimelightManager.getInstance().rearLimelight;

    ///}

    public TurnToGamePiece(Drivetrain drivetrain, CommandXboxController controller, AHRS gyro, GamePiece gamepiece){
        this.drivetrain = drivetrain; 
        this.controller = controller;
        this.angleSupplier = gyro::getAngle;
        this.gamepiece = gamepiece;
        // if(controller != null){
        //     addRequirements(drivetrain);
        // }
    }
    public TurnToGamePiece(Drivetrain drivetrain, CommandXboxController controller, GamePiece gamepiece){
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.angleSupplier = drivetrain::getRawGyroAngle;
        this.gamepiece = gamepiece;
        // if(controller != null){
        //     addRequirements(drivetrain);
        // }

    }
    public TurnToGamePiece(Drivetrain drivetrain, CommandXboxController controller, GamePiece gamepiece, Limelight limelight){
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.angleSupplier = drivetrain::getRawGyroAngle;
        this.gamepiece = gamepiece;
        this.limelight = limelight; 
        // if(controller != null){
        //     addRequirements(drivetrain);
        // }

    }
    

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis(); 
        limelight.setPipeline(Limelight.Pipeline.GAME_PIECE);
        pid.enableContinuousInput(-180, 180);
//        updatePID();
        hasFoundTarget = false;
    }

     /**
     * Executes turning to the target using the Limelight as the primary sensor to determine whether we have turned enough.
     * angleXOffset the offset in degrees we still need to turn to reach the target
     */
    private void turnWithLimelightToCone() {
        //if((System.currentTimeMillis() - startTime) == 500 && controller != null){
            double pidOutput = pid.calculate(angleSupplier.getAsDouble());
            SmartDashboard.putNumber("pointToTarget/pidOutput", pidOutput);
            if (controller != null) {
                drivetrain.drive(MathUtils.deadzone(-controller.getLeftY(), 0.1), 0,pidOutput, false);
            } else {
                drivetrain.drive(0, 0,pidOutput, false);
            }
            averageFilter.add(Math.abs(pid.getPositionError()));
        //}
    }

    private void updatePID() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", Constants.Vision.kP);//0.02
        double i = SmartDashboard.getNumber("pointToTarget/kI", Constants.Vision.kI);
        double d = SmartDashboard.getNumber("pointToTarget/kD", Constants.Vision.kD);//0.0015
        pid.setPID(p, i, d);
    }

    @Override
    public void execute() {
//        updatePID();
    
        if (hasFoundTarget) {
            BlinkinManager.setBoth(gamepiece.pattern);
            turnWithLimelightToCone();
        } else {
            if(limelight.getTa() != 0 && limelight.getTclass().equals(gamepiece.identifier)){
                System.out.println("SEEING GAME PIECE "); 
                angleXOffset = limelight.getTx() - limelight.getTa() * -0.9;
                double gyroAngle = angleSupplier.getAsDouble();
                pid.setSetpoint(gyroAngle + angleXOffset);//*-0.698-2.99);  //16 (very sketchy constant) + angleOffset for back camera
                SmartDashboard.putBoolean("pointToTarget/turningWithLimelight", true);
                hasFoundTarget = true; //was not there

            } else {
                System.out.println("NO GAMEPIECE");
            }
        }

    }

    @Override
    public void end(boolean isInterrupted) {
        BlinkinManager.getInstance().set(BlinkinManager.ColorPair.TELEOP);
        limelight.setPipeline(Limelight.Pipeline.APRIL_TAGS);
        System.out.println("ENDING TurnToGamePiece");
    }

    
    @Override
    public boolean isFinished(){
        averageFilter.add(Math.abs(pid.getPositionError()));
        System.out.println("average" + averageFilter.getAverage()); 
        return (Math.abs(averageFilter.getAverage()) < 2 && hasFoundTarget); // was 0.2 
    }
}