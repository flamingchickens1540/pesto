package org.team1540.robot2023.commands.vision;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.lib.RevBlinkin;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.LimelightManager;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.BlinkinManager;
import org.team1540.robot2023.utils.Limelight;
import org.team1540.robot2023.utils.MathUtils;

import java.util.function.DoubleSupplier;

public class TurnToGamePiece extends CommandBase{
    Limelight limelight = LimelightManager.getInstance().frontLimelight;
    Drivetrain drivetrain;
    CommandXboxController controller; 
    DoubleSupplier angleSupplier;
    private final PIDController pid = new PIDController(Constants.Vision.kP, Constants.Vision.kI, Constants.Vision.kD);
    private boolean hasFoundTarget;
    private final GamePiece gamepiece;

    public enum GamePiece {
        CONE("cone", RevBlinkin.ColorPattern.YELLOW),
        CUBE("cube", RevBlinkin.ColorPattern.VIOLET);

        public final String identifier;
        public final RevBlinkin.ColorPattern pattern;
        GamePiece(String identifier, RevBlinkin.ColorPattern pattern) {
            this.pattern = pattern;
            this.identifier = identifier;
        }
    }

    public TurnToGamePiece(Drivetrain drivetrain, CommandXboxController controller, AHRS gyro, GamePiece gamepiece){
        this.drivetrain = drivetrain; 
        this.controller = controller;
        this.angleSupplier = gyro::getAngle;
        this.gamepiece = gamepiece;
    }
    public TurnToGamePiece(Drivetrain drivetrain, CommandXboxController controller, DoubleSupplier angleSupplier, GamePiece gamepiece){
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.angleSupplier = angleSupplier;
        this.gamepiece = gamepiece;
    }
    long time; 
    @Override
    public void initialize() {
        limelight.setLedState(Limelight.LEDMode.OFF);
        long start = System.currentTimeMillis();
        //limelight.setPipeline(Limelight.Pipeline.GAME_PIECE);
        limelight.setPipelineBad();
        limelight.setLedState(Limelight.LEDMode.ON);
        while(limelight.getLedState() == 1){
            System.out.println("switching pipeline"); 
        }
        long end = System.currentTimeMillis(); 
        time = end - start; 
        pid.enableContinuousInput(-180, 180);
//        updatePID();
        hasFoundTarget = false;
    }

     /**
     * Executes turning to the target using the Limelight as the primary sensor to determine whether we have turned enough.
     * angleXOffset the offset in degrees we still need to turn to reach the target
     */
    private void turnWithLimelightToCone() {
            double pidOutput = pid.calculate(angleSupplier.getAsDouble());
            SmartDashboard.putNumber("pointToTarget/pidOutput", pidOutput);
            drivetrain.drive(MathUtils.deadzone(-controller.getLeftY(), 0.1), 0,pidOutput, false);
    }

    private void updatePID() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", Constants.Vision.kP);//0.02
        double i = SmartDashboard.getNumber("pointToTarget/kI", Constants.Vision.kI);
        double d = SmartDashboard.getNumber("pointToTarget/kD", Constants.Vision.kD);//0.0015

        pid.setPID(p, i, d);
    }
    @Override
    public void execute() {
        System.out.println("time for pipeline to switch = " + time);
//        updatePID();
        //drivetrain.updateWithApriltags(); 
        if (hasFoundTarget) {
            BlinkinManager.setBoth(gamepiece.pattern);
            turnWithLimelightToCone();
        } else {
            if(limelight.getTa() != 0 && limelight.getTclass().equals(gamepiece.identifier)){

                double angleXOffset = limelight.getTx() - limelight.getTa() * -0.9;
                double gyroAngle = angleSupplier.getAsDouble();
                pid.setSetpoint(gyroAngle + angleXOffset);//*-0.698-2.99);  //16 (very sketchy constant) + angleOffset for back camera
                SmartDashboard.putBoolean("pointToTarget/turningWithLimelight", true);
                hasFoundTarget = true;

            } else {
                System.out.println("NO GAMEPIECE");
            }
        }

    }

    @Override
    public void end(boolean isInterrupted) {
        BlinkinManager.getInstance().set(BlinkinManager.ColorPair.TELEOP);
        limelight.setPipeline(Limelight.Pipeline.APRIL_TAGS);
    }
}
