package org.team1540.robot2023.commands.vision;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.lib.RevBlinkin;
import org.team1540.robot2023.LimelightManager;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.BlinkinManager;
import org.team1540.robot2023.utils.Limelight;
import org.team1540.robot2023.utils.MathUtils;

public class TurnToGamePiece extends CommandBase{
    Limelight limelight = LimelightManager.getInstance().frontLimelight;
    Drivetrain drivetrain;
    CommandXboxController controller; 
    AHRS gyro; 
    private final PIDController pid = new PIDController(1, 0, 0);
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
        this.gyro = gyro;
        this.gamepiece = gamepiece;
    }

    @Override
    public void initialize() {
        limelight.setPipeline(Limelight.Pipeline.GAME_PIECE);
        pid.enableContinuousInput(-180, 180);
        updatePID();

//        System.out.println("PTT Initialized");
    }

     /**
     * Executes turning to the target using the Limelight as the primary sensor to determine whether we have turned enough.
     * angleXOffset the offset in degrees we still need to turn to reach the target
     */
    private void turnWithLimelightToCone() {
         //&& limelight.getTclass() == 0){ // check if class id = 0
//            System.out.println("table" + limelight.getNetworkTable());
//            System.out.println("table = " + limelight.getNetworkTable());
//            System.out.println("cone being detected ");
//            System.out.println("class ID" + limelight.getTclass());
//            System.out.println("cone angleXOffset" + angleXOffset);
//            System.out.println(gyroAngle);

            double pidOutput = pid.calculate(gyro.getAngle()); 
            SmartDashboard.putNumber("pointToTarget/pidOutput", pidOutput);
            drivetrain.drive(MathUtils.deadzone(-controller.getLeftY(), 0.1), MathUtils.deadzone(-controller.getLeftX(),0.1),pidOutput, false);
        
    }

    private void updatePID() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", -0.03);//0.02
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", -0.002);//0.0015

        pid.setPID(p, i, d);
    }
    @Override
    public void execute() {
        updatePID();
        if (hasFoundTarget) {
            BlinkinManager.setBoth(gamepiece.pattern);
            turnWithLimelightToCone();
        } else {
            if(limelight.getTa() != 0 && limelight.getTclass().equals(gamepiece.identifier)){

                double angleXOffset = limelight.getTx() - limelight.getTa() * -0.9;
                double gyroAngle = gyro.getAngle();
                pid.setSetpoint(gyroAngle + angleXOffset);//*-0.698-2.99);  //16 (very sketchy constant) + angleOffset for back camera
                SmartDashboard.putBoolean("pointToTarget/turningWithLimelight", true);
                hasFoundTarget = true;

            } else {
                System.out.println("NO GAMEPIECE");
            }
        }

//        System.out.println("tx = " + limelight.getTx());
//        System.out.println("ty = " + limelight.getTy());
//        System.out.println("ta = " + limelight.getTa());
//        System.out.print("class ID = " + limelight.getTclass());
    }

    @Override
    public void end(boolean isInterrupted) {
        BlinkinManager.getInstance().set(BlinkinManager.ColorPair.TELEOP);
        limelight.setPipeline(Limelight.Pipeline.APRIL_TAGS);
    }
}
