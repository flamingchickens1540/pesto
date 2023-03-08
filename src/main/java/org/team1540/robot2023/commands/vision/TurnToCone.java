package org.team1540.robot2023.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.Limelight;

public class TurnToCone extends CommandBase{
    Limelight limelight; 
    private PIDController pidController;
    private  Drivetrain drivetrain;
    CommandXboxController controller; 

    private final PIDController pid = new PIDController(1, 0, 0);

    public TurnToCone(Limelight limelight, Drivetrain drivetrain, CommandXboxController controller){
        this.limelight = limelight; 
        this.drivetrain = drivetrain; 
        this.controller = controller; 
        addRequirements(drivetrain);
    }
    private double getHorizontalDistanceToTarget() {
        double tx = limelight.getTx(); 
        return tx;
    }
    private double getError(double distanceToTarget) {
        return Math.abs(distanceToTarget) / (limelight.getHorizontalFov() / 2);
    }


    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", 0.008);
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", 0.05);
        pidController.setPID(p, i, d);
        pidController.setSetpoint(0);
    
        SmartDashboard.putBoolean("pointToTarget/turningWithLimelight", true);
        System.out.println("PTT Initialized");
    }

     /**
     * Executes turning to the target using the Limelight as the primary sensor to determine whether we have turned enough.
     *
     * angleXOffset the offset in degrees we still need to turn to reach the target
     */
    private void turnWithLimelight() {
        double angleXOffset = limelight.getTx(); 
        System.out.println(angleXOffset);
        if (Math.abs(angleXOffset) > SmartDashboard.getNumber("pointToTarget/targetDeadzoneDegrees", 5)) {

            double pidOutput = pid.calculate(angleXOffset); 
            double multiplier = angleXOffset > 0 ? 1 : -1;

            SmartDashboard.putNumber("pointToTarget/pidOutput", pidOutput);

            pidOutput = pidOutput*multiplier;
            // double valueL = multiplier * -pidOutput;
            // double valueR = multiplier * pidOutput;
            drivetrain.drive(0, 0,pidOutput, false);
        } else {
            System.out.println("Ending Turing with limelight");
        }
    }

    @Override
    public void execute() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", 0.006);
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", 0.015);
        pidController.setPID(p, i, d);
        turnWithLimelight();
    }
}
