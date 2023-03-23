package org.team1540.robot2023.commands.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2023.Constants.GrabberConstants;
import org.team1540.robot2023.utils.AverageFilter;

public class WheeledGrabber extends SubsystemBase {
    private final CANSparkMax motor1 = new CANSparkMax(GrabberConstants.INTAKE_1_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motor2 = new CANSparkMax(GrabberConstants.INTAKE_2_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder encoder1 = motor1.getEncoder();
    private final RelativeEncoder encoder2 = motor2.getEncoder();
    private final AverageFilter averageFilter = new AverageFilter(5);

    public WheeledGrabber() {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();

        setCurrentLimit(20);

        motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        motor2.follow(motor1, true);
        motor1.setInverted(true);

        motor1.set(0);



    }
    public boolean hasGamePiece() {
        return Math.abs(averageFilter.getAverage()) <200;

    }
    public void stop() {
        motor1.set(0);
    }

    public void setCurrentLimit(int limit){
        motor1.setSmartCurrentLimit(limit);
        motor2.setSmartCurrentLimit(limit);
        SmartDashboard.putNumber("intake/currentLimit", limit);
    }

    public void setSpeed(double speed){
        motor1.set(speed);
    }

    public double getCurrent(){
        return motor1.getOutputCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/current", motor1.getOutputCurrent());
        SmartDashboard.putNumber("intake/velocity", averageFilter.getAverage());
        averageFilter.add((encoder1.getVelocity() + encoder2.getVelocity())/2);
    }


}
