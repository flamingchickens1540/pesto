package org.team1540.robot2023.commands.grabber;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2023.Constants.GrabberConstants;

public class Intake extends SubsystemBase {
    private final TalonFX motor1 = new TalonFX(GrabberConstants.INTAKE_1_ID);
    private final TalonFX motor2 = new TalonFX(GrabberConstants.INTAKE_2_ID);
    private boolean isRunning;

    public Intake() {
        motor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0));
        motor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0));

        motor1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
        motor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));

        motor1.setNeutralMode(NeutralMode.Coast);
        motor2.setNeutralMode(NeutralMode.Coast);

        motor1.setInverted(false);
        motor2.setInverted(true);
        motor2.follow(motor1);

        isRunning = false;
    }

    public void runIntake() {
        isRunning = true;
    }

    public void stop() {
        isRunning = false;
        motor1.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        if (isRunning) {
            if (motor1.getStatorCurrent() >= GrabberConstants.INTAKE_CURRENT_THRESH) {
                motor1.set(ControlMode.PercentOutput, 0.2);
            } else motor1.set(ControlMode.PercentOutput, 0.7);
        }
    }
}
