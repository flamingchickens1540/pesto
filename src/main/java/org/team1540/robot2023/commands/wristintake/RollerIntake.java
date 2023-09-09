package org.team1540.robot2023.commands.wristintake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2023.utils.AverageFilter;

import static org.team1540.robot2023.Constants.RollerIntakeConstants.*;

public class RollerIntake extends SubsystemBase {
    private final TalonFX motor = new TalonFX(ROLLER_INTAKE_ID);
    private final AverageFilter averageFilter = new AverageFilter(5);

    public RollerIntake() {
        motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, INTAKE_IDLE_CURRENT, INTAKE_IDLE_CURRENT, 0)
        );
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(false); // TODO: 8/29/2023 figure out inversion
    }

    public void setSpeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        setSpeed(0);
    }

    public boolean hasGamePiece() {
        return averageFilter.getAverage() < VELOCITY_THRESH;
    }

    @Override
    public void periodic() {
        averageFilter.add(motor.getSelectedSensorVelocity());
    }
}
