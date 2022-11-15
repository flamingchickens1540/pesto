package org.team1540.delphi.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.delphi.Constants;

public class Elevator extends SubsystemBase {
    private final VictorSPX leftMotor = new VictorSPX(Constants.Elevator.leftMotor);
    private final VictorSPX rightMotor = new VictorSPX(Constants.Elevator.rightMotor);

    public Elevator() {
        // TODO: Confirm inversion
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor);
    }

    public void setPercent(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }
}
