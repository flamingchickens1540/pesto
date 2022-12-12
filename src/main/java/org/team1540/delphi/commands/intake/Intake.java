package org.team1540.delphi.commands.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.delphi.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.motor, CANSparkMaxLowLevel.MotorType.kBrushless);
    public Intake() {
        intakeMotor.setInverted(false);
    }

    /**
     * Sets the speed of the motors
     * @param percent [-1,1] How fast to spin the motor
     */
    public void setPercent(double percent) {
        this.intakeMotor.set(percent);
    }
}
