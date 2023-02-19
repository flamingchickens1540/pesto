package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2023.Constants.GrabberConstants;

public class PneumaticClaw extends SubsystemBase {
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, GrabberConstants.CLAW_SOLENOID_CHANNEL);

    public PneumaticClaw() {
    }

    public void set(boolean open) {
        solenoid.set(open);
    }
    public void toggle(){
        solenoid.set(!solenoid.get());
    }
}
