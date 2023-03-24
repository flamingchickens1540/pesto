package org.team1540.robot2023;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class LogManager {

    private PowerDistribution pdh;
    private final StringLogEntry commandLog = new StringLogEntry(DataLogManager.getLog(), "CUSTOM:commands");
    private final DoubleLogEntry pdhVoltageLog = new DoubleLogEntry(DataLogManager.getLog(), "CUSTOM:power/voltage");
    private final DoubleLogEntry pdhCurrentLog = new DoubleLogEntry(DataLogManager.getLog(), "CUSTOM:power/totalCurrent");
    private final DoubleLogEntry pdhPowerLog = new DoubleLogEntry(DataLogManager.getLog(), "CUSTOM:power/joules");
    private final int[] pdhChannelLogIDs;

    public LogManager(PowerDistribution pdh) {
        this.pdh = pdh;
        pdhChannelLogIDs = new int[pdh.getNumChannels()];
        for (int i = 0; i<pdh.getNumChannels(); i++) {
            pdhChannelLogIDs[i] = DataLogManager.getLog().start("CUSTOM:power/current/chan"+i, "double");
        }
        CommandScheduler.getInstance().onCommandInitialize((command -> commandLog.append("Initializing: "+command.getName())));
        CommandScheduler.getInstance().onCommandFinish((command -> commandLog.append("Ending: "+command.getName())));
        CommandScheduler.getInstance().onCommandInterrupt((command -> commandLog.append("Interrupting: "+command.getName())));
    }

    public void execute() {
        pdhVoltageLog.append(pdh.getVoltage());
        pdhCurrentLog.append(pdh.getTotalCurrent());
        pdhPowerLog.append(pdh.getTotalPower());

        for (int i = 0; i<pdh.getNumChannels();i++) {
            DataLogManager.getLog().appendDouble(pdhChannelLogIDs[i], pdh.getCurrent(i), RobotController.getFPGATime());
        }
    }
}
