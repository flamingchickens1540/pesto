package org.team1540.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import org.team1540.robot2023.utils.swerve.ModuleCorner;
import org.team1540.robot2023.utils.swerve.SwerveCANDevice;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;


    public SwerveModuleConstants(int moduleID, ModuleCorner corner) {
        this.angleOffset = corner.getRotation2dOffset(moduleID);

        this.driveMotorID = SwerveCANDevice.getDrivingMotorID(moduleID);
        this.angleMotorID = SwerveCANDevice.getTurningMotorID(moduleID);
        this.cancoderID = SwerveCANDevice.getCancoderID(moduleID);

        DataLogManager.log("Initializing Module "+moduleID+" [Offset: "+angleOffset.getDegrees()+"]");
    }
}
