// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1540.robot2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2023.utils.BlinkinManager;

import static org.team1540.robot2023.Globals.aprilTagLayout;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;
    private RobotContainer robotContainer;

    private Command autonomousCommand;
    private boolean hasRunAuto;
    private boolean hasEnabled;
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        ctreConfigs = new CTREConfigs();
        DataLogManager.start("/media/sda1");
        DriverStation.startDataLog(DataLogManager.getLog());
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        this.robotContainer = new RobotContainer();

        // ---------------
        // TODO: ALWAYS REMOVE BEFORE COMMITTING
        // TODO: BAD THINGS HAPPEN IF IT GETS LEFT FOR A COMP
        // TODO: DO NOT APPROVE PRS WITH THE BELOW LINE LEFT IN

//        PathPlannerServer.startServer(5811);
        // ---------------

//        addPeriodic(robotContainer.logManager::execute, 0.25, 0.005);
        // Zero swerve modules 4 seconds after init
        new WaitCommand(5).andThen(() -> {
            robotContainer.drivetrain.resetAllToAbsolute();
            robotContainer.drivetrain.setNeutralMode(NeutralMode.Coast);
        }, robotContainer.drivetrain).ignoringDisable(true).withName("ResetModuleCommand").schedule();

        // OK so this one is really stupid and really shouldn't have to be here, but it does, just deal with it.
        aprilTagLayout.getTagPose(-1);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        LimelightManager.getInstance().periodic();
        CommandScheduler.getInstance().run();

        AutoManager.getInstance().updateSelected();

    }

    public void enabledInit() {
        robotContainer.arm.setRotationNeutralMode(NeutralMode.Brake);
        robotContainer.arm.setExtensionNeutralMode(CANSparkMax.IdleMode.kBrake);
        robotContainer.drivetrain.setNeutralMode(NeutralMode.Brake);
        hasEnabled = true;
    }

    public void enabledPeriodic() {

    }
    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        new WaitCommand(5)
                .andThen(() -> robotContainer.drivetrain.setNeutralMode(NeutralMode.Coast))
                .ignoringDisable(true)
                .schedule();
    }

    @Override
    public void disabledPeriodic() {
        if (!hasEnabled) {
            robotContainer.drivetrain.updateWithApriltags();
        }
    }

    @Override
    public void autonomousInit() {
        enabledInit();
        robotContainer.blinkins.set(BlinkinManager.ColorPair.AUTO);

        robotContainer.setAutoDefaultCommands();
        autonomousCommand = robotContainer.getAutonomousCommand();
        robotContainer.drivetrain.updateWithApriltags();
        hasRunAuto = true;

        robotContainer.drivetrain.zeroFieldOrientation();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
            robotContainer.arm.resetToGyro();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        enabledInit();
        robotContainer.blinkins.set(BlinkinManager.ColorPair.TELEOP);

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        if (!hasRunAuto) {
            robotContainer.drivetrain.zeroFieldOrientation();
            robotContainer.arm.resetToGyro();
        }
        robotContainer.setTeleopDefaultCommands();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        hasEnabled = true;
        System.out.println("Test enabled");
        LiveWindow.setEnabled(false);
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        // Post PDH data during test mode
        SmartDashboard.putData(robotContainer.pdh);
    }


}
