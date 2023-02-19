package org.team1540.robot2023;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.commands.arm.*;
import org.team1540.robot2023.commands.drivetrain.*;
import org.team1540.lib.RevBlinkin;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;
import org.team1540.robot2023.commands.drivetrain.ProxiedGridDriveCommand;
import org.team1540.robot2023.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.BlinkinPair;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.utils.ButtonPanel;
import org.team1540.robot2023.utils.PolePosition;

import static org.team1540.robot2023.Constants.ENABLE_PNEUMATICS;

public class RobotContainer {
    // Hardware
    RevBlinkin frontBlinken = new RevBlinkin(1, RevBlinkin.ColorPattern.WAVES_FOREST);
    RevBlinkin rearBlinken = new RevBlinkin(0, RevBlinkin.ColorPattern.WAVES_FOREST);
    BlinkinPair blinkins = new BlinkinPair(frontBlinken, rearBlinken);
    public final PneumaticHub ph = new PneumaticHub(Constants.PNEUMATIC_HUB);
    public final PowerDistribution pdh = new PowerDistribution(Constants.PDH, PowerDistribution.ModuleType.kRev);
    // Subsystems

    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm();
    WheeledGrabber wheeledGrabber = new WheeledGrabber();
//    PneumaticClaw pneumaticClaw = new PneumaticClaw();
    // Controllers
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController copilot = new CommandXboxController(1);
    ButtonPanel controlPanel = new ButtonPanel(2);
    // Commands

    public RobotContainer() {
        pdh.clearStickyFaults();
        ph.clearStickyFaults();
        if (ENABLE_PNEUMATICS) {
            ph.enableCompressorDigital();
        } else {
            ph.disableCompressor();
        }


        initSmartDashboard();
        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureButtonBindings() {
        // Driver
        driver.a().onTrue(new InstantCommand(drivetrain::zeroGyroscope).andThen(drivetrain::resetAllToAbsolute));
        // Copilot

        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_PURPLE).onTrue(blinkins.commandSet(BlinkinPair.ColorPair.CUBE));
        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_YELLOW).onTrue(blinkins.commandSet(BlinkinPair.ColorPair.CONE));

        controlPanel.onButton(ButtonPanel.PanelButton.TOP_LEFT     ).whileTrue(new ProxiedGridDriveCommand(drivetrain, 6, PolePosition.LEFT));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_CENTER   ).whileTrue(new ProxiedGridDriveCommand(drivetrain, 6, PolePosition.CENTER));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_RIGHT    ).whileTrue(new ProxiedGridDriveCommand(drivetrain, 6, PolePosition.RIGHT));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_LEFT  ).whileTrue(new ProxiedGridDriveCommand(drivetrain, PolePosition.LEFT));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_CENTER).whileTrue(new ProxiedGridDriveCommand(drivetrain, PolePosition.CENTER));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_RIGHT ).whileTrue(new ProxiedGridDriveCommand(drivetrain, PolePosition.RIGHT));


        // coop:button(A,Run Intake [PRESS],copilot)
        copilot.a().toggleOnTrue(new GrabberIntakeCommand(wheeledGrabber));
        // coop:button(B,Run Outtake [HOLD],copilot)
        copilot.b().whileTrue(new GrabberOuttakeCommand(wheeledGrabber));



        //Pneumatic Control
//        copilot.a().onTrue(new InstantCommand(() -> pneumaticClaw.toggle()));
//        copilot.a().onTrue(new InstantCommand(() -> pneumaticClaw.set(true)));
//        copilot.b().onTrue(new InstantCommand(() -> pneumaticClaw.set(false)));


        copilot.x().whileTrue(new ExtensionCommand(arm, 30));
        copilot.y().whileTrue(new ExtensionCommand(arm, 40));
        // coop:button(RBumper, Move to arm straight forward [HOLD],copilot)
        copilot.rightBumper().whileTrue(new PivotToSetpoint(arm, Rotation2d.fromDegrees(90)));
        // coop:button(LBumper, Move to arm straight up [HOLD],copilot)
        copilot.leftBumper().whileTrue(new PivotToSetpoint(arm, Rotation2d.fromDegrees(0)));
        copilot.leftStick().onTrue(new InstantCommand(() -> arm.resetAngle())); // TODO: 2/18/2023 change binding

        // SmartDashboard Commands

    }

    public void setTeleopDefaultCommands() {
        // coop:button(LJoystick, Translate swerve,pilot)
        // coop:button(RJoystick, Rotate swerve [LEFTRIGHT],pilot)
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        // coop:button(LJoystick, Adjust arm angle [UPDOWN],copilot)
        // coop:button(LTrigger, Retract telescope [HOLD],copilot)
        // coop:button(RTrigger, Extend telescope [HOLD],copilot)
        arm.setDefaultCommand(new ManualArm(arm, copilot));
    }

    private void initSmartDashboard() {
        SmartDashboard.putNumber("arm/targetX", 22);
        SmartDashboard.putNumber("arm/targetY", 0);
        SmartDashboard.putNumber("arm/targetAngle", 0);
        SmartDashboard.putNumber("arm/targetExtension", 22);
    }

    public CommandBase getAutonomousCommand() {
        return new PathPlannerDriveCommand(drivetrain);
    }

}
