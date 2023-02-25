package org.team1540.robot2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.lib.RevBlinkin;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.ManualArm;
import org.team1540.robot2023.commands.arm.RetractAndPivotCommand;
import org.team1540.robot2023.commands.auto.BottomGrid1PieceBalance;
import org.team1540.robot2023.commands.auto.BottomGrid1PieceTaxi;
import org.team1540.robot2023.commands.auto.GridDriveAndPivotCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.ProxiedSubstationDriveCommand;
import org.team1540.robot2023.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.BlinkinPair;
import org.team1540.robot2023.utils.ButtonPanel;
import org.team1540.robot2023.utils.PolePosition;

import java.util.concurrent.atomic.AtomicReference;

import static org.team1540.robot2023.Constants.ENABLE_PNEUMATICS;

public class RobotContainer {
    // Hardware


    RevBlinkin frontBlinken = new RevBlinkin(1, RevBlinkin.ColorPattern.WAVES_PARTY);
    RevBlinkin rearBlinken = new RevBlinkin(0, RevBlinkin.ColorPattern.WAVES_FOREST);
    BlinkinPair blinkins = new BlinkinPair(frontBlinken, rearBlinken);
    public final PneumaticHub ph = new PneumaticHub(Constants.PNEUMATIC_HUB);
    public final PowerDistribution pdh = new PowerDistribution(Constants.PDH, PowerDistribution.ModuleType.kRev);
    // Subsystems

    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm();
    WheeledGrabber wheeledGrabber = new WheeledGrabber();
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
        initAutos();
        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);


    }

   private void configureButtonBindings() {
        // Driver
        driver.a().onTrue(new InstantCommand(drivetrain::zeroGyroscope).andThen(drivetrain::resetAllToAbsolute));
        driver.leftBumper().whileTrue(new ProxiedSubstationDriveCommand(drivetrain, Constants.Auto.hpOffsetY));
        // Copilot

        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_PURPLE).onTrue(blinkins.commandSet(BlinkinPair.ColorPair.CUBE));
        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_YELLOW).onTrue(blinkins.commandSet(BlinkinPair.ColorPair.CONE));

        controlPanel.onButton(ButtonPanel.PanelButton.TOP_LEFT     ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.LEFT,   arm, Constants.Auto.armHighCone));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_CENTER   ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.CENTER, arm, Constants.Auto.armHighCube));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_RIGHT    ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.RIGHT,  arm, Constants.Auto.armHighCone));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_LEFT  ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.LEFT,   arm, Constants.Auto.armMidCone));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_CENTER).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.CENTER, arm, Constants.Auto.armMidCube));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_RIGHT ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.RIGHT,  arm, Constants.Auto.armMidCone));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_LEFT  ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.LEFT,   arm, Constants.Auto.armDown));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_CENTER).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.CENTER, arm, Constants.Auto.armDown));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_RIGHT ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.RIGHT,  arm, Constants.Auto.armDown));

        // coop:button(A, Run Intake [HOLD],copilot)
        copilot.a().toggleOnTrue(new GrabberIntakeCommand(wheeledGrabber));
        // coop:button(B,Run Outtake [HOLD],copilot)
        copilot.b().whileTrue(new GrabberOuttakeCommand(wheeledGrabber));



        //coop:button(RBumper, Floor pickup [HOLD], copilot)
        copilot.rightBumper().whileTrue(new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(-115)));
        //coop:button(LBumper, Set arm upright [HOLD], copilot)
        copilot.leftBumper().whileTrue(new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(0)));
        //coop:button(Y, reset arm angle, copilot)
        copilot.y().onTrue(new InstantCommand(() -> arm.resetAngle()));



        AtomicReference<NeutralMode> currentMode = new AtomicReference<>(NeutralMode.Brake);
        new Trigger(RobotController::getUserButton).onTrue(new InstantCommand(()->{
            if (currentMode.get() == NeutralMode.Brake) {
                currentMode.set(NeutralMode.Coast);
            } else {
                currentMode.set(NeutralMode.Brake);
            }
            DataLogManager.log("FPGA User Button: Setting pivot falcons to NeutralMode."+currentMode);
            arm.setRotationNeutralMode(currentMode.get());
        }).ignoringDisable(true));

        // SmartDashboard Commands

    }

    public void setTeleopDefaultCommands() {
        // coop:button(LJoystick, Translate swerve,pilot)
        // coop:button(RJoystick, Rotate swerve [LEFTRIGHT],pilot)
        // coop:button(X, Slow drive [PRESS])
        // coop:button(B, Fast Drive [PRESS])
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver.getHID()));
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

    private void initAutos() {
        AutoManager manager = AutoManager.getInstance();
        manager.addAuto(new BottomGrid1PieceTaxi(drivetrain, arm));
        manager.addAuto(new BottomGrid1PieceBalance(drivetrain, arm));
    }

    public Command getAutonomousCommand() {
        return AutoManager.getInstance().getSelected();
    }

}
