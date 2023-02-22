package org.team1540.robot2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.lib.RevBlinkin;
import org.team1540.robot2023.commands.GridDriveAndPivotCommand;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.ManualArm;
import org.team1540.robot2023.commands.arm.RetractAndPivotCommand;
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
        driver.leftBumper().whileTrue(new ProxiedSubstationDriveCommand(drivetrain, Constants.Auto.hpOffsetY));
        // Copilot
        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_PURPLE).onTrue(blinkins.commandSet(BlinkinPair.ColorPair.CUBE));
        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_YELLOW).onTrue(blinkins.commandSet(BlinkinPair.ColorPair.CONE));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_LEFT     ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.LEFT,   arm, Constants.Auto.angleHighCone));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_CENTER   ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.CENTER, arm, Constants.Auto.angleHighCube));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_RIGHT    ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.RIGHT,  arm, Constants.Auto.angleHighCone));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_LEFT  ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.LEFT,   arm, Constants.Auto.angleMidCone));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_CENTER).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.CENTER, arm, Constants.Auto.angleMidCube));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_RIGHT ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.RIGHT,  arm, Constants.Auto.angleMidCone));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_LEFT  ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.LEFT,   arm, Constants.Auto.angleDown));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_CENTER).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.CENTER, arm, Constants.Auto.angleDown));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_RIGHT ).whileTrue(new GridDriveAndPivotCommand(drivetrain, PolePosition.RIGHT,  arm, Constants.Auto.angleDown));
        copilot.a().toggleOnTrue(new GrabberIntakeCommand(wheeledGrabber));
        copilot.b().whileTrue(new GrabberOuttakeCommand(wheeledGrabber));
        //Pneumatic Control
//        copilot.a().onTrue(new InstantCommand(() -> pneumaticClaw.toggle()));
//        copilot.a().onTrue(new InstantCommand(() -> pneumaticClaw.set(true)));
//        copilot.b().onTrue(new InstantCommand(() -> pneumaticClaw.set(false)));
        //Ground pickup and hybrid node deposit position
        copilot.x().whileTrue(new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(-115)));
        //Upright position for driving
        copilot.leftBumper().whileTrue(new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(0)));
        //Substation pickup, extension is needed
        copilot.rightBumper().whileTrue(new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(-58)));

        copilot.y().onTrue(new InstantCommand(() -> arm.resetAngle())); // TODO: 2/18/2023 change binding


        AtomicReference<NeutralMode> currentMode = new AtomicReference<>(NeutralMode.Brake);
        new Trigger(RobotController::getUserButton).onTrue(new InstantCommand(()->{
            if (currentMode.get() == NeutralMode.Brake) {
                currentMode.set(NeutralMode.Coast);
            } else {
                currentMode.set(NeutralMode.Brake);
            }
            System.out.println(currentMode);
            arm.setRotationNeutralMode(currentMode.get());
        }).ignoringDisable(true));
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
        return new SequentialCommandGroup(
                new RetractAndPivotCommand(arm, Constants.ArmConstants.Setpoints.midCube).withTimeout(3),
                new GrabberOuttakeCommand(wheeledGrabber).withTimeout(2),
                new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(0))

//                new PrintCommand("DRIVING"),
//                new PathPlannerDriveCommand(drivetrain),
//                new PrintCommand("BALANCING"),
//                new AutoBalanceCommand(drivetrain, true)
        );
    }

}
