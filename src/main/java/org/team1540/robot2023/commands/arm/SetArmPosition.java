package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.RollingAverage;
import org.team1540.robot2023.Constants.ArmConstants;

public class SetArmPosition extends CommandBase {
    private final Arm arm;
    private final ArmState goalState;

    private double rotThresh = 10;
    private double extThresh = 10;
    private final RollingAverage rotAvg = new RollingAverage(10);
    private final RollingAverage extAvg = new RollingAverage(10);
    private boolean isFinished = false;

    /**
     * Initializes this command to move the arm according to the goal state:
     *  <ul>
     *      <li>
     *          If the goal state is an illegal position, such as when the arm is above its
     *          height limit or too far away, this command does nothing
     *      </li>
     *      <li>
     *          If the goal state is within {@value ArmConstants#MAX_POINT_DISTANCE}, point
     *          the arm in the general direction but do not extend to it
     *      </li>
     *      <li>
     *          Otherwise, check if the goal state represents a state where the arm would be
     *          below ground, in which case extend to just before the arm hits the ground.
     *      </li>
     *      <li>
     *          If these checks pass with needing to adjust the goal state, start moving the
     *          arm to the specified state
     *      </li>
     *  </ul>
     * @param arm the arm this command runs on
     * @param goalState the goal position of the arm
     */
    public SetArmPosition(Arm arm, ArmState goalState) {
        this.arm = arm;
        Rotation2d angle = Rotation2d.fromRadians(
                //Math.max(goalState.getRotation2d().getRadians(), ArmConstants.PIVOT_MIN_ANGLE)
                goalState.getRotation2d().getRadians() < 0 ?
                        Math.max(goalState.getRotation2d().getRadians(), ArmConstants.PIVOT_MIN_ANGLE) :
                        Math.min(goalState.getRotation2d().getRadians(), -ArmConstants.PIVOT_MIN_ANGLE)
        );
        double extension;

        if (Math.abs(goalState.getX()) > ArmConstants.MAX_POINT_DISTANCE || goalState.getY() > ArmConstants.MAX_LEGAL_HEIGHT) {
            isFinished = true;
            this.goalState = arm.getArmState();
            System.err.println("Illegal arm state given to SetArmPosition");
            return;
        } else if (Math.abs(goalState.getX()) > ArmConstants.MAX_LEGAL_DISTANCE) {
            extension = ArmConstants.ARM_BASE_LENGTH + 1;
        } else if (goalState.getY() <= -ArmConstants.PIVOT_HEIGHT){
            extension = Math.max(ArmConstants.ARM_BASE_LENGTH + 1, arm.getMaxExtension(angle));
        } else extension = goalState.getExtension();
        this.goalState = ArmState.fromRotationExtension(angle, extension);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setRotation(goalState.getRotation2d());
        arm.setExtensionSetPoint(goalState.getExtension());
        arm.setManualControl(false);
    }

    @Override
    public void execute() {
        rotAvg.add(arm.getArmState().getRotation2d().getRadians() - goalState.getRotation2d().getRadians());
        extAvg.add(arm.getArmState().getExtension() - goalState.getExtension());
    }

    @Override
    public boolean isFinished() {
        return (extAvg.getAverageAbs() < extThresh && rotAvg.getAverageAbs() < rotThresh) || isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}
