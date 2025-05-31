package frc.robot.commands.defaultCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.PosPose2d;
import frc.robot.extras.RobotLogManager;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveToPointCmd extends Command {
    private SwerveSubsystem swerveSubsystem;
    private Pose2d targetPosition;
    private Timer timer;

    public SwerveDriveToPointCmd(SwerveSubsystem m_SwerveSubsystem, PosPose2d targetPosition) {
        this.swerveSubsystem = m_SwerveSubsystem;
        this.targetPosition = targetPosition.toFieldPose2d();

        timer = new Timer();

        addRequirements(m_SwerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.initializeDriveToPointAndRotate(targetPosition);
        RobotLogManager.debug(
                "robot pose: "
                        + swerveSubsystem.getPose().getX()
                        + ", "
                        + swerveSubsystem.getPose().getY());
        RobotLogManager.debug(
                "target pose: " + targetPosition.getX() + ", " + targetPosition.getY());
        timer.restart();
    }

    @Override
    public void execute() {
        swerveSubsystem.executeDriveToPointAndRotate(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // use this function if you overide the command to finish it

        if ((swerveSubsystem.isExactlyInPosition(targetPosition)
                        || swerveSubsystem.isNearlyInPosition(targetPosition)
                                && swerveSubsystem.isAtAngle(targetPosition.getRotation()))
                || timer.hasElapsed(5)) {
            // System.out.println("Finished Driving");
            return true;
        }

        if (timer.hasElapsed(3.5)) {
            return true;
        }

        return false;
    }
}
