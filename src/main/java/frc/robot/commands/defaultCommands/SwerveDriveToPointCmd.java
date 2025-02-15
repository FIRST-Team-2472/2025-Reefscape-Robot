package frc.robot.commands.defaultCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.extras.DrivePose2d;
import frc.robot.extras.PosPose2d;

public class SwerveDriveToPointCmd extends Command {
  private SwerveSubsystem swerveSubsystem;
  private Pose2d targetPosition;
  private Timer timer;

  public SwerveDriveToPointCmd(SwerveSubsystem m_SwerveSubsystem, DrivePose2d targetPosition) {
    this.swerveSubsystem = m_SwerveSubsystem;
    this.targetPosition = targetPosition;
    
    timer = new Timer();

    addRequirements(m_SwerveSubsystem);
  }

  public SwerveDriveToPointCmd(SwerveSubsystem m_SwerveSubsystem, PosPose2d targetPosition) {
    this.swerveSubsystem = m_SwerveSubsystem;
    this.targetPosition = targetPosition.toFieldPose2d();
    
    timer = new Timer();

    addRequirements(m_SwerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.initializeDriveToPointAndRotate(targetPosition);
    //System.out.println("robot pose" + swerveSubsystem.getPose().getX()+", "+ swerveSubsystem.getPose().getY());
    //System.out.println("target pose" + targetPosition.getX()+", "+ targetPosition.getY());
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
    // use this function if you overide the command to finsih it
    if (swerveSubsystem.isAtPoint(targetPosition.getTranslation()) && swerveSubsystem.isAtAngle(targetPosition.getRotation())){
      System.out.println("Finished Driving");
      return true;
    }
    return false;
  }
}
