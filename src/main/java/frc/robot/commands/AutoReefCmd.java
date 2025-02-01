package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CoralDispenserSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoReefCmd extends Command{
    CoralDispenserSubsytem coralDispenserSubsytem;
    ElevatorSubsystem elevatorSubsystem;
    SwerveSubsystem swerveSubsystem;
    PoseEstimate robotPosition; 
    
    public AutoReefCmd(CoralDispenserSubsytem coralDispenserSubsytem, ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem) {
        this.coralDispenserSubsytem = coralDispenserSubsytem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(coralDispenserSubsytem, elevatorSubsystem, swerveSubsystem);
    }

    @Override
    public void initialize() {
        robotPosition = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limlight-objFeed");

        

    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
