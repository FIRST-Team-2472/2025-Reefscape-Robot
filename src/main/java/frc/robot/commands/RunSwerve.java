package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RunSwerve extends Command{
    SwerveSubsystem swerveSubsystem;
    
    public RunSwerve(SwerveSubsystem swerveSubsystem, double xPower, double yPower) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        // these are guessed numbers, they need to be tuned
    }

    @Override
    public void initialize(){}

    @Override
    public void execute() {
        if (SwerveSubsystem.isOnRed())
            swerveSubsystem.runModulesFieldRelative(0.1, 0, 0);
    }

    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}