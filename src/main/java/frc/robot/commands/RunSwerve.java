package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RunSwerve extends Command{
    SwerveSubsystem swerveSubsystem;

    double xPower, yPower;
    
    public RunSwerve(SwerveSubsystem swerveSubsystem, double xPower, double yPower) {
        this.swerveSubsystem = swerveSubsystem;

        this.xPower = xPower;
        this.yPower = yPower;

        addRequirements(swerveSubsystem);
        // these are guessed numbers, they need to be tuned
    }

    @Override
    public void initialize(){}

    @Override
    public void execute() {
        if (SwerveSubsystem.isOnRed())
            swerveSubsystem.runModulesFieldRelative(-xPower, -yPower, 0);

        else
            swerveSubsystem.runModulesFieldRelative(xPower, yPower, 0);
    }

    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}