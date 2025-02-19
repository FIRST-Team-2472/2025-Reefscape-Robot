package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CollectCoralCmd extends Command{
    CoralDispenserSubsystem coralSubsystem;
    public CollectCoralCmd(CoralDispenserSubsystem coralSubsystem){
        addRequirements(coralSubsystem);
        this.coralSubsystem = coralSubsystem;

    }
    @Override
    public void initialize() {
        //coralSubsystem.collectCoral();
    }
    @Override
    public void execute() {
    }
    @Override
    public void end(boolean interrupted) {
        //coralSubsystem.stopCollectingCoral();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
