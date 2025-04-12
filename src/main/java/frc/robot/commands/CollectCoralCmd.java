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

    }
    @Override
    public void execute() {
        coralSubsystem.runMotors(.2, -.2);
    }
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.runMotors(0, 0);
    }
    @Override
    public boolean isFinished() {
        return coralSubsystem.hascoral;
    }
}
