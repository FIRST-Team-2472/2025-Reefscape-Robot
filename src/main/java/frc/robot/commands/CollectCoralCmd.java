package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CollectCoralCmd extends Command{

    Timer timer;

    CoralDispenserSubsystem coralDispenserSubsystem;
    public CollectCoralCmd(CoralDispenserSubsystem coralDispenserSubsystem){
        addRequirements(coralDispenserSubsystem);
        this.coralDispenserSubsystem = coralDispenserSubsystem;

        timer = new Timer();

    }
    @Override
    public void initialize() {
        timer.restart();
    }
    @Override
    public void execute() {
        if (coralDispenserSubsystem.seecoral == true){
            coralDispenserSubsystem.runMotors(.5, -.5);
        }
    }
    @Override
    public void end(boolean interrupted) {
        if (timer.hasElapsed(1) && coralDispenserSubsystem.seecoral == false){
            coralDispenserSubsystem.runMotors(0,0);
        } else {
            execute();
        }
    }
    @Override
    public boolean isFinished() {
        return coralDispenserSubsystem.hascoral;
    }
}
