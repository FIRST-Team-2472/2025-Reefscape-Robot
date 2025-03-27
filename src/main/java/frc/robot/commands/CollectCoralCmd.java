package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SensorStatus;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CollectCoralCmd extends Command{

    Timer timer;

    CoralDispenserSubsystem coralDispenserSubsystem;
    int framesCoralSeen = 0;
    public CollectCoralCmd(CoralDispenserSubsystem coralDispenserSubsystem){
        addRequirements(coralDispenserSubsystem);
        this.coralDispenserSubsystem = coralDispenserSubsystem;


    }
    @Override
    public void initialize() {
        framesCoralSeen = 0;
    }
    @Override
    public void execute() {
        if(SensorStatus.seeCoral)
            framesCoralSeen ++;
        coralDispenserSubsystem.runMotors(.3, -.3);
    }
    @Override
    public void end(boolean interrupted) {
        coralDispenserSubsystem.runMotors(0,0);

    }
    @Override
    public boolean isFinished() {
        return coralDispenserSubsystem.hascoral && framesCoralSeen > 20;
    }
}
