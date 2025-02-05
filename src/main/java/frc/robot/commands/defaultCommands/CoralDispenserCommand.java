package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralDispenserCommand extends Command{
    CoralDispenserSubsystem coralDispenserSubsytem;
    Supplier<Double> xboxControllerRightTrigger;

    public CoralDispenserCommand(CoralDispenserSubsystem coralDispenserSubsytem, Supplier<Double> xboxControllerRightTrigger){
        this.coralDispenserSubsytem = coralDispenserSubsytem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        addRequirements(coralDispenserSubsytem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(xboxControllerRightTrigger.get() > 0.5)
            coralDispenserSubsytem.runMotors(.1, -.1);//subject to change
        else 
            coralDispenserSubsytem.runMotors(0, 0);
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

