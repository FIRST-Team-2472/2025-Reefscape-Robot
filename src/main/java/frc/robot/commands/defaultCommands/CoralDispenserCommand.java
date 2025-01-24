package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsytem;

public class CoralDispenserCommand extends Command{
    CoralDispenserSubsytem coralDispenserSubsytem;
    Supplier<Boolean> xboxControllerRightTrigger;

    public CoralDispenserCommand(CoralDispenserSubsytem coralDispenserSubsytem, Supplier<Boolean> xboxControllerRightTrigger){
        this.coralDispenserSubsytem = coralDispenserSubsytem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        addRequirements(coralDispenserSubsytem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(xboxControllerRightTrigger.get())
            coralDispenserSubsytem.runMotors(1, -1);
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
