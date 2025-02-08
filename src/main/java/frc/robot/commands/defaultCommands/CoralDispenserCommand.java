package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SensorStatus;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralDispenserCommand extends Command{
    CoralDispenserSubsystem coralDispenserSubsytem;
    Supplier<Double> xboxControllerRightTrigger, xboxControllerLeftTrigger;

    public CoralDispenserCommand(CoralDispenserSubsystem coralDispenserSubsytem, Supplier<Double> xboxControllerRightTrigger, Supplier<Double> xboxControllerLeftTrigger){
        this.coralDispenserSubsytem = coralDispenserSubsytem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        this.xboxControllerLeftTrigger = xboxControllerLeftTrigger;
        addRequirements(coralDispenserSubsytem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(xboxControllerRightTrigger.get() > 0.5)
            if(SensorStatus.kElevatorHeight > 8 && SensorStatus.kElevatorHeight < 10)
                coralDispenserSubsytem.runMotors(.9, -.3);
            else 
                coralDispenserSubsytem.runMotors(.8, -.8);//subject to change
        else if(xboxControllerLeftTrigger.get() > 0.5){
                coralDispenserSubsytem.runMotors(-.3, .3);
        }
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

