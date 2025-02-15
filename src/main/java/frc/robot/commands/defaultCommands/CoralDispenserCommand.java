package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.SensorStatus;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralDispenserCommand extends Command{
    CoralDispenserSubsystem coralDispenserSubsystem;
    Supplier<Double> xboxControllerRightTrigger, xboxControllerLeftTrigger;

    public CoralDispenserCommand(CoralDispenserSubsystem coralDispenserSubsystem, Supplier<Double> xboxControllerRightTrigger, Supplier<Double> xboxControllerLeftTrigger){
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        this.xboxControllerLeftTrigger = xboxControllerLeftTrigger;
        addRequirements(coralDispenserSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(xboxControllerRightTrigger.get() > 0.5)
            if(SensorStatus.kElevatorHeight > ElevatorConstants.kElevatorL1Height - 1 && SensorStatus.kElevatorHeight < ElevatorConstants.kElevatorL1Height + 1)
                coralDispenserSubsystem.runMotors(.9, -.3);
            else 
                coralDispenserSubsystem.runMotors(.8, -.8);//subject to change
        else if(xboxControllerLeftTrigger.get() > 0.5){
                coralDispenserSubsystem.runMotors(-.3, .3);
        }
        else
            coralDispenserSubsystem.runMotors(0, 0);
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

