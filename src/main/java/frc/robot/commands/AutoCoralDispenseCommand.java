package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.SensorStatus;
import frc.robot.MotorPowerController;
import frc.robot.subsystems.CoralDispenserSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCoralDispenseCommand extends Command{
    CoralDispenserSubsystem coralDispenserSubsystem;
    MotorPowerController motorPowerController;
    Timer timoutTimer;

    public AutoCoralDispenseCommand(CoralDispenserSubsystem coralDispenserSubsystem){
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        this.timoutTimer = new Timer();
        addRequirements(coralDispenserSubsystem);
    }
    
    @Override
    public void initialize() {
        timoutTimer.reset();
    }

    @Override
    public void execute() {
        if(SensorStatus.kElevatorHeight > ElevatorConstants.kElevatorL1Height - 1 && SensorStatus.kElevatorHeight < ElevatorConstants.kElevatorL1Height + 1)
                coralDispenserSubsystem.runMotors(.9, -.3);
            else 
                coralDispenserSubsystem.runMotors(.8, -.8);
    }

    @Override
    public void end(boolean interrupted) {
        coralDispenserSubsystem.runMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        return timoutTimer.hasElapsed(1);
    }
}