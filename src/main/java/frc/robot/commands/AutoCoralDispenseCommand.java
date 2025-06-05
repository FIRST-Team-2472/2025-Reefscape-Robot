package frc.robot.commands;

import frc.robot.MotorPowerController;
import frc.robot.RobotStatus;
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
        timoutTimer.restart();
    }

    @Override
    public void execute() {
        if(RobotStatus.kElevatorHeight > 8 && RobotStatus.kElevatorHeight < 10) {
            coralDispenserSubsystem.runMotors(.9, -.3);
        } else {
            coralDispenserSubsystem.runMotors(1, -1);
        }

        coralDispenserSubsystem.hasCoral = false;
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("Dispense Coral Command ENDED!");
        coralDispenserSubsystem.runMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        if (timoutTimer.hasElapsed(.6)){
            //System.out.println("isFinished Dispensing Coral");
            return true;
        }
        return false;
    }
}