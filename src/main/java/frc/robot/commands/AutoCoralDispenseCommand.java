package frc.robot.commands;

import frc.robot.MotorPowerController;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCoralDispenseCommand extends Command{
    CoralDispenserSubsystem coralDispenserSubsystem;
    LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
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
        ledSubsystem.runningAutonomistCommand(true);
    }

    @Override
    public void execute() {
        coralDispenserSubsystem.runMotors(-1, 1);
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