package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.SensorStatus;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class AutoElevatorCommand extends Command{
    Timer timer = new Timer();
    ElevatorSubsystem elevatorSubsystem;
    LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
    MotorPowerController motorPowerController;
    double elevatorSetHeight;
    
    public AutoElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorSetHeight) {
        this.elevatorSetHeight = elevatorSetHeight;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
        // these are guessed numbers, they need to be tuned
    }

    @Override
    public void initialize() {
        timer.reset();
        ledSubsystem.runningAutonomistCommand(true);
        elevatorSubsystem.setSetHeight(elevatorSetHeight);
    }

    @Override
    public void execute() {// nothing to do since the subsystem handles the driving of it
        
    }

    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSetHeight - SensorStatus.kElevatorHeight) < .5 || timer.hasElapsed(2);
    }
}