package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorStatus;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorCommand extends Command{
    Timer timer = new Timer();
    ElevatorSubsystem elevatorSubsystem;
    MotorPowerController motorPowerController;
    double elevatorSetHeight;
    
    public AutoElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorSetHeight) {
        this.elevatorSetHeight = elevatorSetHeight;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
        // these are guessed numbers, they need to be tuned
        motorPowerController = new MotorPowerController(0.0083, 0.1, 0.5, 2, 1, SensorStatus.kElevatorHeight, 10);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        elevatorSubsystem.runElevatorMotors(motorPowerController.calculate(elevatorSetHeight, SensorStatus.kElevatorHeight));
    }

    public void end(boolean interrupted) {
        elevatorSubsystem.runElevatorMotors(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSetHeight - SensorStatus.kElevatorHeight) < .5 || timer.hasElapsed(2);
    }
}