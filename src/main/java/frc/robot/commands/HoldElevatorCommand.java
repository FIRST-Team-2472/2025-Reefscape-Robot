package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.RobotStatus;
import frc.robot.subsystems.ElevatorSubsystem;

public class HoldElevatorCommand extends Command {

    ElevatorSubsystem elevatorSubsystem;
    MotorPowerController motorPowerController;

    double targetHeight;


    public HoldElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.elevatorSubsystem = elevatorSubsystem;
        motorPowerController = new MotorPowerController(0.07, 0.01, 0.2, 1, 1, RobotStatus.kElevatorHeight, 5);


        addRequirements(elevatorSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetHeight = RobotStatus.kElevatorHeight;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double power = motorPowerController.calculate(targetHeight, RobotStatus.kElevatorHeight);
        elevatorSubsystem.runElevatorMotors(-power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.runElevatorMotors(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
