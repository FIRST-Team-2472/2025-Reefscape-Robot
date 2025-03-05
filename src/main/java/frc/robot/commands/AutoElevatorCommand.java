package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.SensorStatus;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorCommand extends Command {
	Timer timer = new Timer();
	ElevatorSubsystem elevatorSubsystem;
	double elevatorSetHeight;
	MotorPowerController motorPowerController;

	public AutoElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorSetHeight) {
		this.elevatorSetHeight = elevatorSetHeight;
		this.elevatorSubsystem = elevatorSubsystem;
		motorPowerController = new MotorPowerController(0.07, 0.05, 0.2, 1, 1, SensorStatus.kElevatorHeight, 5);
		addRequirements(elevatorSubsystem);
		// these are guessed numbers, they need to be tuned
	}

	@Override
	public void initialize() {

		timer.reset(); // this is broken
	}

	@Override
	public void execute() {
		// nothing to do since the subsystem handles the driving of it
		elevatorSubsystem.runElevatorMotors(Math.max(
				Math.min(-motorPowerController.calculate(elevatorSetHeight, SensorStatus.kElevatorHeight), 1), -1)); // negative
																														// because
																														// up
																														// is
																														// reverse
	}

	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return Math.abs(elevatorSetHeight - SensorStatus.kElevatorHeight) < .1 || timer.hasElapsed(2);
	}
}
