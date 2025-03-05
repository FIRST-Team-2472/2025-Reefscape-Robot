package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.SensorStatus;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class AutoCoralDispenseCommand extends Command {
	CoralDispenserSubsystem coralDispenserSubsystem;
	MotorPowerController motorPowerController;
	Timer timoutTimer;

	public AutoCoralDispenseCommand(CoralDispenserSubsystem coralDispenserSubsystem) {
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
		if (SensorStatus.kElevatorHeight > 8 && SensorStatus.kElevatorHeight < 10) {
			coralDispenserSubsystem.runMotors(.9, -.3);
		} else {
			coralDispenserSubsystem.runMotors(1, -1);
		}

		coralDispenserSubsystem.hascoral = false;
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("Dispense Coral Command ENDED!");
		coralDispenserSubsystem.runMotors(0, 0);
	}

	@Override
	public boolean isFinished() {
		if (timoutTimer.hasElapsed(1)) {
			System.out.println("isFinished Dispensing Coral");
			return true;
		}
		return false;
	}
}
