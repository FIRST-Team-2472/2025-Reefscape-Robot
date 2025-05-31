package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.SensorStatus;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorCommand extends Command {
    Timer timer = new Timer();
    ElevatorSubsystem elevatorSubsystem;
    double elevatorSetHeight, delayTime;
    MotorPowerController motorPowerController;
    Timer delayTimer = new Timer();

    public AutoElevatorCommand(
            ElevatorSubsystem elevatorSubsystem, double elevatorSetHeight, double delayTime) {
        this.elevatorSetHeight = elevatorSetHeight;
        this.elevatorSubsystem = elevatorSubsystem;
        motorPowerController =
                new MotorPowerController(0.07, 0.05, 0.2, 1, 1, SensorStatus.kElevatorHeight, 5);
        addRequirements(elevatorSubsystem);
        this.delayTime = delayTime;
        // these are guessed numbers, they need to be tuned
    }

    public AutoElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorSetHeight) {
        this(elevatorSubsystem, elevatorSetHeight, 0);
    }

    @Override
    public void initialize() {

        timer.restart();
        delayTimer.restart();
    }

    @Override
    public void execute() {
        if (delayTimer.hasElapsed(delayTime))
            elevatorSubsystem.runElevatorMotors(
                    Math.max(
                            Math.min(
                                    -motorPowerController.calculate(
                                            elevatorSetHeight, SensorStatus.kElevatorHeight),
                                    1),
                            -1)); // negative because up is reverse
    }

    public void end(boolean interrupted) {
        elevatorSubsystem.runElevatorMotors(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSetHeight - SensorStatus.kElevatorHeight) < .1
                || timer.hasElapsed(2);
    }
}
