package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.SensorStatus;
import frc.robot.subsystems.AlgaeCollectionSubsystem;

public class AngleAlgaeCollectorCommand extends Command{

    Timer timeoutTimer = new Timer();
    Timer atAngleTimer = new Timer();
    AlgaeCollectionSubsystem algaeCollectionSubsystem;
    MotorPowerController motorPowerController;

    public AngleAlgaeCollectorCommand(AlgaeCollectionSubsystem algaeCollectionSubsystem) {
        this.algaeCollectionSubsystem = algaeCollectionSubsystem;
        addRequirements(algaeCollectionSubsystem);
        motorPowerController = new MotorPowerController(0.03, 0.3, 0.2, 0.3, 1, SensorStatus.kClimberAngle, 5);
        // Use addRequirements() here to declare subsystem dependencies.

    }

    @Override
    public void initialize() {
        timeoutTimer.reset();
        atAngleTimer.reset();
        atAngleTimer.stop();
    }

    @Override
    public void execute() {
        algaeCollectionSubsystem.runPivotMotor(motorPowerController.calculateMotorPowerController(30, SensorStatus.kPivotAngle));
        algaeCollectionSubsystem.runSpinMotor(0.15);
        if (Math.abs(30 - SensorStatus.kPivotAngle) < 2) {
            atAngleTimer.start();
        }
    }
    
    public void end(boolean interrupted) {
        algaeCollectionSubsystem.runPivotMotor(0);
        algaeCollectionSubsystem.runSpinMotor(0);
    }

    @Override
    public boolean isFinished() {
        return atAngleTimer.hasElapsed(1) || timeoutTimer.hasElapsed(2);
    }
}