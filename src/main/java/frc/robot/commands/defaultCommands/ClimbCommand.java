package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.MotorPowerController;
import frc.robot.SensorStatus;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
    ClimbSubsystem climberSusbsystem;
    Supplier<Double> xboxControllerY;
    Supplier<Boolean> xboxControllerLeftBumper, xboxControllerRightBumper;
    boolean anglingOut = false;
    boolean anglingIn = false;
    MotorPowerController climberMotorPowerController = new MotorPowerController(0.0001, 0.0001, 0.0001, 1, 0, SensorStatus.kClimberAngle, 0);

    public ClimbCommand(ClimbSubsystem climberSusbsystem, Supplier<Double> xboxControllerY, Supplier<Boolean> xboxControllerLeftBumper, Supplier<Boolean> xboxControllerRightBumper) {
        this.climberSusbsystem = climberSusbsystem;
        this.xboxControllerY = xboxControllerY;
        this.xboxControllerLeftBumper = xboxControllerLeftBumper;
        this.xboxControllerRightBumper = xboxControllerRightBumper;
        addRequirements(climberSusbsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double y = xboxControllerY.get();
        System.out.println("Controller Y: " + y);

        if (xboxControllerLeftBumper.get()) {
            anglingOut = true;
        }
        if (xboxControllerRightBumper.get()) {
            anglingIn = true;
        }

        if (Math.abs(y) <= OperatorConstants.kXboxControllerDeadband) {
            y = 0;
        } else {
            anglingOut = false;
            anglingIn = false;
        }

        if (anglingIn) {
            y = climberMotorPowerController.calculate(ClimberConstants.kClimberInAngle, SensorStatus.kClimberAngle);
            System.out.println("Angling In: " + y);
        }
        if (anglingOut) {
            y = climberMotorPowerController.calculate(ClimberConstants.kClimberOutAngle, SensorStatus.kClimberAngle);
            System.out.println("Angling Out: " + y);
        }

        // Assuming there's a method to set the motor power in the climber subsystem
        climberSusbsystem.runClimberMotor(y);
        System.out.println("Motor Power Set To: " + y);
    }

    @Override
    public void end(boolean interrupted) {
        climberSusbsystem.runClimberMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}