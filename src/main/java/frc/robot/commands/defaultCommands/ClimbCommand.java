package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;
import frc.robot.PID;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ClimbCommand extends Command {
    ClimbSubsystem climberSusbsystem;
    LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
    Supplier<Double> xboxControllerY;
    Supplier<Boolean> xboxControllerLeftBumper, xboxControllerRightBumper;
    boolean anglingOut = false;
    boolean anglingIn = false;
    PID climberPID = new PID(0.0001, 0.0001, 0.0001, 1, SensorStatus.kClimberAngle);

    public ClimbCommand(ClimbSubsystem climberSusbsystem, Supplier<Double> xboxControllerY,
            Supplier<Boolean> xboxControllerLeftBumper, Supplier<Boolean> xboxControllerRightBumper) {
        this.climberSusbsystem = climberSusbsystem;
        this.xboxControllerY = xboxControllerY;
        this.xboxControllerLeftBumper = xboxControllerLeftBumper;
        this.xboxControllerRightBumper = xboxControllerRightBumper;
        addRequirements(climberSusbsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double y = xboxControllerY.get();

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
        // automated climbing 
        if (anglingIn) {
            ledSubsystem.LEDMode(LEDStatusMode.BLUE);
            y = climberPID.calculatePID(ClimberConstants.kClimberInAngle, SensorStatus.kClimberAngle);
        }
        if (anglingOut) {
            ledSubsystem.LEDMode(LEDStatusMode.BLUE);
            y = climberPID.calculatePID(ClimberConstants.kClimberOutAngle, SensorStatus.kClimberAngle);
        }

        if (y != 0) {
            ledSubsystem.LEDMode(LEDStatusMode.RED);
        }
        if (SensorStatus.kClimberAngle >= ClimberConstants.kClimberInAngle-20) {
            ledSubsystem.LEDMode(LEDStatusMode.GREEN);
        }
        climberSusbsystem.runClimberMotors(y);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}