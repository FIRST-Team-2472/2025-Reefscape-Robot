package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SensorStatus;
import frc.robot.MotorPowerController;
import frc.robot.MotorPowerController;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class ClimbCommand extends Command{
    ClimbSubsystem climberSusbsystem;
    LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
    Supplier<Double> xboxControllerY;
    Supplier<Boolean> xboxControllerLeftBumper, xboxControllerRightBumper;
    boolean anglingOut = false;
    boolean anglingIn = false;
    MotorPowerController climberMotorPowerController = new MotorPowerController(0.0001, 0.0001, 0.0001, 1, 0, SensorStatus.kClimberAngle, 0);
    public ClimbCommand(ClimbSubsystem climberSusbsystem, Supplier<Double> xboxControllerY, Supplier<Boolean> xboxControllerLeftBumper, Supplier<Boolean> xboxControllerRightBumper){
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

        if(xboxControllerLeftBumper.get()){
            anglingOut = true;
        }
        if(xboxControllerRightBumper.get()){
            anglingIn = true;
        }

        if(Math.abs(y) <= OperatorConstants.kXboxControllerDeadband){
            y = 0;
        }else{
            anglingOut = false;
            anglingIn = false;
        }
        if(anglingIn){
            y = climberMotorPowerController.calculateMotorPowerController(ClimberConstants.kClimberInAngle, SensorStatus.kClimberAngle);
        }
        if(anglingOut){
            y = climberMotorPowerController.calculateMotorPowerController(ClimberConstants.kClimberOutAngle, SensorStatus.kClimberAngle);
        }
        ledSubsystem.isClimbing(y != 0);
        ledSubsystem.climbAtAngle(SensorStatus.kClimberAngle >= ClimberConstants.kClimberInAngle-20);
        ledSubsystem.runningAutonomistCommand(anglingIn || anglingOut);
        
        climberSusbsystem.runClimberMotor(y);
    }
    
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}