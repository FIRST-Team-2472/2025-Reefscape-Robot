package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.Supplier;
import frc.robot.Constants.SensorStatus;

public class DeepClimbCommand extends Command{
    ClimbSubsystem climberSubsystem;
    Supplier<Boolean> XboxAPressed;
    
    public DeepClimbCommand(ClimbSubsystem climberSubsystem, Supplier<Boolean> XboxAPressed){
        this.climberSubsystem = climberSubsystem;
        this.XboxAPressed = XboxAPressed;
        addRequirements(climberSubsystem);
    }
     @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (SensorStatus.kClimberAngle > 92){
            climberSubsystem.runClimberMotor(-0.2);
        }
        else if (SensorStatus.kClimberAngle < 88){
            climberSubsystem.runClimberMotor(0.2);
        }
        else {
            climberSubsystem.runClimberMotor(0);
        }
    }
    
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
//button pressed
//move arm to set place(
//run if statements 90
//run climberMotor, input value needed
//add tolerance)
//rotate robot