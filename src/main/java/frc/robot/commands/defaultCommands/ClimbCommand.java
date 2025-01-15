package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command{
    ClimbSubsystem climberSusbsystem;
    Supplier<Double> xboxControllerY;
    Supplier<Boolean> xboxControllerLeftBumper, xboxControllerRightBumper;

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
        if(Math.abs(y) <= OperatorConstants.kXboxControllerDeadband)
            y = 0;

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