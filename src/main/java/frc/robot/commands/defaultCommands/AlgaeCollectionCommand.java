package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeCollectionSubsystem;

public class AlgaeCollectionCommand extends Command {
    AlgaeCollectionSubsystem AlgaeSubsystem;
    Supplier<Boolean> xboxControllerLeftTrigger;
    Supplier<Double> JoystickY;

    public AlgaeCollectionCommand(AlgaeCollectionSubsystem AlgeaSubsystem, Supplier<Boolean> xboxControllerLeftTrigger,
            Supplier<Double> JoystickY) {
        this.AlgaeSubsystem = AlgeaSubsystem;
        this.xboxControllerLeftTrigger = xboxControllerLeftTrigger;
        this.JoystickY = JoystickY;
        addRequirements(AlgaeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double y = JoystickY.get();
        // implementing dead band
        if (Math.abs(y) <= OperatorConstants.kXboxControllerDeadband) {
            y = 0;
        }
        AlgaeSubsystem.runPivotMotor(y);
        
        if (xboxControllerLeftTrigger.get()) {
            AlgaeSubsystem.runSpinMotor(.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
//ToDo add limits