package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeCollectionSubsystem;

public class AlgaeCollectionCommand extends Command {
    AlgaeCollectionSubsystem AlgaeSubsystem;
    Supplier<Boolean> rightJoyStickBackButton;
    Supplier<Double> JoystickY;

    public AlgaeCollectionCommand(AlgaeCollectionSubsystem AlgeaSubsystem, Supplier<Boolean> rightJoyStickBackButton, Supplier<Double> JoystickY) {
        this.AlgaeSubsystem = AlgeaSubsystem;
        this.rightJoyStickBackButton = rightJoyStickBackButton;
        this.JoystickY = JoystickY;
        addRequirements(AlgaeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double y = JoystickY.get();
        if(Math.abs(y) <= OperatorConstants.kFlightControllerDeadband) {
            y = 0;
        } // angle to collect is 220
        y *= 1;//limit it to 30 percent
        AlgaeSubsystem.runPivotMotor(y);

        if (rightJoyStickBackButton.get()) {
            AlgaeSubsystem.runSpinMotor(.2);
        }else{
            AlgaeSubsystem.runSpinMotor(0);
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