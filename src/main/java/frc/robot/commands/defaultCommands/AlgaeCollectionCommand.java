package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollectionSubsystem;

public class AlgaeCollectionCommand extends Command{
    AlgaeCollectionSubsystem AlgaeSubsystem;
    Supplier<Boolean> rightJoyStickBackButton;

    public AlgaeCollectionCommand(AlgaeCollectionSubsystem AlgeaSubsystem, Supplier<Boolean> rightJoyStickBackButton) {
        this.AlgaeSubsystem = AlgeaSubsystem;
        this.rightJoyStickBackButton = rightJoyStickBackButton;
        addRequirements(AlgaeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if (rightJoyStickBackButton.get()) {
            AlgaeSubsystem.runSpinMotor(.2);
            AlgaeSubsystem.setAngleSetpoint(175);
        }else{
            AlgaeSubsystem.runSpinMotor(0);
            AlgaeSubsystem.setAngleSetpoint(120);
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
