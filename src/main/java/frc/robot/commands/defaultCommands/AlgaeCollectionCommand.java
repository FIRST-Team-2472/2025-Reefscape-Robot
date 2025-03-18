package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollectionSubsystem;
import java.util.function.Supplier;

public class AlgaeCollectionCommand extends Command {
    AlgaeCollectionSubsystem AlgaeSubsystem;
    Supplier<Boolean> leftJoystickTrigger, leftJoystickButtonFour;
    boolean hasAlgea = false;
    int currentSetAngle = 120;

    public AlgaeCollectionCommand(
            AlgaeCollectionSubsystem AlgeaSubsystem,
            Supplier<Boolean> leftJoystickTrigger,
            Supplier<Boolean> leftJoystickButtonFour) {
        this.AlgaeSubsystem = AlgeaSubsystem;
        this.leftJoystickTrigger = leftJoystickTrigger;
        this.leftJoystickButtonFour = leftJoystickButtonFour;
        addRequirements(AlgaeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (AlgaeSubsystem.spinmotor.getOutputCurrent() > 5) // 10 is subject to change
        hasAlgea = true;
        else hasAlgea = false;

        if (leftJoystickTrigger.get()) {
            AlgaeSubsystem.runSpinMotor(.5);
            currentSetAngle = 175;

        } else if (leftJoystickButtonFour.get()) {
            currentSetAngle = 170;
            AlgaeSubsystem.runSpinMotor(-1);

        } else {
            if (hasAlgea) {
                AlgaeSubsystem.runSpinMotor(.9);
                incrementSetAngle(165);
            } else {
                AlgaeSubsystem.runSpinMotor(0);
                currentSetAngle = 105;
            }
        }

        AlgaeSubsystem.setAngleSetpoint(currentSetAngle);
        SmartDashboard.putNumber("currentSetAngle", currentSetAngle);
    }

    public void incrementSetAngle(int desired) {
        if (desired > currentSetAngle) currentSetAngle++;
        else if (desired < currentSetAngle) currentSetAngle--;
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
