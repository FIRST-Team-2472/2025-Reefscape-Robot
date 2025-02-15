package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SensorStatus;
import frc.robot.subsystems.AlgaeCollectionSubsystem;

public class AlgaeCollectionCommand extends Command{
    AlgaeCollectionSubsystem AlgaeSubsystem;
    Supplier<Boolean> leftJoystickTrigger, leftJoystickButtonFour;
    boolean hasAlgea = false;

    public AlgaeCollectionCommand(AlgaeCollectionSubsystem AlgeaSubsystem, Supplier<Boolean> leftJoystickTrigger, Supplier<Boolean> leftJoystickButtonFour) {
        this.AlgaeSubsystem = AlgeaSubsystem;
        this.leftJoystickTrigger = leftJoystickTrigger;
        this.leftJoystickButtonFour = leftJoystickButtonFour;
        addRequirements(AlgaeSubsystem);
    }
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(AlgaeSubsystem.pivotmotor.getOutputCurrent() > 10)// 10 is subject to change
            hasAlgea = true;

        if (leftJoystickTrigger.get()) {
            AlgaeSubsystem.runSpinMotor(.2);
            AlgaeSubsystem.setAngleSetpoint(175);

        }else if(leftJoystickButtonFour.get()){
            AlgaeSubsystem.setAngleSetpoint(130);

            if(Math.abs(SensorStatus.kPivotAngle - 130) < 3){
                AlgaeSubsystem.runSpinMotor(-1);
                hasAlgea = false;
            }else{
                AlgaeSubsystem.runSpinMotor(0.2);
            }

        }else{
            if(hasAlgea)
                AlgaeSubsystem.runSpinMotor(.3);
            else
                AlgaeSubsystem.runSpinMotor(0);

            AlgaeSubsystem.setAngleSetpoint(120);// resting vertical angle
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
