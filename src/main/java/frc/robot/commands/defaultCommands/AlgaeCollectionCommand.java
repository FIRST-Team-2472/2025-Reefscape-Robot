package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotStatus;
import frc.robot.subsystems.AlgaeCollectionSubsystem;

public class AlgaeCollectionCommand extends Command{
    AlgaeCollectionSubsystem AlgaeSubsystem;
    Supplier<Boolean> leftJoystickTrigger, leftJoystickButtonFour, flopButton;
    boolean hasAlgea = false;
    int currentSetAngle = 120;

    public AlgaeCollectionCommand(AlgaeCollectionSubsystem AlgeaSubsystem, Supplier<Boolean> leftJoystickTrigger, Supplier<Boolean> leftJoystickButtonFour, Supplier<Boolean> flopButton) {
        this.AlgaeSubsystem = AlgeaSubsystem;
        this.leftJoystickTrigger = leftJoystickTrigger;
        this.leftJoystickButtonFour = leftJoystickButtonFour;
        this.flopButton = flopButton;
        addRequirements(AlgaeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (leftJoystickTrigger.get()) {
            AlgaeSubsystem.runSpinMotor(.5);
            currentSetAngle = 175;

        }else if(leftJoystickButtonFour.get()){
            currentSetAngle = 170;
            AlgaeSubsystem.runSpinMotor(-1);

        }else if(flopButton.get()){
            currentSetAngle = 220;
            AlgaeSubsystem.runSpinMotor(0);

        }else{
            AlgaeSubsystem.runSpinMotor(0);
            currentSetAngle = 105;
        }

        AlgaeSubsystem.setAngleSetpoint(currentSetAngle);
        SmartDashboard.putNumber("currentSetAngle", currentSetAngle);
    }
    public void incrementSetAngle(int desired){
        if(desired > currentSetAngle)
            currentSetAngle++;
        else if(desired < currentSetAngle)
            currentSetAngle --;
    }
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
