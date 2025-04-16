package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStatus;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralDispenserCommand extends Command{
    CoralDispenserSubsystem coralDispenserSubsytem;
    Supplier<Double> xboxControllerRightTrigger, xboxControllerLeftTrigger;

    public CoralDispenserCommand(CoralDispenserSubsystem coralDispenserSubsytem, Supplier<Double> xboxControllerRightTrigger, Supplier<Double> xboxControllerLeftTrigger){
        this.coralDispenserSubsytem = coralDispenserSubsytem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        this.xboxControllerLeftTrigger = xboxControllerLeftTrigger;
        addRequirements(coralDispenserSubsytem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(xboxControllerRightTrigger.get() > 0.3)
            if(RobotStatus.kElevatorHeight > 8 && RobotStatus.kElevatorHeight < 10) {
                coralDispenserSubsytem.runMotors(.9, -.3);
                RobotStatus.hasCoral = false;
            }
            else if (RobotStatus.kElevatorHeight < 3) {
                if (!RobotStatus.hasCoral) {
                    coralDispenserSubsytem.runMotors(.2, -.2);
                } else {
                    coralDispenserSubsytem.runMotors(0, 0);
                }
            }
            else {
                coralDispenserSubsytem.runMotors(.8, -.8);//subject to change
                RobotStatus.hasCoral = false;
            }
        else if(xboxControllerLeftTrigger.get() > 0.5) {
            coralDispenserSubsytem.runMotors(-.3, .3);
            RobotStatus.hasCoral = false;
        }
        else
            coralDispenserSubsytem.runMotors(0, 0);
        
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

