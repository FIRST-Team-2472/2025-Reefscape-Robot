package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;
import frc.robot.RobotStatus;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CoralDispenserCommand extends Command{
    CoralDispenserSubsystem coralDispenserSubsystem;
    Supplier<Double> xboxControllerRightTrigger, xboxControllerLeftTrigger;
    Timer CollectionDelay;
    double delay = .1;

    public CoralDispenserCommand(CoralDispenserSubsystem coralDispenserSubsystem, Supplier<Double> xboxControllerRightTrigger, Supplier<Double> xboxControllerLeftTrigger){
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        this.xboxControllerLeftTrigger = xboxControllerLeftTrigger;
        addRequirements(coralDispenserSubsystem);
        CollectionDelay = new Timer();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(xboxControllerRightTrigger.get() > 0.3)
            if(RobotStatus.kElevatorHeight > ElevatorConstants.kElevatorL1Height - 1 && RobotStatus.kElevatorHeight < ElevatorConstants.kElevatorL1Height + 1) {
                coralDispenserSubsystem.runMotors(.9, -.3);
                coralDispenserSubsystem.hasCoral = false;
            } else if (RobotStatus.kElevatorHeight < 3) {
                if (!RobotStatus.hasCoral) {
                    coralDispenserSubsystem.runMotors(.2, -.2);
                } else {
                    if(CollectionDelay.hasElapsed(delay))
                        coralDispenserSubsystem.runMotors(0, 0);
                    else{
                        coralDispenserSubsystem.runMotors(.3, -.3);
                        CollectionDelay.start();
                    }
                }
            }
            else {
                coralDispenserSubsystem.runMotors(.8, -.8);//subject to change
                coralDispenserSubsystem.hasCoral = false;
            }
        else if(xboxControllerLeftTrigger.get() > 0.3) {
            coralDispenserSubsystem.runMotors(-.3, .3);
            coralDispenserSubsystem.hasCoral = false;
        }else{
            coralDispenserSubsystem.runMotors(0, 0);
            CollectionDelay.reset();
            CollectionDelay.stop();
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

