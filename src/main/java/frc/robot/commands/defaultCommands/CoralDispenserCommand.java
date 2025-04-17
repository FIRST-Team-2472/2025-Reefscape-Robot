package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SensorStatus;
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
            if(SensorStatus.kElevatorHeight > 8 && SensorStatus.kElevatorHeight < 10) {
                coralDispenserSubsytem.runMotors(.9, -.3);
                SensorStatus.hasCoral = false;
            }
            else if (SensorStatus.kElevatorHeight < 3) {
                if (!SensorStatus.hasCoral) {
                    coralDispenserSubsytem.runMotors(.2, -.2);
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
                coralDispenserSubsytem.runMotors(.8, -.8);//subject to change
                SensorStatus.hasCoral = false;
            }
        else if(xboxControllerLeftTrigger.get() > 0.5) {
            coralDispenserSubsytem.runMotors(-.3, .3);
            SensorStatus.hasCoral = false;
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

