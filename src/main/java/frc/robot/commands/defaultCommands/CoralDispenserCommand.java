package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SensorStatus;
import frc.robot.subsystems.CoralDispenserSubsystem;
import java.util.function.Supplier;

public class CoralDispenserCommand extends Command {
    CoralDispenserSubsystem coralDispenserSubsystem;
    Supplier<Double> xboxControllerRightTrigger, xboxControllerLeftTrigger;

    public CoralDispenserCommand(
            CoralDispenserSubsystem coralDispenserSubsystem,
            Supplier<Double> xboxControllerRightTrigger,
            Supplier<Double> xboxControllerLeftTrigger) {
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        this.xboxControllerLeftTrigger = xboxControllerLeftTrigger;
        addRequirements(coralDispenserSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (xboxControllerRightTrigger.get() > .3)
            if (SensorStatus.kElevatorHeight > 8 && SensorStatus.kElevatorHeight < 10) {
                coralDispenserSubsystem.runMotors(.9, -.3);
                coralDispenserSubsystem.hascoral = false;
            } else if (SensorStatus.kElevatorHeight < 3) {
                if (!coralDispenserSubsystem.hascoral) {
                    coralDispenserSubsystem.runMotors(.5, -.5);
                } else {
                    coralDispenserSubsystem.runMotors(0, 0);
                }
            } else {
                coralDispenserSubsystem.runMotors(.8, -.8); // subject to change
                coralDispenserSubsystem.hascoral = false;
            }
        else if (xboxControllerLeftTrigger.get() > 0.3) {
            coralDispenserSubsystem.runMotors(-.3, .3);
            coralDispenserSubsystem.hascoral = false;
        } else coralDispenserSubsystem.runMotors(0, 0);
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
