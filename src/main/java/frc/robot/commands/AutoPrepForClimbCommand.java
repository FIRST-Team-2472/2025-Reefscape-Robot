package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.defaultCommands.CoralDispenserCommand;
import frc.robot.subsystems.CoralCollectionSubsystem;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoPrepForClimbCommand extends Command{
    CoralDispenserSubsystem coralDispenserSubsystem;
    CoralCollectionSubsystem coralCollectionSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    int angle;
    public AutoPrepForClimbCommand(CoralDispenserSubsystem coralDispenserSubsystem, ElevatorSubsystem elevatorSubsystem, CoralCollectionSubsystem coralCollectionSubsystem){
        this.coralCollectionSubsystem = coralCollectionSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        addRequirements(coralCollectionSubsystem, elevatorSubsystem, coralDispenserSubsystem);
    }

    @Override
    public void initialize() {
        new ParallelCommandGroup(
            new InstantCommand(() -> coralCollectionSubsystem.setServoAngle(angle)),
            new AutoElevatorCommand(elevatorSubsystem, 0)
        ).schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}