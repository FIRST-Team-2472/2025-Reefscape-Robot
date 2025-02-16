package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SensorConstants;
import frc.robot.SensorStatus;

import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CoralDispenserCommand extends Command {
    private static final int CORAL_STATION_1Y_RANGE = 3;
    private static final int CORAL_STATION_2Y_RANGE = 5;
    private static final int TRIGGER_DISTANCE = 1;
    CoralDispenserSubsystem coralDispenserSubsystem;
    Supplier<Double> xboxControllerRightTrigger, xboxControllerLeftTrigger;
    SwerveSubsystem swerveSubsystem;

    int fieldMin = 0;

    public CoralDispenserCommand(CoralDispenserSubsystem coralDispenserSubsystem,
            Supplier<Double> xboxControllerRightTrigger, Supplier<Double> xboxControllerLeftTrigger,
            SwerveSubsystem swerveSubsystem) {
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        this.xboxControllerLeftTrigger = xboxControllerLeftTrigger;
        addRequirements(coralDispenserSubsystem);
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (xboxControllerRightTrigger.get() > 0.5)
            activateCoralDispenserUpperLevels();
        else if (xboxControllerLeftTrigger.get() > 0.5) {
            activateCoralDispenserLowestLevel();
        } else if (isNearCoralStation() && !coralDispenserSubsystem.hasCoralAlready()) {
            // TODO determine if we need lower or upper level to activate
            //Controlled by button
            activateCoralDispenserUpperLevels();

        } else
            // Do nothing with this as this stops motors
            coralDispenserSubsystem.runMotors(0, 0);
    }

    // activateCoralDispenserLowestLevel() is Level 1
    private void activateCoralDispenserLowestLevel() {
        coralDispenserSubsystem.runMotors(-.3, .3);
    }

    // activateCoralDispenserUpperLevels() is Levels 2, 3, 4
    private void activateCoralDispenserUpperLevels() {
        if (SensorStatus.kElevatorHeight > 8 && SensorStatus.kElevatorHeight < 10)
            coralDispenserSubsystem.runMotors(.9, -.3);
        else
            coralDispenserSubsystem.runMotors(.8, -.8);// subject to change
        //sensor should flip to true
    }

    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isNearCoralStation() {
        var currentRobotPos = swerveSubsystem.getPose();
        var currentY = currentRobotPos.getY();
        var currentX = currentRobotPos.getX();
        // create variable to hold distance from corner or variable of y- field max or
        // min
        if ((currentY <= CORAL_STATION_1Y_RANGE || currentY >= CORAL_STATION_2Y_RANGE) &&
                (currentX <= TRIGGER_DISTANCE + fieldMin
                        || currentX >= SensorConstants.sizeOfFieldMeters - TRIGGER_DISTANCE)) {
            // assuming maxX is roughly 15
            return true;
        }
        return false;
    }
}