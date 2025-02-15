package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CoralDispenserCommand extends Command{
    private static final int CORAL_STATION_1Y_RANGE = 3;
    private static final int CORAL_STATION_1X_RANGE = 3;
    private static final int CORAL_STATION_2Y_RANGE = 5;
    private static final int CORAL_STATION_2X_RANGE = 12;
    private static final int TRIGGER_DISTANCE = 1;
    CoralDispenserSubsystem coralDispenserSubsytem;
    Supplier<Double> xboxControllerRightTrigger, xboxControllerLeftTrigger;
    SwerveSubsystem swerveSubsystem;

    int fieldMin = 0;
    int fieldMax = 8;
    
    //8 is a placeholder

    public CoralDispenserCommand(CoralDispenserSubsystem coralDispenserSubsytem, Supplier<Double> xboxControllerRightTrigger, Supplier<Double> xboxControllerLeftTrigger, SwerveSubsystem swerveSubsystem){
        this.coralDispenserSubsytem = coralDispenserSubsytem;
        this.xboxControllerRightTrigger = xboxControllerRightTrigger;
        this.xboxControllerLeftTrigger = xboxControllerLeftTrigger;
        addRequirements(coralDispenserSubsytem);
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(xboxControllerRightTrigger.get() > 0.5)
            if(SensorStatus.kElevatorHeight > 8 && SensorStatus.kElevatorHeight < 10)
                coralDispenserSubsytem.runMotors(.9, -.3);
            else 
                coralDispenserSubsytem.runMotors(.8, -.8);//subject to change
        else if(xboxControllerLeftTrigger.get() > 0.5){
                coralDispenserSubsytem.runMotors(-.3, .3);
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

    public boolean isNearCoralStation() {
        var currentRobotPos = swerveSubsystem.getPose();
        var currentY = currentRobotPos.getY();
        var currentX = currentRobotPos.getX();
        //create variable to hold distance from corner or variable of y- field max or min
        if((currentY <= CORAL_STATION_1Y_RANGE || currentY >= CORAL_STATION_2Y_RANGE) && 
        (currentX <= CORAL_STATION_1X_RANGE || currentX >= CORAL_STATION_2X_RANGE )) {
            //assuming maxX is roughly 15
            return true;
        } 
        return false;
    }
}