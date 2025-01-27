package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

import java.util.function.Supplier;
import frc.robot.PID;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;

public class ElevatorCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;
    LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
    Supplier<Double> joystickY;
    PID elevatorPID;
    double elevatorSetHeight = 0;
    Supplier<Boolean> XboxYPressed, XboxBPressed, XboxAPressed, XboxXPressed;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> joystickY,
            Supplier<Boolean> XboxYPressed, Supplier<Boolean> XboxBPressed, Supplier<Boolean> XboxAPressed,
            Supplier<Boolean> XboxXPressed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystickY = joystickY;
        this.XboxYPressed = XboxYPressed;
        this.XboxBPressed = XboxBPressed;
        this.XboxAPressed = XboxAPressed;
        this.XboxXPressed = XboxXPressed;
        addRequirements(elevatorSubsystem);
        elevatorPID = new PID(0.0001, 0.0001, 0.0001, 10, SensorStatus.kElevatorHeight);
    }

    // set height - need variable
    // feed through pid to figure out how to get there
    // ultimately have something to move to that height
    // Called when the command is initially scheduled.

    //
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double y = joystickY.get();
        if (Math.abs(y) <= OperatorConstants.kXboxControllerDeadband)
            y = 0;
        elevatorSetHeight += y;

        // automated presets
        if (XboxYPressed.get()) {
            ledSubsystem.LEDMode(LEDStatusMode.BLUE);
            elevatorSetHeight = ElevatorConstants.kElevatorL4Height;
        }
        if (XboxBPressed.get()) {
            ledSubsystem.LEDMode(LEDStatusMode.BLUE);
            elevatorSetHeight = ElevatorConstants.kElevatorL3Height;
        }
        if (XboxAPressed.get()) {
            ledSubsystem.LEDMode(LEDStatusMode.BLUE);
            elevatorSetHeight = ElevatorConstants.kElevatorL2Height;
        }
        if (XboxXPressed.get()) {
            ledSubsystem.LEDMode(LEDStatusMode.BLUE);
            elevatorSetHeight = ElevatorConstants.kElevatorL1Height;
        }

        // Makes so cannot go past physical limits or below 0
        if (elevatorSetHeight > ElevatorConstants.kElevatorMaxHeight)
            elevatorSetHeight = ElevatorConstants.kElevatorMaxHeight;
        if (elevatorSetHeight < 0)
            elevatorSetHeight = 0;

        elevatorSubsystem.runElevatorMotor(elevatorPID.calculatePID(elevatorSetHeight, SensorStatus.kElevatorHeight));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // todo
    //
}