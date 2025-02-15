package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import java.util.function.Supplier;
import frc.robot.MotorPowerController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;

public class ElevatorCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;
    LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
    Supplier<Double> joystickY;
    MotorPowerController motorPowerController;
    double elevatorSetHeight = 0;
    Supplier<Boolean> XboxYPressed, XboxBPressed, XboxAPressed, XboxXPressed;
    boolean auto;
    Timer timer = new Timer();

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
        motorPowerController = new MotorPowerController(0.07, 0.05, 0.2, 1, 1, SensorStatus.kElevatorHeight, 5);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double y = joystickY.get();
        if (Math.abs(y) <= OperatorConstants.kXboxControllerDeadband) {
            y = 0;
            auto = false;
        }
        elevatorSetHeight += y;

        if (XboxYPressed.get()) {
            elevatorSetHeight = ElevatorConstants.kElevatorL4Height;
            auto = true;
            timer.reset();
        }
        if (XboxBPressed.get()) {
            elevatorSetHeight = ElevatorConstants.kElevatorL3Height;
            auto = true;
            timer.reset();
        }
        if (XboxAPressed.get()) {
            elevatorSetHeight = ElevatorConstants.kElevatorL2Height;
            auto = true;
            timer.reset();
        }
        if (XboxXPressed.get()) {
            elevatorSetHeight = ElevatorConstants.kElevatorL1Height;
            auto = true;
            timer.reset();
        }

        if (timer.hasElapsed(2)) {
            auto = false;
        }

        ledSubsystem.runningAutonomistCommand(auto);

        // Makes so cannot go past physical limits or below 0
        if (elevatorSetHeight > ElevatorConstants.kElevatorMaxHeight)
            elevatorSetHeight = ElevatorConstants.kElevatorMaxHeight;
        if (elevatorSetHeight < 0)
            elevatorSetHeight = 0;

        SmartDashboard.putNumber("elevatorSetHeight", elevatorSetHeight);
        SmartDashboard.putNumber("elevator drive factor",
                -motorPowerController.calculateMotorPowerController(elevatorSetHeight, SensorStatus.kElevatorHeight));
        elevatorSubsystem.runElevatorMotors(Math.max(Math.min(
                -motorPowerController.calculateMotorPowerController(elevatorSetHeight, SensorStatus.kElevatorHeight),
                .6), -1)); // negative because up is reverse

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // todo
    //
}