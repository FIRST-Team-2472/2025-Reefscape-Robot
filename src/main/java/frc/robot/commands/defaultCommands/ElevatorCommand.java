package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SensorStatus;
import frc.robot.PID;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;

public class ElevatorCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  Supplier<Double> joystickY;
  PID elevatorPID;
  double elevatorSetHeight = 0;
  Supplier<Boolean> XboxYPressed, XboxBPressed, XboxAPressed, XboxXPressed;

  public ElevatorCommand(
      ElevatorSubsystem elevatorSubsystem,
      Supplier<Double> joystickY,
      Supplier<Boolean> XboxYPressed,
      Supplier<Boolean> XboxBPressed,
      Supplier<Boolean> XboxAPressed,
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double y = joystickY.get();
    if (Math.abs(y) <= OperatorConstants.kXboxControllerDeadband) y = 0;
    elevatorSetHeight += y;

    if (XboxYPressed.get()) elevatorSetHeight = ElevatorConstants.kElevatorL4Height;
    if (XboxBPressed.get()) elevatorSetHeight = ElevatorConstants.kElevatorL3Height;
    if (XboxAPressed.get()) elevatorSetHeight = ElevatorConstants.kElevatorL2Height;
    if (XboxXPressed.get()) elevatorSetHeight = ElevatorConstants.kElevatorL1Height;

    // Makes so cannot go past physical limits or below 0
    if (elevatorSetHeight > ElevatorConstants.kElevatorMaxHeight)
      elevatorSetHeight = ElevatorConstants.kElevatorMaxHeight;
    if (elevatorSetHeight < 0) elevatorSetHeight = 0;

    elevatorSubsystem.runElevatorMotor(
        elevatorPID.calculatePID(elevatorSetHeight, SensorStatus.kElevatorHeight));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // todo
  //
}
