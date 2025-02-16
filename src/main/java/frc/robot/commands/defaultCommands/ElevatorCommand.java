package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;

public class ElevatorCommand extends Command{
    ElevatorSubsystem elevatorSubsystem;
    Supplier<Double> joystickY;
    double elevatorSetHeight = 0;
    Supplier<Boolean> XboxYPressed,XboxBPressed,XboxAPressed,XboxXPressed;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> joystickY, Supplier<Boolean> XboxYPressed, Supplier<Boolean> XboxBPressed, Supplier<Boolean> XboxAPressed, Supplier<Boolean> XboxXPressed){
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystickY = joystickY;
        this.XboxYPressed = XboxYPressed;
        this.XboxBPressed = XboxBPressed;
        this.XboxAPressed = XboxAPressed;
        this.XboxXPressed = XboxXPressed;
        addRequirements(elevatorSubsystem);
       
    }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double y = joystickY.get();
    if(Math.abs(y) <= OperatorConstants.kXboxControllerDeadband)
        y = 0;
    
    elevatorSetHeight += y;

    if(XboxYPressed.get())
        elevatorSetHeight = ElevatorConstants.kElevatorL4Height;
    if(XboxBPressed.get())
        elevatorSetHeight = ElevatorConstants.kElevatorL3Height;
    if(XboxAPressed.get())
        elevatorSetHeight = ElevatorConstants.kElevatorL2Height;
    if(XboxXPressed.get())
        elevatorSetHeight = ElevatorConstants.kElevatorL1Height;
        
    //Makes so cannot go past physical limits or below 0
    if (elevatorSetHeight > ElevatorConstants.kElevatorMaxHeight)
        elevatorSetHeight = ElevatorConstants.kElevatorMaxHeight;
    if (elevatorSetHeight < 0)
        elevatorSetHeight = 0;

    SmartDashboard.putNumber("elevatorSetHeight", elevatorSetHeight);
    elevatorSubsystem.runElevatorMotorsWithMotorPowerController(elevatorSetHeight); //negative because up is reverse
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //todo 
  //
}