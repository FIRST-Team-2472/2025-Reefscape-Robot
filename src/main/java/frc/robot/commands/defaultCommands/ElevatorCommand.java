package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;
import frc.robot.MotorPowerController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;

public class ElevatorCommand extends Command{
    ElevatorSubsystem elevatorSubsystem;
    Supplier<Double> joystickY;
   MotorPowerController motorPowerController;
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
        motorPowerController = new MotorPowerController(0.0083, 0.1, 0.3, 2, 1, SensorStatus.kElevatorHeight, 10);
    }

    //set height - need variable
    //feed throughMotorPowerController to figure out how to get there
    //ultimately have something to move to that height 
    //Called when the command is initially scheduled.

    //
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double y = joystickY.get();
    if(Math.abs(y) <= OperatorConstants.kXboxControllerDeadband)
        y = 0;
    //y *= .3; // remove this when it becomes automatic
    
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
    SmartDashboard.putNumber("elevator drive factor", -motorPowerController.calculateMotorPowerController(elevatorSetHeight, SensorStatus.kElevatorHeight));
    elevatorSubsystem.runElevatorMotors(Math.max(Math.min(-motorPowerController.calculateMotorPowerController(elevatorSetHeight, SensorStatus.kElevatorHeight), .3), -1)); //negative because up is reverse
    
    //elevatorSubsystem.runElevatorMotors(y); // remove this when it becomes automatic
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