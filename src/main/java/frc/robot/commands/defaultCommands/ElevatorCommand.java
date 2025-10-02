package frc.robot.commands.defaultCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.Constants.SensorConstants;
import frc.robot.Constants;
import frc.robot.MotorPowerController;
import frc.robot.RobotStatus;

public class ElevatorCommand extends Command{
    ElevatorSubsystem elevatorSubsystem;
    Supplier<Double> joystickY;
    MotorPowerController motorPowerController;
    double elevatorSetHeight = RobotStatus.kElevatorHeight;
    Supplier<Boolean> XboxYPressed,XboxBPressed,XboxAPressed,XboxXPressed,toggleElevatorRestrictions;
    private final Timer modeToggleCooldownTimer = new Timer();

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> joystickY, Supplier<Boolean> XboxYPressed, Supplier<Boolean> XboxBPressed, Supplier<Boolean> XboxAPressed, Supplier<Boolean> XboxXPressed, Supplier<Boolean> toggleElevatorRestrictions){
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystickY = joystickY;
        this.XboxYPressed = XboxYPressed;
        this.XboxBPressed = XboxBPressed;
        this.XboxAPressed = XboxAPressed;
        this.XboxXPressed = XboxXPressed;
        this.toggleElevatorRestrictions = toggleElevatorRestrictions;
        addRequirements(elevatorSubsystem);
        motorPowerController = new MotorPowerController(0.07, 0.01, 0.2, 1, 1, RobotStatus.kElevatorHeight, 5);
    }

  @Override
  public void initialize() {
        modeToggleCooldownTimer.start();
        elevatorSetHeight = RobotStatus.kElevatorHeight;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = joystickY.get();
    if(Math.abs(y) <= OperatorConstants.kXboxControllerDeadband)
        y = 0;
    //if kid mode we slow down the inputs to match the ability of the robot in the mode
    if(RobotStatus.kidMode)
        elevatorSetHeight += y*Constants.TeleDriveConstants.kKidDriveMultiplier;
    else
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
    if(RobotStatus.elevatorSafety && RobotStatus.seeCoral)
        elevatorSetHeight = 0;

    SmartDashboard.putNumber("elevatorSetHeight", elevatorSetHeight);

    SmartDashboard.putNumber("elevator drive factor", -motorPowerController.calculate(elevatorSetHeight, RobotStatus.kElevatorHeight));
    //if kid mode we add power limits too if not we just use the pid controller
    if(RobotStatus.kidMode)
        elevatorSubsystem.runElevatorMotors(Math.max(Math.min(-motorPowerController.calculate(elevatorSetHeight, RobotStatus.kElevatorHeight), Constants.TeleDriveConstants.kKidDriveMultiplier), -Constants.TeleDriveConstants.kKidDriveMultiplier)); //negative because up is reverse
    else
        elevatorSubsystem.runElevatorMotors(-motorPowerController.calculate(elevatorSetHeight, RobotStatus.kElevatorHeight)); //negative because up is reverse
    
    if(toggleElevatorRestrictions.get() && modeToggleCooldownTimer.get() > .3){
        RobotStatus.elevatorSafety = !RobotStatus.elevatorSafety;
        modeToggleCooldownTimer.reset();
    }
    SmartDashboard.putBoolean("Elevator Safety", RobotStatus.elevatorSafety);
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