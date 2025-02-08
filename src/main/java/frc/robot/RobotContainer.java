// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoPrepForClimbCommand;
import frc.robot.commands.defaultCommands.CoralDispenserCommand;

import frc.robot.commands.defaultCommands.ElevatorCommand;
import frc.robot.commands.defaultCommands.SwerveJoystickCmd;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralCollectionSubsystem;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Joystick;


public class RobotContainer {
  //Add subsystems below this comment
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  CoralCollectionSubsystem coralCollectionSubsystem = new CoralCollectionSubsystem();

  CoralDispenserSubsystem coralDispenserSubsystem = new CoralDispenserSubsystem();


  //Make sure this xbox controller is correct and add driver sticks
  CommandXboxController xboxController = new CommandXboxController(OperatorConstants.kXboxControllerPort);

  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> -leftJoystick.getX(),
      ()-> -leftJoystick.getY(),
      ()-> -rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1),
      ()-> leftJoystick.getRawButton(1)
    ));

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, 
      ()-> xboxController.getLeftY(),
      ()-> xboxController.y().getAsBoolean(), 
      ()-> xboxController.b().getAsBoolean(), 
      ()-> xboxController.a().getAsBoolean(), 
      ()-> xboxController.x().getAsBoolean()
      ));

    coralDispenserSubsystem.setDefaultCommand(new CoralDispenserCommand(coralDispenserSubsystem, 
    () -> xboxController.getRightTriggerAxis(),
    () -> xboxController.getLeftTriggerAxis()
    ));

    configureBindings();
  }

  private void configureBindings() {
    //Controllers need to be added
    /*
    xboxController.a().onTrue(new AutoPrepForClimbCommand(coralCollectionSubsystem, 30));
    xboxController.b().onTrue(new AutoPrepForClimbCommand(coralCollectionSubsystem, 0));
    */
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
