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
import frc.robot.commands.defaultCommands.ElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
//Add subsystems below this comment

ElevatorSubsystem elevatorSubsystem;

//Make sure this xbox controller is correct and add driver sticks
  XboxController xboxController = new XboxController(OperatorConstants.kXboxControllerPort);


  public RobotContainer() {

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, 
    () -> xboxController.getLeftY(), 
    () -> xboxController.getYButton(), 
    () -> xboxController.getBButton(), 
    () -> xboxController.getAButton(), 
    () -> xboxController.getXButton()));

    configureBindings();
  }

  private void configureBindings() {
    //Controllers need to be added
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
