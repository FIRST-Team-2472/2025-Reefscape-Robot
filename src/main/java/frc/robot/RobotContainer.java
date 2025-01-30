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


import frc.robot.commands.defaultCommands.SwerveJoystickCmd;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;


public class RobotContainer {
  //Add subsystems below this comment
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


  //Make sure this xbox controller is correct and add driver sticks
  XboxController xboxController = new XboxController(OperatorConstants.kXboxControllerPort);


  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //Controllers need to be added
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
