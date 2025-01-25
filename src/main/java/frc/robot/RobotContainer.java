// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.defaultCommands.ElevatorCommand;
import frc.robot.commands.defaultCommands.SwerveJoystickCmd;
import frc.robot.subsystems.CoralDispenserSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // Add subsystems below this comment
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  CoralDispenserSubsytem coralDispenserSubsytem = new CoralDispenserSubsytem();

  // Make sure this xbox controller is correct and add driver sticks
  XboxController xboxController = new XboxController(OperatorConstants.kXboxControllerPort);

  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
        new SwerveJoystickCmd(
            swerveSubsystem,
            () -> leftJoystick.getY(),
            () -> -leftJoystick.getX(),
            () -> rightJoystick.getX()));

    elevatorSubsystem.setDefaultCommand(
        new ElevatorCommand(
            elevatorSubsystem,
            () -> xboxController.getLeftY(),
            () -> xboxController.getYButton(),
            () -> xboxController.getBButton(),
            () -> xboxController.getAButton(),
            () -> xboxController.getXButton()));

    configureBindings();
  }

  private void configureBindings() {
    // Controllers need to be added
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
