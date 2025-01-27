// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.CoralDispenserSubsytem;
import frc.robot.commands.defaultCommands.ElevatorCommand;
import frc.robot.commands.defaultCommands.SwerveJoystickCmd;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {
  // private GenericEntry headingShuffleBoard, odometerShuffleBoard, rollSB,
  // pitchSB;

  // Add subsystems below this comment
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  CoralDispenserSubsytem coralDispenserSubsytem = new CoralDispenserSubsytem();
  LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

  // Make sure this xbox controller is correct and add driver sticks
  XboxController xboxController = new XboxController(OperatorConstants.kXboxControllerPort);

  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");
  ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");

  private static final SendableChooser<String> colorChooser = SwerveSubsystem.colorChooser;
  GenericEntry headingShuffleBoard = swerveSubsystem.headingShuffleBoard;
  GenericEntry odometerShuffleBoard = swerveSubsystem.odometerShuffleBoard;
  GenericEntry rollSB = swerveSubsystem.rollSB;
  GenericEntry pitchSB = swerveSubsystem.pitchSB;
  String red = swerveSubsystem.red;
  String blue = swerveSubsystem.blue;
  Color color = ledSubsystem.color;
  private SuppliedValueWidget<Boolean> colorWidget = Shuffleboard.getTab("Driver Board")
  .addBoolean("Robot Statice", () -> true);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> leftJoystick.getY(),
        () -> -leftJoystick.getX(),
        () -> rightJoystick.getX()));

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem,
        () -> xboxController.getLeftY(),
        () -> xboxController.getYButton(),
        () -> xboxController.getBButton(),
        () -> xboxController.getAButton(),
        () -> xboxController.getXButton()));

    configureBindings();

    // Gets tabs from Shuffleboard

    // Sets up the different displays on suffle board
    headingShuffleBoard = programmerBoard.add("Robot Heading", 0).getEntry();
    odometerShuffleBoard = programmerBoard.add("Robot Location", "").getEntry();
    rollSB = programmerBoard.add("Roll", 0).getEntry();
    pitchSB = programmerBoard.add("Pitch", 0).getEntry();

    // makes a team color choser
    colorChooser.addOption(red, red);
    colorChooser.addOption(blue, blue);
    driverBoard.add("Team Chooser", colorChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    colorWidget.withProperties(Map.of("colorWhenTrue", color));
  }

  private void configureBindings() {
    // Controllers need to be added
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
