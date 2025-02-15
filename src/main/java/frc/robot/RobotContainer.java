// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.Joystick;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.defaultCommands.AlgaeCollectionCommand;
import frc.robot.commands.defaultCommands.ClimbCommand;
import frc.robot.commands.defaultCommands.CoralDispenserCommand;

import frc.robot.commands.defaultCommands.ElevatorCommand;
import frc.robot.commands.defaultCommands.SwerveJoystickCmd;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AlgaeCollectionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {
  private final String testAuto = "testAuto", swervedtptest = "Swerve Drive to Point Test";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();

  //Add subsystems below this comment
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  CoralDispenserSubsytem coralDispenserSubsytem = new CoralDispenserSubsytem();
  LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

  AlgaeCollectionSubsystem algaeCollectionSubsystem = new AlgaeCollectionSubsystem();

  ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  CoralDispenserSubsystem coralDispenserSubsystem = new CoralDispenserSubsystem();


  //Make sure this xbox controller is correct and add driver sticks
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
      ()-> -leftJoystick.getY(),
      ()-> -rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1),
      ()-> leftJoystick.getRawButton(1)
    ));

    m_chooser.addOption(testAuto, testAuto);
    m_chooser.addOption(swervedtptest, swervedtptest);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, 
    () -> -xboxController.getLeftY(), 
    () -> xboxController.getYButton(), 
    () -> xboxController.getBButton(), 
    () -> xboxController.getAButton(), 
    () -> xboxController.getXButton()
    ));

    coralDispenserSubsystem.setDefaultCommand(new CoralDispenserCommand(coralDispenserSubsystem, 
    () -> xboxController.getRightTriggerAxis(),
    () -> xboxController.getLeftTriggerAxis()
    ));
     
    algaeCollectionSubsystem.setDefaultCommand(new AlgaeCollectionCommand(algaeCollectionSubsystem, 
    () -> leftJoystick.getRawButton(2), 
    () -> leftJoystick.getRawAxis(3)
    ));
    
    climbSubsystem.setDefaultCommand(new ClimbCommand(climbSubsystem, 
    () -> xboxController.getRightY(), 
    () -> xboxController.getLeftBumperButton(), 
    () -> xboxController.getRightBumperButton()
    ));

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
    m_autoSelected = m_chooser.getSelected();

    if(m_autoSelected == testAuto)
        //return AutoBuilder.buildAuto("test");
        return new SequentialCommandGroup(
          commandSequences.test(swerveSubsystem)
        );

    if(m_autoSelected == swervedtptest)
      return new SequentialCommandGroup(
        commandSequences.swervePointTest(swerveSubsystem)
      );

    return null;
  }

}
