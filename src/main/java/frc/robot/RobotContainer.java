// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCoralDispenseCommand;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.AutoPrepForClimbCommand;
import frc.robot.commands.HoldElevatorCommand;
import frc.robot.commands.defaultCommands.AlgaeCollectionCommand;
import frc.robot.commands.defaultCommands.ClimbCommand;
import frc.robot.commands.defaultCommands.CoralDispenserCommand;

import frc.robot.commands.defaultCommands.ElevatorCommand;
import frc.robot.commands.defaultCommands.SwerveJoystickCmd;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralCollectionSubsystem;
import frc.robot.subsystems.AlgaeCollectionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.commands.RunSwerve;

public class RobotContainer {

  private String m_autoSelected;
  private final String
    HToSit = "Drive To H and Sit",
    HL1 = "Drive To and Place on H L1", 
    HL4 = "Drive To and Place on H L4",
    IL4 = "Drive To and Place on I L4",
    FL4 = "Drive To and Place on F L4",
    DriveForwardTest = "Drive Forward Test",
    IL4ToLL4 = "Drive To IL4 LL4",
    FL4ToCL4 = "Drive To FL4 CL4"; 

  private String m_testSelected;
  private final String
    coralcollectionautotest = "Test Coral Collection - TEST FIRST!!!",
    GL1 = "Drive To and Place on G L1",
    GL4 = "Drive To and Place on G L4",
    IL1 = "Drive To and Place on I L1",
    FL1 = "Drive To and Place on F L1", 
    setOdodmetryToTest = "set Odometry to test";

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> testChooser = new SendableChooser<>();
  private final CommandSequences commandSequences = new CommandSequences();

  // Add subsystems below this comment
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //Add subsystems below this comment

  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  CoralCollectionSubsystem coralCollectionSubsystem = new CoralCollectionSubsystem();

  AlgaeCollectionSubsystem algaeCollectionSubsystem = new AlgaeCollectionSubsystem();

  ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  CoralDispenserSubsystem coralDispenserSubsystem = new CoralDispenserSubsystem();
  LEDSubsystem ledSubsystem = new LEDSubsystem();

  // Make sure this xbox controller is correct and add driver sticks
  CommandXboxController xboxController = new CommandXboxController(OperatorConstants.kXboxControllerPort);

  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> -leftJoystick.getX(), // negative because we get the inverse value
        () -> -leftJoystick.getY(), // negative because we get the inverse value
        () -> -rightJoystick.getX(),
        () -> rightJoystick.getRawButton(1),
        () -> rightJoystick.getRawButton(4)));

    autoChooser.addOption(DriveForwardTest, DriveForwardTest);
    autoChooser.addOption(HToSit, HToSit);
    autoChooser.addOption(HL1, HL1);
    autoChooser.addOption(HL4, HL4);
    autoChooser.addOption(IL4, IL4);
    autoChooser.addOption(FL4, FL4);
    autoChooser.addOption(IL4ToLL4, IL4ToLL4);
    autoChooser.addOption(FL4ToCL4, FL4ToCL4);

    testChooser.addOption(GL1, GL1);
    testChooser.addOption(GL4, GL4);
    testChooser.addOption(IL1, IL1);
    testChooser.addOption(FL1, FL1);
    testChooser.addOption(coralcollectionautotest, coralcollectionautotest);
    testChooser.addOption(setOdodmetryToTest, setOdodmetryToTest);

    if (DriverStation.isFMSAttached() == true) {
      ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
      driverBoard.add("Auto choices", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    } else {
      ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
      driverBoard.add("Auto choices", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

      ShuffleboardTab autoTestingBoard = Shuffleboard.getTab("Auto Testing");
      autoTestingBoard.add("Auto choices", testChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    ShuffleboardTab autoTestingBoard = Shuffleboard.getTab("Auto Testing");
    autoTestingBoard.add("Auto choices - in testing", testChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, 
        () -> -xboxController.getLeftY(),
        () -> xboxController.y().getAsBoolean(), 
        () -> xboxController.b().getAsBoolean(),
        () -> xboxController.a().getAsBoolean(), 
        () -> xboxController.x().getAsBoolean()));

    coralDispenserSubsystem.setDefaultCommand(new CoralDispenserCommand(coralDispenserSubsystem,
        () -> xboxController.getRightTriggerAxis(), 
        () -> xboxController.getLeftTriggerAxis()));
/**
    algaeCollectionSubsystem.setDefaultCommand(new AlgaeCollectionCommand(algaeCollectionSubsystem,
        () -> leftJoystick.getRawButton(1), 
        () -> leftJoystick.getRawButton(4),
        () -> leftJoystick.getRawButton(3)));

    climbSubsystem.setDefaultCommand(new ClimbCommand(climbSubsystem, 
        () -> xboxController.getRightY(),
        () -> xboxController.leftBumper().getAsBoolean(), 
        () -> xboxController.rightBumper().getAsBoolean()));
*//
    configureBindings();

  }

  private void configureBindings() {
    // xboxController.povUp().onTrue(new InstantCommand(() ->
    // coralCollectionSubsystem.setServoAngle(0)));
    // xboxController.povDown().onTrue(new InstantCommand(() ->
    // coralCollectionSubsystem.setServoAngle(180)));
    // Controllers need to be added
    /*
     * xboxController.a().onTrue(new
     * AutoPrepForClimbCommand(coralCollectionSubsystem, 30));
     * xboxController.b().onTrue(new
     * AutoPrepForClimbCommand(coralCollectionSubsystem, 0));
     */
    InstantCommand zeroGyro = new InstantCommand() {
      public boolean runsWhenDisabled() {
        return true;
      }
      @Override
      public void initialize() {
        swerveSubsystem.zeroRobotHeading();
      } 
      @Override
          public boolean isFinished() {
              return true;
          }
    };
    SmartDashboard.putData("Zero Gyro", zeroGyro);
  }

  public Command getAutonomousCommand() {
    m_autoSelected = autoChooser.getSelected();
    m_testSelected = testChooser.getSelected();

    if (m_autoSelected != null) {
      switch (m_autoSelected) {
        case HToSit:
          return new SequentialCommandGroup(
              commandSequences.StartToH(swerveSubsystem));

        case HL1:
          return new SequentialCommandGroup(
              commandSequences.StartToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case HL4:
          return new SequentialCommandGroup(
              commandSequences.StartToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case IL4:
          return new SequentialCommandGroup(
              commandSequences.StartToI(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case FL4:
          return new SequentialCommandGroup(
              commandSequences.StartToF(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case DriveForwardTest:
          return new SequentialCommandGroup(
              commandSequences.driveForwardTest(swerveSubsystem));

        case IL4ToLL4:
              return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                  commandSequences.StartToI(swerveSubsystem),
                  new SequentialCommandGroup(
                      new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height, .2),
                      new HoldElevatorCommand(elevatorSubsystem)
                  ) 
                ),
                new ParallelDeadlineGroup(
                  new AutoCoralDispenseCommand(coralDispenserSubsystem), 
                  new HoldElevatorCommand(elevatorSubsystem)
                ),
                new ParallelCommandGroup(
                  commandSequences.setElevatorL0(elevatorSubsystem),
                  commandSequences.IToLeftPlayer(swerveSubsystem)
                ),
                new ParallelDeadlineGroup(
                  commandSequences.collectCoral(coralDispenserSubsystem), 
                new RunSwerve(swerveSubsystem, -0.05, .05)),
                
  
                commandSequences.LeftPlayerToL(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
              );
        case FL4ToCL4:
              return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                  commandSequences.StartToF(swerveSubsystem),
                  new SequentialCommandGroup(
                    new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height, .2),
                    new HoldElevatorCommand(elevatorSubsystem)
                  ) 
                ),
                new ParallelDeadlineGroup(
                  new AutoCoralDispenseCommand(coralDispenserSubsystem), 
                  new HoldElevatorCommand(elevatorSubsystem)
                ),
                new ParallelCommandGroup(
                      commandSequences.setElevatorL0(elevatorSubsystem),
                      commandSequences.FToRightPlayer(swerveSubsystem)
                ),                  
                new ParallelDeadlineGroup(
                  commandSequences.collectCoral(coralDispenserSubsystem), 
                new RunSwerve(swerveSubsystem, -0.05, -0.05)),
    
                  commandSequences.RightPlayerToD(swerveSubsystem),
                  commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                      ElevatorConstants.kElevatorL4Height)
              );

        default:
          return null;
      }
    } else if (m_testSelected != null) {
      switch (m_testSelected) {
        case coralcollectionautotest:
          return new SequentialCommandGroup(
              commandSequences.coralcollectionautotest(coralDispenserSubsystem));

        case GL1:
          return new SequentialCommandGroup(
              commandSequences.StartToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case GL4:
          return new SequentialCommandGroup(
              commandSequences.StartToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case IL1:
          return new SequentialCommandGroup(
              commandSequences.StartToI(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case FL1:
          return new SequentialCommandGroup(
              commandSequences.StartToF(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case setOdodmetryToTest:
          commandSequences.setOdodmeteryToTest(swerveSubsystem);
      }
    }

    return null;
  }
}
