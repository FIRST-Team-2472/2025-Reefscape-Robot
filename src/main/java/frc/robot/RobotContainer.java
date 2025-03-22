// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoPrepForClimbCommand;
import frc.robot.commands.defaultCommands.AlgaeCollectionCommand;
import frc.robot.commands.defaultCommands.ClimbCommand;
import frc.robot.commands.defaultCommands.CoralDispenserCommand;

import frc.robot.commands.defaultCommands.ElevatorCommand;
import frc.robot.commands.defaultCommands.SwerveJoystickCmd;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PositionFilteringSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralCollectionSubsystem;
import frc.robot.subsystems.AlgaeCollectionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralDispenserSubsystem;


public class RobotContainer {
  private final String MiddleToH = "Drive from Middle and Sit", driveforward = "Drive forward",
  MiddleToHL1 = "Drive from Middle and Place on H L1", MiddleToHL4 = "Drive from Middle and Place on H L4",
  CageThreeToHL1 = "Drive from Cage 3 and Place on H L1", CageTwoToIL4 = "Drive from Cage 2 and Place on I L4",
  coralcollectionautotest = "Test Coral Collection - TEST FIRST!!!", MiddleToGL1 = "Drive from Middle and Place on G L1",
  MiddleToGL4 = "Drive from Middle and Place on G L4", CageThreeToHL4 = "Drive from Cage 3 and Place on H L4", 
  CageThreeToGL1 = "Drive from Cage 3 and Place on G L1", CageThreeToGL4 = "Drive from Cage 3 and Place on G L4", 
  CageFourToHL1 = "Drive from Cage 4 and Place on H L1", CageFourToHL4 = "Drive from Cage 4 and Place on H L4", 
  CageFourToGL1 = "Drive from Cage 4 and Place on G L1", CageFourToGL4 = "Drive from Cage 4 and Place on G L4",
  CageOneToIL1 = "Drive from Cage 1 and Place on I L1", CageOneToIL4 = "Drive from Cage 1 and Place on I L4",
  CageTwoToIL1 = "Drive from Cage 2 and Place on I L1",
  CageFiveToFL1 = "Drive from Cage 5 and Place on F L1", CageFiveToFL4 = "Drive from Cage 5 and Place on F L4",
  CageSixToFL1 = "Drive from Cage 6 and Place on F L1", CageSixToFL4 = "Drive from Cage 6 and Place on F L4",
  DriveForwardTest = "Drive Forward Test", setOdodmetryToTest = "set Odometry to test", CageFiveToFL4ToCL4 = "Drive from Cage 5 to FL4 CL4";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();

  //Add subsystems below this comment
  private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
  private final PositionFilteringSubsystem positionFilteringSubsystem = new PositionFilteringSubsystem(limeLightSubsystem);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(positionFilteringSubsystem);

  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  CoralCollectionSubsystem coralCollectionSubsystem = new CoralCollectionSubsystem();

  AlgaeCollectionSubsystem algaeCollectionSubsystem = new AlgaeCollectionSubsystem();

  ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  CoralDispenserSubsystem coralDispenserSubsystem = new CoralDispenserSubsystem();


  //Make sure this xbox controller is correct and add driver sticks
  CommandXboxController xboxController = new CommandXboxController(OperatorConstants.kXboxControllerPort);

  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> -leftJoystick.getX(),
      ()-> -leftJoystick.getY(),
      ()-> rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1),
      ()-> rightJoystick.getRawButton(4)
    ));

    
    //m_chooser.addOption(coralcollectionautotest, coralcollectionautotest);
    m_chooser.addOption(MiddleToH, MiddleToH);
    m_chooser.addOption(MiddleToHL1, MiddleToHL1);
    m_chooser.addOption(MiddleToHL4, MiddleToHL4);
    // m_chooser.addOption(CageThreeToHL1, CageThreeToHL1);
    m_chooser.addOption(CageTwoToIL4, CageTwoToIL4);
    // m_chooser.addOption(MiddleToGL1, MiddleToGL1);
    // m_chooser.addOption(MiddleToGL4, MiddleToGL4);
    // m_chooser.addOption(CageThreeToHL4, CageThreeToHL4);
    // m_chooser.addOption(CageThreeToGL1, CageThreeToGL1);
    // m_chooser.addOption(CageThreeToGL4, CageThreeToGL1);
    // m_chooser.addOption(CageFourToHL1, CageFourToHL1);
    // m_chooser.addOption(CageFourToHL4, CageFourToHL4);
    // m_chooser.addOption(CageFourToGL1, CageFourToGL1);
    // m_chooser.addOption(CageFourToGL4, CageFourToGL4); 
    // m_chooser.addOption(CageOneToIL1, CageOneToIL1);
    // m_chooser.addOption(CageOneToIL4, CageOneToIL4);
    // m_chooser.addOption(CageTwoToIL1, CageTwoToIL1); 
    // m_chooser.addOption(CageFiveToFL1, CageFiveToFL1);
    m_chooser.addOption(CageFiveToFL4, CageFiveToFL4); 
    // m_chooser.addOption(CageSixToFL1, CageSixToFL1);
    //m_chooser.addOption(CageSixToFL4, CageSixToFL4);
    //m_chooser.addOption(driveforward, driveforward);
    m_chooser.addOption(DriveForwardTest, DriveForwardTest);
    m_chooser.addOption(setOdodmetryToTest, setOdodmetryToTest);
    m_chooser.addOption(CageFiveToFL4ToCL4, CageFiveToFL4ToCL4);
    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, 
      ()-> -xboxController.getLeftY(),
      ()-> xboxController.y().getAsBoolean(), 
      ()-> xboxController.b().getAsBoolean(), 
      ()-> xboxController.a().getAsBoolean(), 
      ()-> xboxController.x().getAsBoolean()
      ));

    coralDispenserSubsystem.setDefaultCommand(new CoralDispenserCommand(coralDispenserSubsystem, 
    () -> xboxController.getRightTriggerAxis(),
    () -> xboxController.getLeftTriggerAxis()
    ));
     
    algaeCollectionSubsystem.setDefaultCommand(new AlgaeCollectionCommand(algaeCollectionSubsystem, 
    () -> leftJoystick.getRawButton(1),
    () -> leftJoystick.getRawButton(4)
    ));
    
    climbSubsystem.setDefaultCommand(new ClimbCommand(climbSubsystem, 
    () -> xboxController.getRightY(), 
    () -> xboxController.leftBumper().getAsBoolean(), 
    () -> xboxController.rightBumper().getAsBoolean()
    ));

    configureBindings();
  }

  private void configureBindings() {
    xboxController.povUp().onTrue(new InstantCommand(() -> coralCollectionSubsystem.setServoAngle(0)));
    xboxController.povDown().onTrue(new InstantCommand(() -> coralCollectionSubsystem.setServoAngle(180)));
    //Controllers need to be added
    /*
    xboxController.a().onTrue(new AutoPrepForClimbCommand(coralCollectionSubsystem, 30));
    xboxController.b().onTrue(new AutoPrepForClimbCommand(coralCollectionSubsystem, 0));
    */
  }

  public Command getAutonomousCommand() {
    m_autoSelected = m_chooser.getSelected();

    switch (m_autoSelected) {
      case MiddleToH:
        return new SequentialCommandGroup(
          commandSequences.MiddleToH(swerveSubsystem)
        );
      case MiddleToHL1:
        return new SequentialCommandGroup(
          commandSequences.MiddleToH(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case MiddleToHL4:
        return new SequentialCommandGroup(
          commandSequences.MiddleToH(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case CageThreeToHL1:
        return new SequentialCommandGroup(
          commandSequences.CageThreeToH(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case CageThreeToHL4:
        return new SequentialCommandGroup(
          commandSequences.CageThreeToH(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case CageTwoToIL1:
        return new SequentialCommandGroup(
          commandSequences.CageTwoToI(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case CageTwoToIL4:
        return new SequentialCommandGroup(
          commandSequences.CageTwoToI(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case coralcollectionautotest:
        return new SequentialCommandGroup(
          commandSequences.coralcollectionautotest(coralDispenserSubsystem)
        );
      case MiddleToGL1:
        return new SequentialCommandGroup(
          commandSequences.MiddleToG(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case MiddleToGL4:
        return new SequentialCommandGroup(
          commandSequences.MiddleToG(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case CageThreeToGL1:
        return new SequentialCommandGroup(
          commandSequences.CageThreeToG(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case CageThreeToGL4:
        return new SequentialCommandGroup(
          commandSequences.CageThreeToG(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case CageFourToGL1:
        return new SequentialCommandGroup(
          commandSequences.CageFourToG(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case CageFourToGL4:
        return new SequentialCommandGroup(
          commandSequences.CageFourToG(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case CageFourToHL1:
        return new SequentialCommandGroup(
          commandSequences.CageFourToH(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case CageFourToHL4:
        return new SequentialCommandGroup(
          commandSequences.CageFourToH(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case CageOneToIL1:
        return new SequentialCommandGroup(
          commandSequences.CageOneToI(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case CageOneToIL4:
        return new SequentialCommandGroup(
          commandSequences.CageOneToI(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case CageFiveToFL1:
        return new SequentialCommandGroup(
          commandSequences.CageFiveToF(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case CageFiveToFL4:
        return new SequentialCommandGroup(
          commandSequences.CageFiveToF(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case CageSixToFL1:
        return new SequentialCommandGroup(
          commandSequences.CageSixToF(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL1Height)
        );
      case CageSixToFL4:
        return new SequentialCommandGroup(
          commandSequences.CageSixToF(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
        );
      case DriveForwardTest:
        return new SequentialCommandGroup(
          commandSequences.driveForwardTest(swerveSubsystem)
        );
      case setOdodmetryToTest:
        commandSequences.setOdodmeteryToTest(swerveSubsystem);
      case CageFiveToFL4ToCL4:
        return new SequentialCommandGroup(
          commandSequences.CageFiveToF(swerveSubsystem),
          commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height),
          commandSequences.FToRightPlayer(swerveSubsystem),
          commandSequences.collectCoral(coralDispenserSubsystem)
        );
                    
       default :
          commandSequences.driveForward(swerveSubsystem, positionFilteringSubsystem);
    }

    return null;
  }
}
