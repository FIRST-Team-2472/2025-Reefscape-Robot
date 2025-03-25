// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoPrepForClimbCommand;
import frc.robot.commands.defaultCommands.AlgaeCollectionCommand;
import frc.robot.commands.defaultCommands.ClimbCommand;
import frc.robot.commands.defaultCommands.CoralDispenserCommand;

import frc.robot.commands.defaultCommands.ElevatorCommand;
import frc.robot.commands.defaultCommands.SwerveJoystickCmd;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PositionFilteringSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralCollectionSubsystem;
import frc.robot.subsystems.AlgaeCollectionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralDispenserSubsystem;


public class RobotContainer {
  private final String twoCoral = "Swerve Drive to Point Test", drivensitmid = "Drive and Sit - Middle",
  drivenplaceonefrommid = "Drive and place one on G from middle", 
  drivenplaceonefromleft = "Drive and place one on J from Cage 1",
  driveforward = "Drive forward", driveAndPlaceOneFromRight = "Drive and place one on E from Cage 5",
  driveAndPlaceTwoFromLeft = "Drive and place on J and L from cage 1",
  driveAndPlaceTwoFromRight = "Drive and place two on E and C from Cage 1";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();

  //Add subsystems below this comment
  private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
  private final PositionFilteringSubsystem positionFilteringSubsystem = new PositionFilteringSubsystem(limeLightSubsystem);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(positionFilteringSubsystem);
  private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

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
      ()-> -rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1),
      ()-> rightJoystick.getRawButton(4)
    ));

    m_chooser.addOption(twoCoral, twoCoral);
    m_chooser.addOption(drivensitmid, drivensitmid);
    m_chooser.addOption(drivenplaceonefromleft, drivenplaceonefromleft);
    //m_chooser.addOption(driveforward, driveforward);
    m_chooser.addOption(drivenplaceonefrommid, drivenplaceonefrommid);
    m_chooser.addOption(driveAndPlaceOneFromRight, driveAndPlaceOneFromRight);
    m_chooser.addOption(driveAndPlaceTwoFromLeft, driveAndPlaceTwoFromLeft);
    m_chooser.addOption(driveAndPlaceTwoFromRight, driveAndPlaceTwoFromRight);

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

    if(m_autoSelected == twoCoral)
      return new SequentialCommandGroup(
        commandSequences.twoCoralCfourRoneHoneRfive(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem)
      );

      if(m_autoSelected == drivensitmid)
      return new SequentialCommandGroup(
        commandSequences.driveAndSitFromMiddle(swerveSubsystem)
      );

      if(m_autoSelected == drivenplaceonefromleft)
      return new SequentialCommandGroup(
        commandSequences.driveAndPlaceOneFromLeft(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem)
      );

      if(m_autoSelected == driveforward)
      return new SequentialCommandGroup(
        commandSequences.driveForward(swerveSubsystem, positionFilteringSubsystem)
      );

      if(m_autoSelected == drivenplaceonefrommid)
      return new SequentialCommandGroup(
        commandSequences.driveAndPlaceOneFromMiddle(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem)
      );

      if(m_autoSelected == driveAndPlaceOneFromRight)
      return new SequentialCommandGroup(
        commandSequences.driveAndPlaceOneFromRight(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem)
      );

      if(m_autoSelected == driveAndPlaceTwoFromLeft)
      return new SequentialCommandGroup(
        commandSequences.driveAndPlaceTwoFromLeft(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem)
      );

      if(m_autoSelected == driveAndPlaceTwoFromRight)
      return new SequentialCommandGroup(
        commandSequences.driveAndPlaceTwoFromRight(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem)
      );

    return commandSequences.driveForward(swerveSubsystem, positionFilteringSubsystem);
  }
}
