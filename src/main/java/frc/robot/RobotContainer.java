// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.w3c.dom.ls.LSParserFilter;

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
  private final String MiddleToHL1 = "Drive from Middle and Place on H L1",
      CageThreeToHL1 = "Drive from Cage 3 and Place on H L1", CageTwoToIL4 = "Drive from Cage 2 and Place on I L4",
      MiddleToGL1 = "Drive from Middle and Place on G L1",
      MiddleToGL4 = "Drive from Middle and Place on G L4", CageThreeToHL4 = "Drive from Cage 3 and Place on H L4",
      CageThreeToGL1 = "Drive from Cage 3 and Place on G L1", CageThreeToGL4 = "Drive from Cage 3 and Place on G L4",
      CageFourToHL1 = "Drive from Cage 4 and Place on H L1", CageFourToHL4 = "Drive from Cage 4 and Place on H L4",
      CageFourToGL1 = "Drive from Cage 4 and Place on G L1", CageFourToGL4 = "Drive from Cage 4 and Place on G L4",
      CageOneToIL1 = "Drive from Cage 1 and Place on I L1", CageOneToIL4 = "Drive from Cage 1 and Place on I L4",
      CageTwoToIL1 = "Drive from Cage 2 and Place on I L1",
      CageFiveToFL1 = "Drive from Cage 5 and Place on F L1", CageFiveToFL4 = "Drive from Cage 5 and Place on F L4",
      CageSixToFL1 = "Drive from Cage 6 and Place on F L1", CageSixToFL4 = "Drive from Cage 6 and Place on F L4",
      CageFiveToFL4ToCL4 = "Drive from Cage 5 to FL4 CL4", CageTwoToIL4ToLL4 = "Drive from Cage 2 to IL4 LL4";

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final CommandSequences commandSequences = new CommandSequences();

  // Add subsystems below this comment
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // Add subsystems below this comment

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
        () -> rightJoystick.getRawButton(4),
        () -> leftJoystick.getRawButton(11), // change button
        () -> rightJoystick.getRawButton(11)));

    autoChooser.addOption(MiddleToHL1, MiddleToHL1);
    autoChooser.addOption(CageThreeToHL1, CageThreeToHL1);
    autoChooser.addOption(CageTwoToIL4, CageTwoToIL4);
    autoChooser.addOption(MiddleToGL1, MiddleToGL1);
    autoChooser.addOption(MiddleToGL4, MiddleToGL4);
    autoChooser.addOption(CageThreeToHL4, CageThreeToHL4);
    autoChooser.addOption(CageThreeToGL1, CageThreeToGL1);
    autoChooser.addOption(CageThreeToGL4, CageThreeToGL4);
    autoChooser.addOption(CageFourToHL1, CageFourToHL1);
    autoChooser.addOption(CageFourToHL4, CageFourToHL4);
    autoChooser.addOption(CageFourToGL1, CageFourToGL1);
    autoChooser.addOption(CageFourToGL4, CageFourToGL4);
    autoChooser.addOption(CageOneToIL1, CageOneToIL1);
    autoChooser.addOption(CageOneToIL4, CageOneToIL4);
    autoChooser.addOption(CageTwoToIL1, CageTwoToIL1);
    autoChooser.addOption(CageFiveToFL1, CageFiveToFL1);
    autoChooser.addOption(CageFiveToFL4, CageFiveToFL4);
    autoChooser.addOption(CageSixToFL1, CageSixToFL1);
    autoChooser.addOption(CageSixToFL4, CageSixToFL4);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem,
        () -> -xboxController.getLeftY(),
        () -> xboxController.y().getAsBoolean(),
        () -> xboxController.b().getAsBoolean(),
        () -> xboxController.a().getAsBoolean(),
        () -> xboxController.x().getAsBoolean(),
        () -> leftJoystick.getRawButton(2)));

    coralDispenserSubsystem.setDefaultCommand(new CoralDispenserCommand(coralDispenserSubsystem,
        () -> xboxController.getRightTriggerAxis(),
        () -> xboxController.getLeftTriggerAxis()));
    if (!RobotStatus.kidMode) {
      algaeCollectionSubsystem.setDefaultCommand(new AlgaeCollectionCommand(algaeCollectionSubsystem,
          () -> leftJoystick.getRawButton(1),
          () -> leftJoystick.getRawButton(4),
          () -> leftJoystick.getRawButton(3)));
    }

    /*
     * climbSubsystem.setDefaultCommand(new ClimbCommand(climbSubsystem,
     * () -> xboxController.getRightY(),
     * () -> xboxController.leftBumper().getAsBoolean(),
     * () -> xboxController.rightBumper().getAsBoolean()));
     */
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

    if (m_autoSelected != null) {
      switch (m_autoSelected) {

        case MiddleToHL1:
          return new SequentialCommandGroup(
              commandSequences.MiddleToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case CageTwoToIL4:
          return new SequentialCommandGroup(
              commandSequences.CageTwoToI(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageThreeToGL4:
          return new SequentialCommandGroup(
              commandSequences.CageThreeToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageFiveToFL4:
          return new SequentialCommandGroup(
              commandSequences.CageFiveToF(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageTwoToIL4ToLL4:
          return new SequentialCommandGroup(
              new ParallelDeadlineGroup(
                  commandSequences.CageTwoToI(swerveSubsystem),
                  new SequentialCommandGroup(
                      new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height, .5),
                      new HoldElevatorCommand(elevatorSubsystem))),
              new ParallelDeadlineGroup(
                  new AutoCoralDispenseCommand(coralDispenserSubsystem),
                  new HoldElevatorCommand(elevatorSubsystem)),
              new ParallelCommandGroup(
                  commandSequences.setElevatorL0(elevatorSubsystem),
                  commandSequences.IToLeftPlayer(swerveSubsystem)),
              new ParallelDeadlineGroup(
                  commandSequences.collectCoral(coralDispenserSubsystem),
                  new RunSwerve(swerveSubsystem, 0.1, -0.1)),

              commandSequences.LeftPlayerToL(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));
        case CageFiveToFL4ToCL4:
          return new SequentialCommandGroup(
              new ParallelDeadlineGroup(
                  commandSequences.CageFiveToF(swerveSubsystem),
                  new SequentialCommandGroup(
                      new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height, .5),
                      new HoldElevatorCommand(elevatorSubsystem))),
              new ParallelDeadlineGroup(
                  new AutoCoralDispenseCommand(coralDispenserSubsystem),
                  new HoldElevatorCommand(elevatorSubsystem)),
              new ParallelCommandGroup(
                  commandSequences.setElevatorL0(elevatorSubsystem),
                  commandSequences.FToRightPlayer(swerveSubsystem)),
              new ParallelDeadlineGroup(
                  commandSequences.collectCoral(coralDispenserSubsystem),
                  new RunSwerve(swerveSubsystem, 0.1, 0.1)),

              commandSequences.RightPlayerToD(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageThreeToHL1:
          return new SequentialCommandGroup(
              commandSequences.CageThreeToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case MiddleToGL1:
          return new SequentialCommandGroup(
              commandSequences.MiddleToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case MiddleToGL4:
          return new SequentialCommandGroup(
              commandSequences.MiddleToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageThreeToHL4:
          return new SequentialCommandGroup(
              commandSequences.CageThreeToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageThreeToGL1:
          return new SequentialCommandGroup(
              commandSequences.CageThreeToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case CageFourToHL1:
          return new SequentialCommandGroup(
              commandSequences.CageFourToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case CageFourToHL4:
          return new SequentialCommandGroup(
              commandSequences.CageFourToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageFourToGL1:
          return new SequentialCommandGroup(
              commandSequences.CageFourToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case CageFourToGL4:
          return new SequentialCommandGroup(
              commandSequences.CageFourToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageOneToIL1:
          return new SequentialCommandGroup(
              commandSequences.CageOneToI(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case CageOneToIL4:
          return new SequentialCommandGroup(
              commandSequences.CageOneToI(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        case CageTwoToIL1:
          return new SequentialCommandGroup(
              commandSequences.CageTwoToI(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case CageFiveToFL1:
          return new SequentialCommandGroup(
              commandSequences.CageFiveToF(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case CageSixToFL1:
          return new SequentialCommandGroup(
              commandSequences.CageSixToF(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case CageSixToFL4:
          return new SequentialCommandGroup(
              commandSequences.CageSixToF(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

        default:
          return null;
      }

    }

    return null;
  }
}