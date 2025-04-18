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
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralCollectionSubsystem;
import frc.robot.subsystems.AlgaeCollectionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.commands.RunSwerve;

public class RobotContainer {
  private final String MiddleToH = "Drive from Middle and Sit", driveforward = "Drive forward",
      MiddleToHL1 = "Drive from Middle and Place on H L1", MiddleToHL4 = "Drive from Middle and Place on H L4",
      CageThreeToHL1 = "Drive from Cage 3 and Place on H L1", CageTwoToIL4 = "Drive from Cage 2 and Place on I L4",
      coralcollectionautotest = "Test Coral Collection - TEST FIRST!!!",
      MiddleToGL1 = "Drive from Middle and Place on G L1",
      MiddleToGL4 = "Drive from Middle and Place on G L4", CageThreeToHL4 = "Drive from Cage 3 and Place on H L4",
      CageThreeToGL1 = "Drive from Cage 3 and Place on G L1", CageThreeToGL4 = "Drive from Cage 3 and Place on G L4",
      CageFourToHL1 = "Drive from Cage 4 and Place on H L1", CageFourToHL4 = "Drive from Cage 4 and Place on H L4",
      CageFourToGL1 = "Drive from Cage 4 and Place on G L1", CageFourToGL4 = "Drive from Cage 4 and Place on G L4",
      CageOneToIL1 = "Drive from Cage 1 and Place on I L1", CageOneToIL4 = "Drive from Cage 1 and Place on I L4",
      CageTwoToIL1 = "Drive from Cage 2 and Place on I L1",
      CageFiveToFL1 = "Drive from Cage 5 and Place on F L1", CageFiveToFL4 = "Drive from Cage 5 and Place on F L4",
      CageSixToFL1 = "Drive from Cage 6 and Place on F L1", CageSixToFL4 = "Drive from Cage 6 and Place on F L4",
      DriveForwardTest = "Drive Forward Test", setOdodmetryToTest = "set Odometry to test",
      CageFiveToFL4ToCL4 = "Drive from Cage 5 to FL4 CL4", CageTwoToIL4ToLL4 = "Drive from Cage 2 to IL4 LL4";

  private String m_autoSelected;
  private String m_testSelected;

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> testChooser = new SendableChooser<>();
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();

  //Add subsystems below this comment
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  CoralCollectionSubsystem coralCollectionSubsystem = new CoralCollectionSubsystem();

  AlgaeCollectionSubsystem algaeCollectionSubsystem = new AlgaeCollectionSubsystem();

  ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  CoralDispenserSubsystem coralDispenserSubsystem = new CoralDispenserSubsystem();

  // Make sure this xbox controller is correct and add driver sticks
  CommandXboxController xboxController = new CommandXboxController(OperatorConstants.kXboxControllerPort);

  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> -leftJoystick.getX(),
        () -> -leftJoystick.getY(),
        () -> rightJoystick.getX(),
        () -> rightJoystick.getRawButton(1),
        () -> rightJoystick.getRawButton(4)));

    testChooser.addOption(coralcollectionautotest, coralcollectionautotest);
    autoChooser.addOption(MiddleToH, MiddleToH);
    autoChooser.addOption(MiddleToHL1, MiddleToHL1);
    autoChooser.addOption(MiddleToHL4, MiddleToHL4);
    testChooser.addOption(CageThreeToHL1, CageThreeToHL1);
    autoChooser.addOption(CageTwoToIL4, CageTwoToIL4);
    testChooser.addOption(MiddleToGL1, MiddleToGL1);
    testChooser.addOption(MiddleToGL4, MiddleToGL4);
    testChooser.addOption(CageThreeToHL4, CageThreeToHL4);
    testChooser.addOption(CageThreeToGL1, CageThreeToGL1);
    testChooser.addOption(CageThreeToGL4, CageThreeToGL4);
    testChooser.addOption(CageFourToHL1, CageFourToHL1);
    testChooser.addOption(CageFourToHL4, CageFourToHL4);
    testChooser.addOption(CageFourToGL1, CageFourToGL1);
    testChooser.addOption(CageFourToGL4, CageFourToGL4);
    testChooser.addOption(CageOneToIL1, CageOneToIL1);
    testChooser.addOption(CageOneToIL4, CageOneToIL4);
    testChooser.addOption(CageTwoToIL1, CageTwoToIL1);
    testChooser.addOption(CageFiveToFL1, CageFiveToFL1);
    autoChooser.addOption(CageFiveToFL4, CageFiveToFL4);
    testChooser.addOption(CageSixToFL1, CageSixToFL1);
    testChooser.addOption(CageSixToFL4, CageSixToFL4);
    autoChooser.addOption(DriveForwardTest, DriveForwardTest);
    testChooser.addOption(setOdodmetryToTest, setOdodmetryToTest);
    autoChooser.addOption(CageFiveToFL4ToCL4, CageFiveToFL4ToCL4);
    autoChooser.addOption(CageTwoToIL4ToLL4, CageTwoToIL4ToLL4);

    if(DriverStation.isFMSAttached() == true) {
    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    } else {
      ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
      driverBoard.add("Auto choices", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

      ShuffleboardTab autoTestingBoard = Shuffleboard.getTab("Auto Testing");
      autoTestingBoard.add("Auto choices", testChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

      ShuffleboardTab autoTestingBoard = Shuffleboard.getTab("Auto Testing");
      autoTestingBoard.add("Auto choices", testChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem,
        () -> -xboxController.getLeftY(),
        () -> xboxController.y().getAsBoolean(),
        () -> xboxController.b().getAsBoolean(),
        () -> xboxController.a().getAsBoolean(),
        () -> xboxController.x().getAsBoolean()));

    coralDispenserSubsystem.setDefaultCommand(new CoralDispenserCommand(coralDispenserSubsystem,
        () -> xboxController.getRightTriggerAxis(),
        () -> xboxController.getLeftTriggerAxis()));

    algaeCollectionSubsystem.setDefaultCommand(new AlgaeCollectionCommand(algaeCollectionSubsystem,
        () -> leftJoystick.getRawButton(1),
        () -> leftJoystick.getRawButton(4)));

    climbSubsystem.setDefaultCommand(new ClimbCommand(climbSubsystem,
        () -> xboxController.getRightY(),
        () -> xboxController.leftBumper().getAsBoolean(),
        () -> xboxController.rightBumper().getAsBoolean()));

    configureBindings();
  }

  private void configureBindings() {
    //xboxController.povUp().onTrue(new InstantCommand(() -> coralCollectionSubsystem.setServoAngle(0)));
    //xboxController.povDown().onTrue(new InstantCommand(() -> coralCollectionSubsystem.setServoAngle(180)));
    //Controllers need to be added
    /*
     * xboxController.a().onTrue(new
     * AutoPrepForClimbCommand(coralCollectionSubsystem, 30));
     * xboxController.b().onTrue(new
     * AutoPrepForClimbCommand(coralCollectionSubsystem, 0));
     */
  }

  public Command getAutonomousCommand() {
    m_autoSelected = autoChooser.getSelected();
    m_testSelected = testChooser.getSelected();

    if (m_autoSelected != null) {
      switch (m_autoSelected) {
        case MiddleToH:
          return new SequentialCommandGroup(
              commandSequences.MiddleToH(swerveSubsystem));

        case MiddleToHL1:
          return new SequentialCommandGroup(
              commandSequences.MiddleToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL1Height));

        case MiddleToHL4:
          return new SequentialCommandGroup(
              commandSequences.MiddleToH(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

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

        case DriveForwardTest:
          return new SequentialCommandGroup(
              commandSequences.driveForwardTest(swerveSubsystem));

        case CageTwoToIL4ToLL4:
              return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                  commandSequences.CageTwoToI(swerveSubsystem),
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
                commandSequences.collectCoral(coralDispenserSubsystem),
    
                commandSequences.LeftPlayerToL(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
              );
        case CageFiveToFL4ToCL4:
              return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                  commandSequences.CageFiveToF(swerveSubsystem),
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
                commandSequences.collectCoral(coralDispenserSubsystem),
    
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

        case CageThreeToGL4:
          return new SequentialCommandGroup(
              commandSequences.CageThreeToG(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height));

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

        case setOdodmetryToTest:
          commandSequences.setOdodmeteryToTest(swerveSubsystem);

        case CageFiveToFL4ToCL4:
          return new SequentialCommandGroup(
            new SequentialCommandGroup(
                  commandSequences.CageFiveToF(swerveSubsystem),
                  commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem, ElevatorConstants.kElevatorL4Height)
            ),
            new ParallelCommandGroup(
                  commandSequences.setElevatorL0(elevatorSubsystem),
                  commandSequences.FToRightPlayer(swerveSubsystem)
            ),
                
              commandSequences.collectCoral(coralDispenserSubsystem),

              commandSequences.RightPlayerToD(swerveSubsystem),
              commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                  ElevatorConstants.kElevatorL4Height)
          );
      }
    }

    return null;
  }
}
