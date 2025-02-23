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
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PositionFilteringSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralCollectionSubsystem;
import frc.robot.subsystems.AlgaeCollectionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralDispenserSubsystem;


public class RobotContainer {
  private final String CageOneToH = "Drive from Cage One to H", CageOneToI = "Drive from Cage One to I",
  CageTwoToH = "Drive from Cage Two to H", CageTwoToI = "Cage Two to I", CageThreeToH = "Drive from Cage Three to H",
  CageThreeToG = "Drive from Cage Three to G", MiddleToH = "Drive from Middle to H", MiddleToG = "Drive from Middle to G",
  CageFourToG = "Drive from Cage Four to G", CageFourToF = "Drive from Cage Four to F", CageFiveToG = "Drive from Cage Five to G",
  CageFiveToF = "Drive from Cage Five to F", CageSixToG = "Drive from Cage Six to G", CageSixToF = "Drive from Cage Six to F",
  driveforward = "Drive forward";

  private final String PlaceOnL4 = "Place on Level 4", PlaceOnL3 = "Place on Level 3", PlaceOnL2 = "Place on Level 2", PlaceOnL1 = "Place on Level 1";

  private final String HToLeftPlayer = "H to Left Player", GToRightPlayer = "G to Right Player", FToRightPlayer = "F to Right Player", IToRightPlayr = "I to Right Player";

  private final String LeftPlayerToL = "Left Player to L", LeftPlayerToK = "Left Player to K", RightPlayerToC = "Right Player to C", RightPlayerToD = "Right Player to D";

  private String AutoStepOne;
  private String AutoStepTwo;
  private String AutoStepThree;
  private String AutoStepFour;
  private String AutoStepFive;

  private final SendableChooser<String> AutoStepOneChooser = new SendableChooser<>();
  private final SendableChooser<String> AutoStepTwoChooser = new SendableChooser<>();
  private final SendableChooser<String> AutoStepThreeChooser = new SendableChooser<>();
  private final SendableChooser<String> AutoStepFourChooser = new SendableChooser<>();
  private final SendableChooser<String> AutoStepFiveChooser = new SendableChooser<>();

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
      ()-> -rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1),
      ()-> rightJoystick.getRawButton(4)
    ));

    //Step One Options for Autos

    AutoStepOneChooser.addOption(CageOneToH, CageOneToH);
    AutoStepOneChooser.addOption(CageOneToI, CageOneToI);
    AutoStepOneChooser.addOption(CageTwoToH, CageTwoToH);
    AutoStepOneChooser.addOption(CageTwoToI, CageTwoToI);
    AutoStepOneChooser.addOption(CageThreeToH, CageThreeToH);
    AutoStepOneChooser.addOption(CageThreeToG, CageThreeToG);
    AutoStepOneChooser.addOption(MiddleToH, MiddleToH);
    AutoStepOneChooser.addOption(MiddleToG, MiddleToG);
    AutoStepOneChooser.addOption(CageFourToG, CageFourToG);
    AutoStepOneChooser.addOption(CageFourToF, CageFourToF);
    AutoStepOneChooser.addOption(CageFiveToG, CageFiveToG);
    AutoStepOneChooser.addOption(CageFiveToF, CageFiveToF);
    AutoStepOneChooser.addOption(CageSixToG, CageSixToG);
    AutoStepOneChooser.addOption(CageSixToF, CageSixToF);

    //Step Two Options for Autos

    AutoStepTwoChooser.addOption(PlaceOnL4, PlaceOnL4);
    AutoStepTwoChooser.addOption(PlaceOnL3, PlaceOnL3);
    AutoStepTwoChooser.addOption(PlaceOnL2, PlaceOnL2);
    AutoStepTwoChooser.addOption(PlaceOnL1, PlaceOnL1);

    //Step Three Options for Autos

    AutoStepThreeChooser.addOption(HToLeftPlayer, HToLeftPlayer);
    AutoStepThreeChooser.addOption(GToRightPlayer, GToRightPlayer);
    AutoStepThreeChooser.addOption(FToRightPlayer, FToRightPlayer);
    AutoStepThreeChooser.addOption(IToRightPlayr, IToRightPlayr);

    //Step Four Options for Autos

    AutoStepFourChooser.addOption(LeftPlayerToL, LeftPlayerToL);
    AutoStepFourChooser.addOption(LeftPlayerToK, LeftPlayerToK);
    AutoStepFourChooser.addOption(RightPlayerToC, RightPlayerToC);
    AutoStepFourChooser.addOption(RightPlayerToD, RightPlayerToD);

    //Step Five Options for Autos

    AutoStepFiveChooser.addOption(PlaceOnL4, PlaceOnL4);
    AutoStepFiveChooser.addOption(PlaceOnL3, PlaceOnL3);
    AutoStepFiveChooser.addOption(PlaceOnL2, PlaceOnL2);
    AutoStepFiveChooser.addOption(PlaceOnL1, PlaceOnL1);

    //m_chooser.addOption(driveforward, driveforward);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto Step One", AutoStepOneChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Auto Step Two", AutoStepTwoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Auto Step Three", AutoStepThreeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Auto Step Four", AutoStepFourChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Auto Step Five", AutoStepFiveChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

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
    AutoStepOne = AutoStepOneChooser.getSelected();
    AutoStepTwo = AutoStepTwoChooser.getSelected();
    AutoStepThree = AutoStepThreeChooser.getSelected();

    Command autoCommand = null;
    Command autoCommand2 = null;
    Command autoCommand3 = null;
    Command autoCommand4 = null;
    Command autoCommand5 = null;

    switch (AutoStepOne) {
      case CageOneToH:
        autoCommand = commandSequences.CageOneToH(swerveSubsystem);
        break;
      case CageOneToI:
        autoCommand = commandSequences.CageOneToI(swerveSubsystem);
        break;
      case CageTwoToH:
        autoCommand = commandSequences.CageTwoToH(swerveSubsystem);
        break;
      case CageTwoToI:
        autoCommand = commandSequences.CageTwoToI(swerveSubsystem);
        break;
      case CageThreeToH:
        autoCommand = commandSequences.CageThreeToH(swerveSubsystem);
        break;
      case CageThreeToG:
        autoCommand = commandSequences.CageThreeToG(swerveSubsystem);
        break;
      case MiddleToH:
        autoCommand = commandSequences.MiddleToH(swerveSubsystem);
        break;
      case MiddleToG:
        autoCommand = commandSequences.MiddleToG(swerveSubsystem);
        break;
      case CageFourToG:
        autoCommand = commandSequences.CageFourToG(swerveSubsystem);
        break;
      case CageFourToF:
        autoCommand = commandSequences.CageFourToF(swerveSubsystem);
        break;
      case CageFiveToG:
        autoCommand = commandSequences.CageFiveToG(swerveSubsystem);
        break;
      case CageFiveToF:
        autoCommand = commandSequences.CageFiveToF(swerveSubsystem);
        break;
      case CageSixToG:
        autoCommand = commandSequences.CageSixToG(swerveSubsystem);
        break;
      case CageSixToF:
        autoCommand = commandSequences.CageSixToF(swerveSubsystem);
        break;
      default:
        autoCommand = commandSequences.driveForward(swerveSubsystem, positionFilteringSubsystem);
        break;
    }

    switch (AutoStepTwo) {
      case PlaceOnL4:
        autoCommand2 = autoCommand.andThen(commandSequences.PlaceOnL4(elevatorSubsystem, coralDispenserSubsystem));
        break;
      case PlaceOnL3:
        autoCommand2 = autoCommand.andThen(commandSequences.PlaceOnL3(elevatorSubsystem, coralDispenserSubsystem));
        break;
      case PlaceOnL2:
        autoCommand2 = autoCommand.andThen(commandSequences.PlaceOnL2(elevatorSubsystem, coralDispenserSubsystem));
        break;
      case PlaceOnL1:
        autoCommand2 = autoCommand.andThen(commandSequences.PlaceOnL1(elevatorSubsystem, coralDispenserSubsystem));
        break;
      default:
        autoCommand2 = autoCommand.andThen(commandSequences.PlaceOnL1(elevatorSubsystem, coralDispenserSubsystem));
        break;
    }

    switch (AutoStepThree) {
      case HToLeftPlayer:
        autoCommand3 = autoCommand.andThen(autoCommand2).andThen(commandSequences.HToLeftPlayer(swerveSubsystem));
        break;
      case GToRightPlayer:
        autoCommand3 = autoCommand.andThen(autoCommand2).andThen(commandSequences.GToRightPlayer(swerveSubsystem));
        break;
      case FToRightPlayer:
        autoCommand3 = autoCommand.andThen(autoCommand2).andThen(commandSequences.FToRightPlayer(swerveSubsystem));
        break;
      case IToRightPlayr:
        autoCommand3 = autoCommand.andThen(autoCommand2).andThen(commandSequences.IToRightPlayer(swerveSubsystem));
        break;
      default:
        autoCommand3 = autoCommand.andThen(autoCommand2).andThen(commandSequences.IToRightPlayer(swerveSubsystem));
        break;
    }

    switch (AutoStepFour) {
      case LeftPlayerToL:
        autoCommand4 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(commandSequences.LeftPlayerToL(swerveSubsystem));
        break;
      case LeftPlayerToK:
        autoCommand4 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(commandSequences.LeftPlayerToK(swerveSubsystem));
        break;
      case RightPlayerToC:
        autoCommand4 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(commandSequences.RightPlayerToC(swerveSubsystem));
        break;
      case RightPlayerToD:
        autoCommand4 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(commandSequences.RightPlayerToD(swerveSubsystem));
        break;
      default:
        autoCommand4 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(commandSequences.RightPlayerToD(swerveSubsystem));
        break;
    }

    switch (AutoStepFive) {
      case PlaceOnL4:
        autoCommand5 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(autoCommand4).andThen(commandSequences.PlaceOnL4(elevatorSubsystem, coralDispenserSubsystem));
        break;
      case PlaceOnL3:
        autoCommand5 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(autoCommand4).andThen(commandSequences.PlaceOnL3(elevatorSubsystem, coralDispenserSubsystem));
        break;
      case PlaceOnL2:
        autoCommand5 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(autoCommand4).andThen(commandSequences.PlaceOnL2(elevatorSubsystem, coralDispenserSubsystem));
        break;
      case PlaceOnL1:
        autoCommand5 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(autoCommand4).andThen(commandSequences.PlaceOnL1(elevatorSubsystem, coralDispenserSubsystem));
        break;
      default:
        autoCommand5 = autoCommand.andThen(autoCommand2).andThen(autoCommand3).andThen(autoCommand4).andThen(commandSequences.PlaceOnL1(elevatorSubsystem, coralDispenserSubsystem));
        break;
    }

    return null;
  }
}
