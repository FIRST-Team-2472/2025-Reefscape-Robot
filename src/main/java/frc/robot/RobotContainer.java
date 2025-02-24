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
  private final String driveforward = "Drive forward";

  private final String CageOne = "Cage One", CageTwo = "Cage Two", CageThree = "Cage Three", MiddleBarge = "Middle of Barge", CageFour = "Cage Four", CageFive = "Cage Five", CageSix = "Cage Six";
  private final String A = "A", B = "B", C = "C", D = "D", E = "E", F = "F", G = "G", H = "H", I = "I", J = "J", K = "K", L = "L";
  private final String One = "1", OneOneHalf = "1.5", Two = "2", TwoOneHalf = "2.5", Three = "3";

  private final String L4 = "L4", L3 = "L3", L2 = "L2", L1 = "L1";
  private final String LeftCoralStation = "Left Coral Station", RightCoralStation = "Right Coral Station";

  private final String HToLeftPlayer = "H to Left Player", GToRightPlayer = "G to Right Player", FToRightPlayer = "F to Right Player", IToRightPlayr = "I to Right Player";

  private final String LeftPlayerToL = "Left Player to L", LeftPlayerToK = "Left Player to K", RightPlayerToC = "Right Player to C", RightPlayerToD = "Right Player to D";

  private String CoralAmmount;
  private String BeginingPosition;
  private String BranchPosition1;
  private String BranchLevel1;
  private String HumanStation1;
  private String BranchPosition2;
  private String BranchLevel2;
  private String HumanStation2;
  private String BranchPosition3;
  private String BranchLevel3;


  private final SendableChooser<String> HowManyCoral = new SendableChooser<>();
  private final SendableChooser<String> RobotPosition = new SendableChooser<>();
  private final SendableChooser<String> ReefPosition1 = new SendableChooser<>();
  private final SendableChooser<String> ReefLevel1 = new SendableChooser<>();
  private final SendableChooser<String> CoralStation1 = new SendableChooser<>();
  private final SendableChooser<String> ReefPosition2 = new SendableChooser<>();
  private final SendableChooser<String> ReefLevel2 = new SendableChooser<>();
  private final SendableChooser<String> CoralStation2 = new SendableChooser<>();
  private final SendableChooser<String> ReefPosition3 = new SendableChooser<>();
  private final SendableChooser<String> ReefLevel3 = new SendableChooser<>();

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

    HowManyCoral.addOption(One, One);
    HowManyCoral.addOption(OneOneHalf, OneOneHalf);
    HowManyCoral.addOption(Two, Two);
    HowManyCoral.addOption(TwoOneHalf, TwoOneHalf);
    HowManyCoral.addOption(Three, Three);
    
    RobotPosition.addOption(CageOne, CageOne);
    RobotPosition.addOption(CageTwo, CageTwo);
    RobotPosition.addOption(CageThree, CageThree);
    RobotPosition.addOption(MiddleBarge, MiddleBarge);
    RobotPosition.addOption(CageFour, CageFour);
    RobotPosition.addOption(CageFive, CageFive);
    RobotPosition.addOption(CageSix, CageSix);

    ReefPosition1.addOption(A, A);
    ReefPosition1.addOption(B, B);
    ReefPosition1.addOption(C, C);
    ReefPosition1.addOption(D, D);
    ReefPosition1.addOption(E, E);
    ReefPosition1.addOption(F, F);
    ReefPosition1.addOption(G, G);
    ReefPosition1.addOption(H, H);
    ReefPosition1.addOption(I, I);
    ReefPosition1.addOption(J, J);
    ReefPosition1.addOption(K, K);
    ReefPosition1.addOption(L, L);

    ReefLevel1.addOption(L4, L4);
    ReefLevel1.addOption(L3, L3);
    ReefLevel1.addOption(L2, L2);
    ReefLevel1.addOption(L1, L1);

    CoralStation1.addOption(LeftCoralStation, LeftCoralStation);
    CoralStation1.addOption(RightCoralStation, RightCoralStation);

    ReefPosition2.addOption(A, A);
    ReefPosition2.addOption(B, B);
    ReefPosition2.addOption(C, C);
    ReefPosition2.addOption(D, D);
    ReefPosition2.addOption(E, E);
    ReefPosition2.addOption(F, F);
    ReefPosition2.addOption(G, G);
    ReefPosition2.addOption(H, H);
    ReefPosition2.addOption(I, I);
    ReefPosition2.addOption(J, J);
    ReefPosition2.addOption(K, K);
    ReefPosition2.addOption(L, L);
    
    ReefLevel2.addOption(L4, L4);
    ReefLevel2.addOption(L3, L3);
    ReefLevel2.addOption(L2, L2);
    ReefLevel2.addOption(L1, L1);

    CoralStation2.addOption(LeftCoralStation, LeftCoralStation);
    CoralStation2.addOption(RightCoralStation, RightCoralStation);

    ReefPosition3.addOption(A, A);
    ReefPosition3.addOption(B, B);
    ReefPosition3.addOption(C, C);
    ReefPosition3.addOption(D, D);
    ReefPosition3.addOption(E, E);
    ReefPosition3.addOption(F, F);
    ReefPosition3.addOption(G, G);
    ReefPosition3.addOption(H, H);
    ReefPosition3.addOption(I, I);
    ReefPosition3.addOption(J, J);
    ReefPosition3.addOption(K, K);
    ReefPosition3.addOption(L, L);

    ReefLevel3.addOption(L4, L4);
    ReefLevel3.addOption(L3, L3);
    ReefLevel3.addOption(L2, L2);
    ReefLevel3.addOption(L1, L1);

    //m_chooser.addOption(driveforward, driveforward);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("How Many Coral?", HowManyCoral).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Robot Starting Position", RobotPosition).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Reef Position 1", ReefPosition1).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Reef Level 1", ReefLevel1).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Coral Station 1", CoralStation1).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Reef Position 2", ReefPosition2).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Reef Level 2", ReefLevel2).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Coral Station 2", CoralStation2).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Reef Position 3", ReefPosition3).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.add("Reef Level 3", ReefLevel3).withWidget(BuiltInWidgets.kComboBoxChooser);

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
    CoralAmmount = HowManyCoral.getSelected();
    BeginingPosition = RobotPosition.getSelected();
    BranchPosition1 = ReefPosition1.getSelected();
    BranchLevel1 = ReefLevel1.getSelected();
    HumanStation1 = CoralStation1.getSelected();
    BranchPosition2 = ReefPosition2.getSelected();
    BranchLevel2 = ReefLevel2.getSelected();
    HumanStation2 = CoralStation2.getSelected();
    BranchPosition3 = ReefPosition3.getSelected();
    BranchLevel3 = ReefLevel3.getSelected();

    double numCoral = 0;
    int startPos = 12;
    char reefPos1 = 'z';
    double reefHeight1 = 0.0;
    int coralStation1 = 5;
    char reefPos2 = 'z';
    double reefHeight2 = 0.0;
    int coralStation2 = 5;
    char reefPos3 = 'z';
    double reefHeight3 = 0.0;

    switch (CoralAmmount) {
      case One:
        numCoral = 1;
        break;
      case OneOneHalf:
        numCoral = 1.5;
        break;
      case Two:
        numCoral = 2;
        break;
      case TwoOneHalf:
        numCoral = 2.5;
        break;
      case Three:
        numCoral = 3;
        break;
      default:
        numCoral = 0;
        break;
    }

    switch (BeginingPosition) {
      case CageOne:
        startPos = 0;
        break;
      case CageTwo:
        startPos = 1;
        break;
      case CageThree:
        startPos = 2;
        break;
      case MiddleBarge:
        startPos = 3;
        break;
      case CageFour:
        startPos = 4;
        break;
      case CageFive:
        startPos = 5;
        break;
      case CageSix:
        startPos = 6;
        break;
      default:
        startPos = 12;
        break;
    }

    switch (BranchPosition1) {
      case A:
        reefPos1 = 'A';
        break;
      case B:
        reefPos1 = 'B';
        break;
      case C:
        reefPos1 = 'C';
        break;
      case D:
        reefPos1 = 'D';
        break;
      case E:
        reefPos1 = 'E';
        break;
      case F:
        reefPos1 = 'F';
        break;
      case G:
        reefPos1 = 'G';
        break;
      case H:
        reefPos1 = 'H';
        break;
      case I:
        reefPos1 = 'I';
        break;
      case J:
        reefPos1 = 'J';
        break;
      case K:
        reefPos1 = 'K';
        break;
      case L:
        reefPos1 = 'L';
        break;
      default:
        reefPos1 = 'z';
        break;
    }

    switch (BranchLevel1) {
      case L4:
        reefHeight1 = ElevatorConstants.kElevatorL4Height;
        break;
      case L3:
        reefHeight1 = ElevatorConstants.kElevatorL3Height;
        break;
      case L2:
        reefHeight1 = ElevatorConstants.kElevatorL2Height;
        break;
      case L1:
        reefHeight1 = ElevatorConstants.kElevatorL1Height;
        break;
      default:
        reefHeight1 = 0;
        break;
    }

    switch (HumanStation1) {
      case LeftCoralStation:
        coralStation1 = 0;
        break;
      case RightCoralStation:
        coralStation1 = 1;
        break;
      default:
        coralStation1 = 5;
        break;
    }

    switch (BranchPosition2) {
      case A:
        reefPos2 = 'A';
        break;
      case B:
        reefPos2 = 'B';
        break;
      case C:
        reefPos2 = 'C';
        break;
      case D:
        reefPos2 = 'D';
        break;
      case E:
        reefPos2 = 'E';
        break;
      case F:
        reefPos2 = 'F';
        break;
      case G:
        reefPos2 = 'G';
        break;
      case H:
        reefPos2 = 'H';
        break;
      case I:
        reefPos2 = 'I';
        break;
      case J:
        reefPos2 = 'J';
        break;
      case K:
        reefPos2 = 'K';
        break;
      case L:
        reefPos2 = 'L';
        break;
      default:
        reefPos2 = 'z';
        break;
    }

    switch (BranchLevel2) {
      case L4:
        reefHeight2 = ElevatorConstants.kElevatorL4Height;
        break;
      case L3:
        reefHeight2 = ElevatorConstants.kElevatorL3Height;
        break;
      case L2:
        reefHeight2 = ElevatorConstants.kElevatorL2Height;
        break;
      case L1:
        reefHeight2 = ElevatorConstants.kElevatorL1Height;
        break;
      default:
        reefHeight2 = 0;
        break;
    }

    switch (HumanStation2) {
      case LeftCoralStation:
        coralStation2 = 0;
        break;
      case RightCoralStation:
        coralStation2 = 1;
        break;
      default:
        coralStation2 = 5;
        break;
    }

    switch (BranchPosition3) {
      case A:
        reefPos3 = 'A';
        break;
      case B:
        reefPos3 = 'B';
        break;
      case C:
        reefPos3 = 'C';
        break;
      case D:
        reefPos3 = 'D';
        break;
      case E:
        reefPos3 = 'E';
        break;
      case F:
        reefPos3 = 'F';
        break;
      case G:
        reefPos3 = 'G';
        break;
      case H:
        reefPos3 = 'H';
        break;
      case I:
        reefPos3 = 'I';
        break;
      case J:
        reefPos3 = 'J';
        break;
      case K:
        reefPos3 = 'K';
        break;
      case L:
        reefPos3 = 'L';
        break;
      default:
        reefPos3 = 'z';
        break;
    }

    switch (BranchLevel3) {
      case L4:
        reefHeight3 = ElevatorConstants.kElevatorL4Height;
        break;
      case L3:
        reefHeight3 = ElevatorConstants.kElevatorL3Height;
        break;
      case L2:
        reefHeight3 = ElevatorConstants.kElevatorL2Height;
        break;
      case L1:
        reefHeight3 = ElevatorConstants.kElevatorL1Height;
        break;
      default:
        reefHeight3 = 0;
        break;
    }
    if (numCoral == 1) {
      return commandSequences.driverSelectedAuto(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem, startPos, reefPos1, reefHeight1);
    } else if (numCoral == 1.5) {
      return commandSequences.driverSelectedAuto(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem, startPos, reefPos1, reefHeight1, coralStation1);
    } else if (numCoral == 2) {
      return commandSequences.driverSelectedAuto(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem, startPos, reefPos1, reefHeight1, coralStation1, reefPos2, reefHeight2);
    } else if (numCoral == 2.5) {
      return commandSequences.driverSelectedAuto(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem, startPos, reefPos1, reefHeight1, coralStation1, reefPos2, reefHeight2, coralStation2);
    } else if (numCoral == 3) {
      return commandSequences.driverSelectedAuto(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem, startPos, reefPos1, reefHeight1, coralStation1, reefPos2, reefHeight2, coralStation2, reefPos3, reefHeight3);
    } else {
      return commandSequences.driveForward(swerveSubsystem, positionFilteringSubsystem);
    }
  }
}
