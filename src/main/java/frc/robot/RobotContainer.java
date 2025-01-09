// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
  private final String ReefSomething = "Reef to _";


  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

//Add subsystems below this comment

//I made this comment here for demonstration purposes!

//Make sure this xbox controller is correct and add driver sticks
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    //Stuff for autonomous starts here!
    m_chooser.addOption(ReefSomething, ReefSomething); //Adds the option to the sendable chooser on shuffleboard

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    /**
     * Named Commands start here
     * Ex. NamedCommands.registerCommand("runIntake", new IntakeNoteCmd(intakeMotorSubsystem, pitchMotorSubsystem, 0, 8));
     */

    //When swerve subsystem gets imported this should fix itself
     AutoBuilder.configureHolonomic(
      () -> swerveSubsystem.getPose(), // Robot pose supplier for auto (correct range -180-180)
      swerveSubsystem ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      () -> swerveSubsystem.getChassisSpeedsRobotRelative(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      swerveSubsystem :: runModulesRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
      () -> SwerveSubsystem.isOnRed(),
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      swerveSubsystem // Reference to this subsystem to set requirements
  );
  }

  private void configureBindings() {
    //Controllers need to be added
  }

  public Command getAutonomousCommand() {
    m_autoSelected = m_chooser.getSelected();

    //Once autos are all set up add the following
    /**
     * if(m_autoSelected == NAME SPECIFIED IN THE CHOOSER ABOVE)
        return AutoBuilder.buildAuto("NAME OF AUTO IN PATHPLANNER");
     */

    /*If we decide on an auto that should run by default delete return null and make something for if nothing 
    is selected just run our default auto*/
    return null;
  }
}
