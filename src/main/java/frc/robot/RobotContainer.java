// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
//Add subsystems below this comment


//Make sure this xbox controller is correct and add driver sticks
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kXboxControllerPort);

  public RobotContainer() {
    configureBindings();

    //Stuff for autonomous starts here!

    //ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    //driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    /**
     * Named Commands start here
     * Ex. NamedCommands.registerCommand("runIntake", new IntakeNoteCmd(intakeMotorSubsystem, pitchMotorSubsystem, 0, 8));
     */
  }

  private void configureBindings() {
    //Controllers need to be added
  }

  public Command getAutonomousCommand() {
    //m_autoSelected = m_chooser.getSelected();

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
