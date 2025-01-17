// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  private final String RCthreeRone = "RCthreeRone";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  RobotConfig config;

//Add subsystems below this comment
private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

//Make sure this xbox controller is correct and add driver sticks
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kXboxControllerPort);

  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> leftJoystick.getX(),
      ()-> leftJoystick.getY(),
      ()-> rightJoystick.getX()
    ));

    configureBindings();

    m_chooser.addOption(RCthreeRone, RCthreeRone);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    if(config != null){
      AutoBuilder.configure(
              () -> swerveSubsystem.getPose(), // Robot pose supplier for auto (correct range -180-180)
              swerveSubsystem ::setOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              () -> swerveSubsystem.getChassisSpeedsRobotRelative(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              swerveSubsystem :: runModulesRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
              AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
              config,
              () -> SwerveSubsystem.isOnRed(),
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              swerveSubsystem // Reference to this subsystem to set requirements
          );
    }
  }

  private void configureBindings() {
    //Controllers need to be added
  }

  public Command getAutonomousCommand() {
    m_autoSelected = m_chooser.getSelected();

    if(m_autoSelected == RCthreeRone)
        return AutoBuilder.buildAuto("test");
    return null;
  }
}
