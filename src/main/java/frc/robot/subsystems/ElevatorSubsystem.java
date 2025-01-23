// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {
  
  private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.kElevatorMotorID, MotorType.kBrushless);

  private DigitalInput LeftLimitSwitch = new DigitalInput(SensorConstants.kLeftLimitSwitchID);
  private DigitalInput MiddleLimitSwitch = new DigitalInput(SensorConstants.kMiddleLimitSwitchID);
  private DigitalInput RightLimitSwitch = new DigitalInput(SensorConstants.kRightLimitSwitchID);

  public ElevatorSubsystem() {

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(35);
    config.idleMode(IdleMode.kBrake);

    elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void runElevatorMotor(double powerPercent){
    elevatorMotor.set(powerPercent);
  }

  @Override
  public void periodic() {
    // updating the sensors status to be read by other files
    SensorStatus.kLeftLimitSwitchValue = LeftLimitSwitch.get();
    SensorStatus.kMiddleLimitSwitchValue = MiddleLimitSwitch.get();
    SensorStatus.kRightLimitSwitchValue = RightLimitSwitch.get();
    SensorStatus.kElevatorHeight = elevatorMotor.getEncoder().getPosition()*ElevatorConstants.kElevatorGearRatio*ElevatorConstants.kSprocketCircumference*2;// the times two is bc its 2 stage
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
