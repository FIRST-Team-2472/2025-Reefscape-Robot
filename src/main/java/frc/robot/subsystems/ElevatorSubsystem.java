// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorStatus;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
  
  private SparkMax leftElevatorMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorID, MotorType.kBrushless);
  private SparkMax rightElevatorMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorID, MotorType.kBrushless);

  public double lastLeftElevatorReading = 0;
  public double lastRightElevatorReading = 0;

  public ElevatorSubsystem() {

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(35);
    config.idleMode(IdleMode.kBrake);

    leftElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  public void runElevatorMotors(double powerPercent){
    leftElevatorMotor.set(powerPercent);
    rightElevatorMotor.set(powerPercent);

  }

  @Override
  public void periodic() {
    // updating the sensors status to be read by other files

    // code to check that both motors are working and returning the other motors value if one isnt
    if (leftElevatorMotor.getEncoder().getPosition() != lastLeftElevatorReading) {
      SensorStatus.kElevatorHeight = leftElevatorMotor.getEncoder().getPosition() * ElevatorConstants.kElevatorMotorRotationsToInches;
    } else {
      SensorStatus.kElevatorHeight = rightElevatorMotor.getEncoder().getPosition() * ElevatorConstants.kElevatorMotorRotationsToInches;
    }

    lastLeftElevatorReading = leftElevatorMotor.getEncoder().getPosition();
    lastRightElevatorReading = rightElevatorMotor.getEncoder().getPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
