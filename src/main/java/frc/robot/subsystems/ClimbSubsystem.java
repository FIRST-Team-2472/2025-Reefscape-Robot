package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ClimberConstants;

public class ClimbSubsystem {
    private SparkMax climberMotor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
}
