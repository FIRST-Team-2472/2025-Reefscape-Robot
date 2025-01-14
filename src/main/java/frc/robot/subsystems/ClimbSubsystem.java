package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SensorStatus;

public class ClimbSubsystem extends SubsystemBase{
    private SparkMax climberMotor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);

    public ClimbSubsystem() {
        
        SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(35);
            config.idleMode(IdleMode.kBrake);
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void runClimberMotor(double powerPercent){
    climberMotor.set(powerPercent);
  }
     @Override
  public void periodic() {
    // updating the sensors status to be read by other files
    SensorStatus.kClimberAngle = climberMotor.getEncoder().getPosition()*ClimberConstants.kClimberGearRatio;
  }
}
