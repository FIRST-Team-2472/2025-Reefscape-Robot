package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.SensorStatus;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends SubsystemBase{
    private SparkMax climberMotor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(SensorConstants.kClimberABSEncoderDIOPort);
    public ClimbSubsystem() {
        
      SparkMaxConfig config = new SparkMaxConfig();
          config.smartCurrentLimit(35);
          config.idleMode(IdleMode.kBrake);
      climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      SensorStatus.kClimberAngle = absoluteEncoder.get()*360;// setting it before the pid reads it and multiplying it by 360 to convert it from rotations to degrees
    }

    /**
     * 
     * @param power as -1 to 1 power for the motor positive is up and negative is down
     */
    public void runClimberMotor(double powerPercent){
    climberMotor.set(powerPercent);
    //check that right angle is mounted correctly and if not you may need to change one to be negative
  }
     @Override
  public void periodic() {
    // updating the sensors status to be read by other files
    SensorStatus.kClimberAngle = (absoluteEncoder.get()*360+120) % 360;//multiplying it by 360 to convert it from rotations to degrees
    SmartDashboard.putNumber("Climber angle", SensorStatus.kClimberAngle);
  }
}
