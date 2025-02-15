package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeCollectionSubsystem extends SubsystemBase {
  // creating the motors.
  private SparkMax pivotmotor = new SparkMax(AlgaeConstants.kPivotMotorID, MotorType.kBrushless);
  private SparkMax spinmotor = new SparkMax(AlgaeConstants.kSpinMotorID, MotorType.kBrushless);

  // duty cycle encoders are absolute encoders that send pulses that are
  // proportional to the current position of the encoder.
  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(SensorConstants.kAlgeaABSEncoderDIOPort);

  public AlgaeCollectionSubsystem() {
    //configuring motors
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(35);
    config.idleMode(IdleMode.kBrake);
    pivotmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spinmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SensorStatus.kPivotAngle = absoluteEncoder.get() * 360;// updating it before its read,
                                                           // converting it to degrees as well
  }

  public void runPivotMotor(double powerPercent) {
    pivotmotor.set(powerPercent);
  }

  public void runSpinMotor(double powerPercent) {
    pivotmotor.set(powerPercent);
  }

  @Override
  public void periodic() {
    // updating the sensors status to be read by other files
    SensorStatus.kPivotAngle = absoluteEncoder.get() * 360;// converting it to degrees
  }
}
