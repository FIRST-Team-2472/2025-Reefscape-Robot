package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorPowerController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.RobotStatus;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeCollectionSubsystem extends SubsystemBase {
  public SparkMax pivotmotor = new SparkMax(AlgaeConstants.kPivotMotorID, MotorType.kBrushless);
  public SparkMax spinmotor = new SparkMax(AlgaeConstants.kSpinMotorID, MotorType.kBrushless);

  private MotorPowerController angleController = new MotorPowerController(.003, .05, .1, .5, 2, 120, 5);

  private double pivotAngleSetPoint = 120;

  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(SensorConstants.kAlgeaABSEncoderDIOPort);

  public AlgaeCollectionSubsystem() {

    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig config2 = new SparkMaxConfig();
    config.smartCurrentLimit(35);
    config.idleMode(IdleMode.kBrake);
    pivotmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config2.smartCurrentLimit(7);
    config2.idleMode(IdleMode.kCoast);
    spinmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    RobotStatus.kPivotAngle = absoluteEncoder.get() * 360;// updating it before its read, converting it to degrees as
                                                           // well
  }

  public void runPivotMotor(double powerPercent) {
    if(RobotStatus.kPivotAngle > 200)
      Math.max(0, powerPercent);// clamps it so it cant drive down when beyond this angle
    else if(RobotStatus.kPivotAngle < 115)
      Math.min(0, powerPercent);// clamps it so it cant drive up when beyond this angle
    pivotmotor.set(powerPercent);
  }

  public void runSpinMotor(double powerPercent) {
    spinmotor.set(powerPercent);
  }

  public void setAngleSetpoint(double angle) {
    angle = Math.min(220, Math.max(105, angle)); // clamp betweein vertical and on the ground
    pivotAngleSetPoint = angle;
  }

  @Override
  public void periodic() {
    // updating the sensors status to be read by other files
    RobotStatus.kPivotAngle = (absoluteEncoder.get() * 360 + 180) % 360;// converting it to degrees and offsetting it
                                                                         // by 180
    SmartDashboard.putNumber("Algea Collector angle", RobotStatus.kPivotAngle);
    SmartDashboard.putNumber("Spin motor output", spinmotor.getOutputCurrent());

    // driving it to hold its angle
    runPivotMotor(-angleController.calculate(pivotAngleSetPoint, RobotStatus.kPivotAngle));
  }
}