package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralDispenserConstants;
import frc.robot.Constants.SensorStatus;
import frc.robot.VL53L4CD;

public class CoralDispenserSubsytem extends SubsystemBase {
  private SparkMax leftMotor =
      new SparkMax(CoralDispenserConstants.kLeftMotorID, MotorType.kBrushless);
  private SparkMax rightMotor =
      new SparkMax(CoralDispenserConstants.kRightMotorID, MotorType.kBrushless);
  VL53L4CD timeOfFlightSensor = new VL53L4CD(I2C.Port.kOnboard);

  public CoralDispenserSubsytem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(35);
    config.idleMode(IdleMode.kBrake);

    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    timeOfFlightSensor.init();
    timeOfFlightSensor.startRanging();
  }

  public void runMotors(double leftPower, double rightPower) {
    leftMotor.set(leftPower);
    rightMotor.set(rightPower);
  }

  public void periodic() {
    double temp = timeOfFlightSensor.measure().distanceMillimeters;
    SensorStatus.kTimeOfFlightDistance = temp;
    SmartDashboard.putNumber("distance sensor", temp);
  }
}
