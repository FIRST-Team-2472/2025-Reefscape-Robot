package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralDispenserConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.VL53L4CD;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralDispenserSubsytem extends SubsystemBase{
    private SparkMax leftMotor = new SparkMax(CoralDispenserConstants.kLeftMotorID, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(CoralDispenserConstants.kRightMotorID, MotorType.kBrushless);
    VL53L4CD timeOfFlightSensor = new VL53L4CD(I2C.Port.kOnboard);
    
    public CoralDispenserSubsytem(){

        SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(35);
            config.idleMode(IdleMode.kBrake);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        timeOfFlightSensor.init();
        timeOfFlightSensor.startRanging();
    }

    public void periodic() {
        
        double temp = timeOfFlightSensor.measure().distanceMillimeters;
        SensorStatus.kTimeOfFlightDistance = temp;
        SmartDashboard.putNumber("distance sensor", temp);
    }

}
