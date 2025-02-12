package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralDispenserConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralDispenserSubsystem extends SubsystemBase{
    private SparkMax leftMotor = new SparkMax(CoralDispenserConstants.kLeftMotorID, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(CoralDispenserConstants.kRightMotorID, MotorType.kBrushless);
    //VL53L4CD timeOfFlightSensor = new VL53L4CD(I2C.Port.kOnboard);
    
    public CoralDispenserSubsystem(){

        SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(35);
            config.idleMode(IdleMode.kBrake);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //timeOfFlightSensor.init();
        //timeOfFlightSensor.startRanging();
    }
    public void runMotors(double leftPower, double rightPower){
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    @Override
    public void periodic() {
        
        //double temp = timeOfFlightSensor.measure().distanceMillimeters;
        //SensorStatus.kTimeOfFlightDistance = temp;
        ///SmartDashboard.putNumber("distance sensor", temp);
    }

}
