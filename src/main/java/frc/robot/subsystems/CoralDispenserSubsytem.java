package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralDispenserConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SensorStatus;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralDispenserSubsytem extends SubsystemBase{
    private SparkMax leftMotor = new SparkMax(CoralDispenserConstants.kLeftMotorID, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(CoralDispenserConstants.kRightMotorID, MotorType.kBrushless);
    private I2C timeOfFlightSensor = new I2C(Port.kOnboard, 0);
    private byte[] dataArray = new byte[32];
    
    public CoralDispenserSubsytem(){

        SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(35);
            config.idleMode(IdleMode.kBrake);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {
        boolean sensorRead = timeOfFlightSensor.read(0, 32, dataArray);
        if(!sensorRead){
            System.out.println("Time of flight sensor failed to read");
        }
        SensorStatus.kTimeOfFlightDistance = convertByteArrayToDouble(dataArray);
    }
    public double convertByteArrayToDouble(byte[] array){
        return 0.1;// put logic here
    }
}
