package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralDispenserConstants;
import frc.robot.SensorStatus;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class CoralDispenserSubsystem extends SubsystemBase{
    private SparkMax leftMotor = new SparkMax(CoralDispenserConstants.kLeftMotorID, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(CoralDispenserConstants.kRightMotorID, MotorType.kBrushless);
    private LaserCan laserCan = new LaserCan(0);
    int fails = 0;
    public boolean seeCoral, hasCoral = false;
    
    public CoralDispenserSubsystem(){

        SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(10);// max they can handle over a extended period of time before melting
            config.idleMode(IdleMode.kBrake);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        }catch (ConfigurationFailedException e) {
            //System.out.println("Configuration failed" + e);
        }
    }
    public void runMotors(double leftPower, double rightPower){
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    public void autoIntake() {
        if (fails < 7) {
            if (seeCoral && SensorStatus.kTimeOfFlightDistance > 80) {
                hasCoral = true;
                seeCoral = false;
            }
            if (SensorStatus.kTimeOfFlightDistance < 80) {
                seeCoral = true;
            }
        } else {
            hasCoral = false;
            seeCoral = false;
            System.out.println("fail");
        }

        if (hasCoral && seeCoral) {
            hasCoral = false;
        }
    }

    @Override
    public void periodic() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if (measurement != null && measurement.status ==LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            double distance = measurement.distance_mm;
            SmartDashboard.putNumber("distance sensor", distance);
            SensorStatus.kTimeOfFlightDistance = distance;
            if (fails > 0) {
                fails--;
            }
        }else if(measurement != null){
            //do nothing
        }else{
            fails++;
            //System.out.println("Oh no! The target is not in range, or we can't get a reliable measurement");
        }
        autoIntake();
        SmartDashboard.putBoolean("seeCoral", seeCoral);
        SmartDashboard.putBoolean("hasCoral", hasCoral);
        SensorStatus.seeCoral = seeCoral;
        SensorStatus.hasCoral = hasCoral;
    }

}
