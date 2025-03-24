package frc.robot;

import java.util.HashMap;

public class MPCConfig { 
    public HashMap<String, MPCConfigValues> mpcValues = new HashMap<String, MPCConfigValues>(); 
    public MPCConfig(){
        mpcValues.put("AlgaeMPCValues", new MPCConfigValues(0.003, 0.05, 0.1, 0.5, 2.0, 120.0, 5.0));
        mpcValues.put("ElevatorMPCValues", new MPCConfigValues(0.07, 0.05, 0.2, 1.0, 1.0, SensorStatus.kElevatorHeight, 5.0));
        mpcValues.put("SwerveSpeedMPCValues", new MPCConfigValues(0.15, 0.0, 0.005, 1.0, 0.2, 0.0, 1.0));
        mpcValues.put("SwerveTurningdMPCValues", new MPCConfigValues(0.15, 0.02, 0.7, 1.0, 0.1, 0.0, 1.0));
    }

    public MPCConfigValues getMPCValues(String key){
        return mpcValues.get(key);
    }
    public class MPCConfigValues{
        double Kp, Ki, dTime, time, allowedError, lastSensorRead, integralProportionalThreshold;
        public MPCConfigValues(double Kp, double Ki, double dTime, double time, double allowedError, double lastSensorRead, double integralProportionalThreshold){
            this.Kp = Kp;
            this.Ki = Ki;
            this.dTime = dTime;
            this.time = time;
            this.allowedError = allowedError;
            this.lastSensorRead = lastSensorRead;
            this.integralProportionalThreshold = integralProportionalThreshold;
        }
    }
}