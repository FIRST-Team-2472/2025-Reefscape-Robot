package frc.robot;



public class PID {
    
    private double Kp, Ki,I, Kd, error, lasterror, timeIncrement, time, timeremaining, lastSensorRead, decreaseSpeed, pastPoint;

    public PID(double Kp, double Ki, double Kd, double time, double lastSensorRead){
    this.Kd = Kd;
    this.Ki = Ki;
    this.Kp = Kp;
    this.time = time;
    this.lastSensorRead = lastSensorRead;
    timeremaining = time*50;
    I = 0;
    error = 0;
    timeIncrement = 1/time/50; // 50 = 1/code refresh rate (per second)
}
    public double calculatePID(double setpoint, double sensorRead){
        lasterror = error;
        error = setpoint - sensorRead;
        if (error != 0 && I<1 && I>-1){
            if (error < 0) 
            I += -timeIncrement;
            else 
            I += timeIncrement;
        };
        if (Math.abs((sensorRead - lastSensorRead)*timeremaining) > Math.abs(setpoint)){
            pastPoint = setpoint - ((sensorRead - lastSensorRead)*timeremaining);
            decreaseSpeed = pastPoint/timeremaining;
        } else decreaseSpeed = 0;
        timeremaining -= timeIncrement;
        return Math.min(1, Math.max(-1, (error * Kp) + Ki*I + (decreaseSpeed*Kd)));
    }
}