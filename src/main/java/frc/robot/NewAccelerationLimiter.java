package frc.robot;

import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class NewAccelerationLimiter {
    private double accelerationChange, lastAcceleration, positiveRateLimit, negativeRateLimit, previousSpeed, previousTime, elapsedTime, currentTime, wantedAcceleration, clampedAcceleration, clampedSpeed;

    public NewAccelerationLimiter(double positiveRateLimit, double negativeRateLimit){
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        previousSpeed = 0;
        lastAcceleration = 0;
        previousTime = WPIUtilJNI.now() * 1e-6;//Convert micro-seconds into seconds
    }
    public double calculate(double MPCwantedSpeed){
        currentTime = WPIUtilJNI.now() * 1e-6;//Convert micro-seconds into seconds
        elapsedTime = currentTime - previousTime;

        wantedAcceleration  = Math.abs(MPCwantedSpeed) - previousSpeed;//Change in speed wanted by the Motor Power Controller
        accelerationChange = wantedAcceleration - lastAcceleration;
        clampedAcceleration = MathUtil.clamp(accelerationChange,negativeRateLimit*elapsedTime, positiveRateLimit*elapsedTime);//Clamped change in speed based on time elapsed
        clampedSpeed = MathUtil.clamp(previousSpeed + clampedAcceleration + lastAcceleration, -1, 1);//changes motor speed to new clamped speed based on acceleration limits
        previousTime = currentTime;
        previousSpeed = clampedSpeed;
        lastAcceleration += accelerationChange;
        clampedSpeed *= MPCwantedSpeed/Math.abs(MPCwantedSpeed);
        return clampedSpeed;
    }
    public void setInitialSpeed(double InitialSpeed){
        previousSpeed = InitialSpeed;
    }
}
