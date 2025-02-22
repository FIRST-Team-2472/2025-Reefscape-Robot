package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class NewAccelerationLimiter {
    private double positiveRateLimit, negativeRateLimit, previousSpeed, previousTime, elapsedTime, currentTime, wantedAcceleration, clampedAcceleration, clampedSpeed;

    public NewAccelerationLimiter(double positiveRateLimit, double negativeRateLimit){
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        previousSpeed = 0;
        previousTime = WPIUtilJNI.now() * 1e-6;//Convert micro-seconds into seconds
    }
    public double calculate(double MPCwantedSpeed){
        currentTime = WPIUtilJNI.now() * 1e-6;//Convert micro-seconds into seconds
        elapsedTime = currentTime - previousTime;

        wantedAcceleration  = MPCwantedSpeed - previousSpeed;//Change in speed wanted by the Motor Power Controller
        clampedAcceleration = MathUtil.clamp(wantedAcceleration,negativeRateLimit*elapsedTime, positiveRateLimit*elapsedTime);//Clamped change in speed based on time elapsed
        clampedSpeed = MathUtil.clamp(previousSpeed + clampedAcceleration, -1, 1);//changes motor speed to new clamped speed based on acceleration limits
        previousTime = currentTime;
        previousSpeed = clampedSpeed;
        return clampedSpeed;
    }
    public void setInitialSpeed(double InitialSpeed){
        previousSpeed = InitialSpeed;
    }
}
