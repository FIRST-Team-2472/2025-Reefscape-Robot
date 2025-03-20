package frc.robot.extras;

public class AccelLimiter {
    double maxIncrease;
    double maxDecrease;
    double previousOutput = 0;

    public AccelLimiter(double maxIncrease, double maxDecrease) {
        this.maxIncrease = maxIncrease;
        this.maxDecrease = maxDecrease;
    }

    public double calculate(double wantedPower) {
        double output = previousOutput;
        double acceleration = wantedPower - previousOutput;

        if (Math.abs(wantedPower) - Math.abs(previousOutput) > 0) {
            if (wantedPower > 0) { // If is positive
                output += Math.min(acceleration, maxIncrease);
            } else {
                output += Math.max(acceleration, -maxIncrease);
            }

        } else {
            if (wantedPower > 0) {
                output += Math.max(acceleration, -maxDecrease);
            } else {
                output += Math.min(acceleration, maxDecrease);
            }
        }

        previousOutput = output;
        return output;
    }

    public void zeroSpeed() {
        previousOutput = 0;
    }
}
