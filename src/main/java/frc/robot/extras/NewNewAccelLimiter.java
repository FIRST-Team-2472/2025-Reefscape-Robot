package frc.robot.extras;

public class NewNewAccelLimiter {
    double maxIncrease; 
    double maxDecrease;
    double previousOutput = 0;
    public NewNewAccelLimiter(double maxIncrease, double maxDecrease){
        this.maxIncrease = maxIncrease;
        this.maxDecrease = maxDecrease;
    }
    public double calculate(double wantedDrive){
        double output = previousOutput;
        if(Math.abs(wantedDrive) - Math.abs(previousOutput) > 0){
            if(wantedDrive > 0){
                output += Math.min(wantedDrive - previousOutput, maxIncrease);
            }else{
                output += Math.max(wantedDrive - previousOutput, -maxIncrease);
            }
        }else{
            if(wantedDrive > 0){
                output += Math.max(wantedDrive - previousOutput, -maxDecrease);
            }else{
                output += Math.min(wantedDrive - previousOutput, maxDecrease);
            }
        }
        previousOutput = output;
        return output;
    }
    public void zeroSpeed(){
        previousOutput = 0;
    }
}
