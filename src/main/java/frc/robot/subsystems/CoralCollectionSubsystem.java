package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralCollectionSubsystem extends SubsystemBase{
    Servo intakeServo = new Servo(0);
    public CoralCollectionSubsystem() {
        intakeServo.setAngle(0);

    }
    public void setServoAngle(double angle){
        intakeServo.setAngle(angle);
    }
}
