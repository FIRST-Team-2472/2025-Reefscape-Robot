package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStatus;

import static frc.robot.Constants.LEDConstants.*;

import java.util.Map;
import java.util.function.DoubleSupplier;

public class LEDSubsystem extends SubsystemBase {

    AddressableLED LEDs = new AddressableLED(kLEDPWMPort);
    AddressableLEDBuffer LEDBuffer;
    LEDPattern pattern;
    

    public LEDSubsystem() {
        LEDBuffer = new AddressableLEDBuffer(kBackLEDStripLEDCount + kElevatorLEDStripLEDCount);

        LEDs.setLength(LEDBuffer.getLength());
        
        LEDs.setData(LEDBuffer);

        LEDs.start();

    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();   

        if(RobotStatus.hasCoral){
            pattern = LEDPattern.solid(Color.kGreen);
        }else if(RobotStatus.seeCoral)   {
            pattern = LEDPattern.solid(Color.kPurple);
        }else if(RobotStatus.isDispensing)   {
            // all hues at maximum saturation and full brightness
            pattern = LEDPattern.rainbow(255, 225);
            // Create a new pattern that scrolls the rainbow pattern across the LED strip
            pattern = pattern.scrollAtAbsoluteSpeed(kRainbowScrollSpeed, kLEDSpacing);
        }else{
            pattern = LEDPattern.steps(Map.of(0.00, Color.kRed, 0.25, Color.kYellow, 0.50, Color.kRed, 0.75, Color.kYellow));
        }
        // this will need tweaking so it only happens on the one LED strip
        // the numbers come from the match time - 15 seconds and 15 seconds left of match
        LEDPattern countdownMask = LEDPattern.progressMaskLayer(() -> (DriverStation.getMatchTime() - 135) / 15);
        pattern = pattern.mask(countdownMask);
        
        // Apply the LED pattern to the data buffer
        pattern.applyTo(LEDBuffer);

        // Write the data to the LED strip
        LEDs.setData(LEDBuffer);
    }

    private void countDownDrain() {
        
        LEDPattern countdownMaskR = LEDPattern.progressMaskLayer(() -> (DriverStation.getMatchTime() - 135) / 30);
        pattern = pattern.mask(countdownMaskR);
        pattern.reversed();
        LEDPattern countdownMaskl = LEDPattern.progressMaskLayer(() -> (DriverStation.getMatchTime() - 135) / 30);
        pattern = pattern.mask(countdownMaskl);
    }
    private void countDownBar() {
        // this will need tweaking so it only happens on the one LED strip
        // the numbers come from the match time - 15 seconds and 15 seconds left of match
        LEDPattern countdownMask = LEDPattern.progressMaskLayer(() -> (DriverStation.getMatchTime() - 135) / 15);
        pattern = pattern.mask(countdownMask);
    }
}