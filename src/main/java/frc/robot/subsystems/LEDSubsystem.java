package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStatus;
import frc.robot.Constants.ElevatorConstants;

import static frc.robot.Constants.LEDConstants.*;

import java.util.Map;

public class LEDSubsystem extends SubsystemBase {

    AddressableLED LEDs = new AddressableLED(kLEDPWMPort);
    AddressableLEDBuffer LEDBuffer;
    

    public LEDSubsystem() {
        LEDBuffer = new AddressableLEDBuffer(kBackLEDStripLEDCount);

        LEDs.setLength(LEDBuffer.getLength());
        
        LEDs.setData(LEDBuffer);

        LEDs.start();

    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        LEDPattern pattern;   
        Color h = new Color(0,255,0);
        Color yellow = new Color(187, 0, 75);

        if(RobotStatus.hasCoral){
            pattern = LEDPattern.solid(h);
        }else if(RobotStatus.seeCoral)   {
            pattern = LEDPattern.solid(Color.kPurple);
        }else if(RobotStatus.isDispensing)   {
            // all hues at maximum saturation and full brightness
            pattern = LEDPattern.rainbow(255, 225);
            // Create a new pattern that scrolls the rainbow pattern across the LED strip
            pattern = pattern.scrollAtAbsoluteSpeed(kRainbowScrollSpeed, kLEDSpacing);
        }else{
            pattern = LEDPattern.steps(Map.of(0.00, Color.kRed, 0.2, yellow, 0.40, Color.kRed, 0.6, yellow, .8, Color.kRed));
        }
        // this will need tweaking so it only happens on the one LED strip
        // the numbers come from the match time - 15 seconds and 15 seconds left of match
        if(DriverStation.isFMSAttached()){
            LEDPattern countdownMask = LEDPattern.progressMaskLayer(() -> DriverStation.getMatchTime() / 15);
            pattern = pattern.mask(countdownMask);
        }
        
        // Apply the LED pattern to the data buffer
        pattern.applyTo(LEDBuffer);

        // Write the data to the LED strip
        LEDs.setData(LEDBuffer);
    }

    /* public void off(boolean off) {
        this.off = off;
    }

    public void green(boolean g) {
        green = g;
    }

    public void blue(boolean b) {
        blue = b;
    }

    public void yellow(boolean y) {
        yellow = y;
    }

    public void cyan(boolean c) {
        cyan = c;
    }

    public void red(boolean r) {
        red = r;
    }

    public void purple(boolean p) {
        purple = p;
    } 

    public void Timer(Timer t) {
        if (t.hasElapsed(140)){
            red(t.get() % .5 > .25);
            off(t.get() % .5 < .25);
            green(false);
            purple(false);
            blue(false);
            yellow(false);
            cyan(false);
        } else if (t.hasElapsed(130)) {
            blue(t.get() % 1 > .5);
            off(t.get() % 1 < .5);
            red(false);
            green(false);
            purple(false);
            yellow(false);
            cyan(false);
        }
    } 

    private void coral() {
        green(RobotStatus.hasCoral);
        purple(RobotStatus.seeCoral);
    } */
}