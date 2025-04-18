package frc.robot;
import java.io.Reader;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;
public class CustomLEDPattern implements LEDPattern {

    public void applyTo(LEDReader reader, LEDWriter writer) {}
    private void countDownDrain(LEDPattern pattern) {
        
        LEDPattern countdownMaskR = progressMaskLayer(() -> (DriverStation.getMatchTime() - 135) / 30,
        0,20);
        pattern = pattern.mask(countdownMaskR);
        pattern.reversed();
        LEDPattern countdownMaskl = progressMaskLayer(() -> (DriverStation.getMatchTime() - 135) / 30,
        0,20);
        pattern = pattern.mask(countdownMaskl);
    }
    private void countDownBar(LEDPattern pattern) {
        // this will need tweaking so it only happens on the one LED strip
        // the numbers come from the match time - 15 seconds and 15 seconds left of match
        LEDPattern countdownMask = LEDPattern.progressMaskLayer(() -> (DriverStation.getMatchTime() - 135) / 15);
        pattern = pattern.mask(countdownMask);
    }

    static LEDPattern progressMaskLayer(DoubleSupplier progressSupplier, int startIndex, int endIndex) {
        return (reader, writer) -> {
        double progress = MathUtil.clamp(progressSupplier.getAsDouble(), 0, 1);

        int bufLen = endIndex-startIndex+1;
        int max = (int) (bufLen * progress) + startIndex;

        for (int led = startIndex; led < max; led++) {
            writer.setLED(led, Color.kWhite);
        }

        for (int led = max; led < endIndex+1; led++) {
            writer.setLED(led, Color.kBlack);
        }
        };
    }
    
}
