package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SensorStatus;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LEDSubsystem extends SubsystemBase {

    private static LEDSubsystem INSTANCE;

    private final DigitalOutput channel1, channel2, channel3;

    private LEDStatusMode currentStatusMode;

    public boolean disableLEDs,red,green,blue,yellow,purple,cyan,off;

    public Color color = Color.kBlack;

    public LEDSubsystem() {
        // DIO outputs
        channel1 = new DigitalOutput(Constants.LEDConstants.kChannel1);
        channel2 = new DigitalOutput(Constants.LEDConstants.kChannel2);
        channel3 = new DigitalOutput(Constants.LEDConstants.kChannel3);

        currentStatusMode = LEDStatusMode.OFF;
    }

    public static LEDSubsystem getInstance() {
        // Method to allow calling this class and getting the single instance from
        // anywhere, creating the instance if the first time.
        if (INSTANCE == null) {
            INSTANCE = new LEDSubsystem();
        }
        return INSTANCE;
    }

    public static enum LEDStatusMode {
        IDLE(0),
        RED(1),
        GREEN(2),
        BLUE(3),
        YELLOW(4),
        PURPLE(5),
        CYAN(6),
        OFF(7);

        private final int code;

        private LEDStatusMode(int code) {
            this.code = code;
        }

        public int getPositionTicks() {
            return code;
        }
    }

    @Override
    public void periodic() {
        int code = 0;
        if (!disableLEDs) {
            if (green) {
                LEDMode(LEDStatusMode.GREEN);
            } else if (blue) {
                LEDMode(LEDStatusMode.BLUE);
            } else if (yellow) {
                LEDMode(LEDStatusMode.YELLOW);
            } else if (cyan) {
                LEDMode(LEDStatusMode.CYAN);
            } else if (purple) {
                LEDMode(LEDStatusMode.PURPLE);
            } else if (red) {
                LEDMode(LEDStatusMode.RED);
            } else if (off) {
                LEDMode(LEDStatusMode.OFF);
            } else {
                LEDMode(LEDStatusMode.IDLE);
            }
            code = currentStatusMode.code;
        } else {
            // LEDs are disabled
            code = 0;
        }

        // Code for encoding the code to binary on the digitalOutput pins
        channel1.set((code & 1) > 0); // 2^0
        channel2.set((code & 2) > 0); // 2^1
        channel3.set((code & 4) > 0); // 2^2

       /* switch(code) {
            case (1): 
            color = Color.kFirstRed;
            break;
            case(2):
            color = Color.kDarkGreen;
            break;
            case (3): 
            color = Color.kFirstBlue;
            break;
            case (4): 
            color = Color.kFirstRed;
            break;
            case (5): 
            color = Color.kGoldenrod;
            break;
            case (6): 
            color = Color.kMediumPurple;
            break;
            case (7): 
            color = Color.kWhite;
            break;
            default:
            color = Color.kBlack;
        }
*/
        
        
        coral();
    }

    public void LEDMode(LEDStatusMode code) {
        currentStatusMode = code;
    }

    public void disableLEDs() {
        disableLEDs = true;
    }

    public void enableLEDs() {
        disableLEDs = false;
    }

    public void off(boolean off) {
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
        green(SensorStatus.hasCoral);
        purple(SensorStatus.seeCoral);
    }
}