package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem INSTANCE;

    private final DigitalOutput channel1, channel2, channel3;

    private LEDStatusMode currentStatusMode;

    private boolean disableLEDs,red,green,blue,yellow,purple,cyan;

    public Color color;

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
        OFF(0),
        RED(1),
        GREEN(2),
        BLUE(3),
        YELLOW(4),
        PURPLE(5),
        CYAN(6),
        CENTURION(7);

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

        switch(code) {
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

    public void climbAtAngle(boolean atangle) {
        green = atangle;
    }

    public void runningAutonomistCommand(boolean running) {
        blue = running;
    }

    public void coralCollected(boolean collected) {
        yellow = collected;
    }

    public void elevatorCan(boolean can) {
        cyan = can;
    }

    public void isClimbing(boolean climbing) {
        red = climbing;
    }
}