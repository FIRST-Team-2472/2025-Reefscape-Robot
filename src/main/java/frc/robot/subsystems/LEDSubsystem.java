package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem INSTANCE;
    
    private final DigitalOutput channel1,channel2,channel3;

    private LEDStatusMode currentStatusMode;

    private boolean disableLEDs;
    private boolean robotDisabled;

    public LEDSubsystem() {
        // DIO outputs
        channel1 = new DigitalOutput(Constants.LEDConstants.kChannel1);
        channel2 = new DigitalOutput(Constants.LEDConstants.kChannel2);
        channel3 = new DigitalOutput(Constants.LEDConstants.kChannel3);
        robotDisabled = true;

        currentStatusMode = LEDStatusMode.OFF;
    }

    public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
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
        if(!disableLEDs) {
        code = currentStatusMode.code;
        } else {
            //LEDs are disabled
            code = 0;
        }

        // Code for encoding the code to binary on the digitalOutput pins
        channel1.set((code & 1) > 0);   // 2^0
        channel2.set((code & 2) > 0);   // 2^1
        channel3.set((code & 4) > 0);   // 2^2

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
}