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

    private LEDSubsystem() {
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
        DANGER(1),
        INTAKE(2),
        HAS_NOTE(3),
        AIMING(4),
        AIMING_ON_TARGET(5),
        SHOOTING(6),
        SHOOTING_ON_TARGET(7),
        DONE_SHOOTING(8),
        NO_AUTO(9),
        BLUE_AUTO(10),
        RED_AUTO(11),
        AMP_IDLE(12);

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
        int code = 5;
        if(!disableLEDs) {
        code = currentStatusMode.code;
        } else {
            //LEDs are disabled
            code = 5;
        }

        // Code for encoding the code to binary on the digitalOutput pins
        channel1.set((code & 1) > 0);   // 2^0
        channel2.set((code & 2) > 0);   // 2^1
        channel3.set((code & 4) > 0);   // 2^2
    }

    // Disables LEDs (turns them off)
    public void disableLEDs() {
        disableLEDs = true;
    }

    // Enables LEDs (turns them on)
    public void enableLEDs() {
        disableLEDs = false;
    }
}