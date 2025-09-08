package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotStatus;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.extras.NewNewAccelLimiter;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> slowButton, resetHeadingButton, leftModeToggle, rightModeToggle;
    private final NewNewAccelLimiter xSpeedLimiter, ySpeedLimiter, turningSpeedLimiter;
    private final Timer modeToggleCooldownTimer = new Timer();

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> slowButton, Supplier<Boolean> resetHeadingButton, Supplier<Boolean> leftModeToggle, Supplier<Boolean> rightModeToggle) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.leftModeToggle = leftModeToggle;
        this.rightModeToggle = rightModeToggle;
        this.turningSpdFunction = turningSpdFunction;
        this.slowButton = slowButton;
        this.resetHeadingButton = resetHeadingButton;
        xSpeedLimiter = new NewNewAccelLimiter(.02, .1);
        ySpeedLimiter = new NewNewAccelLimiter(.02, .1);
        turningSpeedLimiter = new NewNewAccelLimiter(.02, .1);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        modeToggleCooldownTimer.start();
        System.out.println("Swerve Joystick contoslled!");
    }

    @Override
    public void execute() {
        //toggle between on and off for kid mode vs demoing
        if(leftModeToggle.get() && rightModeToggle.get() && modeToggleCooldownTimer.get() > 1) {
            modeToggleCooldownTimer.restart();
            RobotStatus.kidMode = !RobotStatus.kidMode;
        }
        if(resetHeadingButton.get())
            swerveSubsystem.zeroRobotHeading();

        // 1. Get real-time joystick inputs flipping the x and y of controller to the fields x and y
        double xSpeed = ySpdFunction.get();
        double ySpeed = xSpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // System.out.print("Joystick Input: (" + xSpeed + ", " + ySpeed + ")");

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OperatorConstants.kFlightControllerDeadband ?  xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OperatorConstants.kFlightControllerDeadband ?  ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OperatorConstants.kFlightControllerDeadband ?  turningSpeed : 0.0;

        // 4. invert direction if on red alliance
        if(SwerveSubsystem.isOnRed()){
            xSpeed *= -1;
            ySpeed *= -1;
        }
        if(slowButton.get()){
            xSpeed *=.3;
            ySpeed *=.3;
            turningSpeed *=.3;
        }
        //kid drive specific
        if(RobotStatus.kidMode){
            xSpeed *= Constants.TeleDriveConstants.kKidDriveMultiplier;
            ySpeed *= Constants.TeleDriveConstants.kKidDriveMultiplier;
            turningSpeed *= Constants.TeleDriveConstants.kKidDriveMultiplier;
            xSpeed = xSpeedLimiter.calculate(xSpeed);
            ySpeed = ySpeedLimiter.calculate(ySpeed);
            turningSpeed = turningSpeedLimiter.calculate(turningSpeed);
        }else if(!slowButton.get()){
            xSpeed = input_2_speed(xSpeed);
            ySpeed = input_2_speed(ySpeed);
        }

        swerveSubsystem.runModulesFieldRelative(xSpeed, ySpeed, turningSpeed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    // alters the joystick input according to a polynomial function for more precise control
    public static double input_2_speed(double x) {
        return 19.4175 * Math.pow(x, 13) - 103.7677 * Math.pow(x, 11) + 195.0857 * Math.pow(x, 9)
                - 165.5452 * Math.pow(x, 7) + 61.8185 * Math.pow(x, 5) - 6.5099 * Math.pow(x, 3) + 0.5009 * x;
    }
}
