package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> slowButton, resetHeadingButton;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> slowButton, Supplier<Boolean> resetHeadingButton) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slowButton = slowButton;
        this.resetHeadingButton = resetHeadingButton;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Swerve Joystick contoslled!");
    }

    @Override
    public void execute() {

        if(resetHeadingButton.get())
            swerveSubsystem.zeroHeading();
      
        // 1. Get joystick values
        double xSpeed = ySpdFunction.get();
        double ySpeed = xSpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OperatorConstants.kFlightControllerDeadband ?  xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OperatorConstants.kFlightControllerDeadband ?  ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OperatorConstants.kFlightControllerDeadband ?  turningSpeed : 0.0;

        // 3. Apply polynomial function or slowmode to joystick values
        if (!slowButton.get()) {
            xSpeed = input_2_speed(xSpeed);
            ySpeed = input_2_speed(ySpeed);

        } else if(slowButton.get()){
            xSpeed *= .3;
            ySpeed *= .3;
        }

        // 4. invert direction if on blue alliance
        if(!SwerveSubsystem.isOnRed()){
            xSpeed *= -1;
            ySpeed *= -1;
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
