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
        
        double xSpeed = ySpdFunction.get();
        double ySpeed = xSpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        System.out.print("Joystick Input: (" + xSpeed + ", " + ySpeed + ")");

        // 2. Apply deadband & square output for more precise movement at low speed
        xSpeed = Math.abs(xSpeed) > OperatorConstants.kFlightControllerDeadband ?  xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OperatorConstants.kFlightControllerDeadband ?  ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OperatorConstants.kFlightControllerDeadband ?  turningSpeed : 0.0;

        xSpeed = slowButton.get() ? xSpeed*.3 : xSpeed;
        ySpeed = slowButton.get() ? ySpeed*.3 : ySpeed;
        
        swerveSubsystem.runModulesFieldRelative(xSpeed, ySpeed, turningSpeed);
    }

    @Override
    public void end(boolean interrupted) {

        // swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
