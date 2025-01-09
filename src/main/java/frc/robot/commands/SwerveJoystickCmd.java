package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Swerve Joystick contoslled!");
    }

    @Override
    public void execute() {
        double slow = 1;

        // 1. Get real-time joystick inputs
        double xSpeed = -ySpdFunction.get()*.3;
        double ySpeed = -xSpdFunction.get()*.3;
        double turningSpeed = -turningSpdFunction.get();

        System.out.print("Joystick Input: (" + xSpeed + ", " + ySpeed + ")");

        // 2. Apply deadband & square output for more precise movement at low speed
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ?  xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ?  ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ?  turningSpeed : 0.0;
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
