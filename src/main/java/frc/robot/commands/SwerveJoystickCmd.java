package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    Supplier<Boolean> slowButton;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> slowButton) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slowButton = slowButton;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Swerve Joystick contoslled!");
    }

    @Override
    public void execute() {

        // 1. Get real-time joystick inputs flipping the x and y of controller to the fields x and y
        double xSpeed = ySpdFunction.get();
        double ySpeed = xSpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        System.out.print("Joystick Input: (" + xSpeed + ", " + ySpeed + ")");

        // 2. Apply deadband & square output for more precise movement at low speed
        xSpeed = Math.abs(xSpeed) > OperatorConstants.kDeadband ?  xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OperatorConstants.kDeadband ?  ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OperatorConstants.kDeadband ?  turningSpeed : 0.0;

        // 3. Apply Speed multiplier
        if(slowButton.get()){
            xSpeed *= 0.3;
            ySpeed *= 0.3;
        }
        swerveSubsystem.runModulesFieldRelative(-xSpeed, -ySpeed, -turningSpeed);
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