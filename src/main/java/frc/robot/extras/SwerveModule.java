package frc.robot.extras;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    final TalonFX driveMotor;
    final VelocityVoltage driveMotorVelocityVoltage = new VelocityVoltage(0);

    final TalonFX turningMotor;
    final VelocityVoltage turningMotorVelocityVoltage = new VelocityVoltage(0);

    private final PIDController turningPidController;

    // Abslute encoder for swerve drive module
    public CANcoder absoluteEncoder;
    private double absoluteEncoderOffset;
    private boolean absoluteEncoderReversed;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        turningMotor.getConfigurator().apply(new TalonFXConfiguration());

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        absoluteEncoder = new CANcoder(absoluteEncoderId);
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        // lets us use a PID system for the turning motor
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // tells the PID controller that our motor can go from -PI to PI (it can rotate
        // continously)
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getRotorPosition().getValue() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        // Use absolute encoder for most things instead of this
        // Isn't currently bound to a certian range. Will count up indefintly

        // measured in revolutions not radians. easier to understand
        return (turningMotor.getRotorPosition().getValue() * ModuleConstants.kTurningEncoderRot2Rad) / (2 * Math.PI);
    }

    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValue() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec;
    }

    public double getTurningVelocity() {
        // measured in revolutions not radians. easier to understand
        return (driveMotor.getRotorVelocity().getValue() * ModuleConstants.kTurningEncoderRPMS2RadPerSec) / (2 * Math.PI);
    }

    public double getAbsolutePosition() {
        // converts from (-.5, .5) to (-180, 180)
        double angle = 360 * absoluteEncoder.getAbsolutePosition().getValue();
        angle -= absoluteEncoderOffset;
        angle *= absoluteEncoderReversed ? -1 : 1;
        
        return angle;
        // return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningMotor.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d().fromDegrees(getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsolutePosition()));
    }

    // a swerve module state is composed of a speed and direction
    public void setDesiredState(SwerveModuleState state) {
        // prevents wheels from changing direction if it is given barely any speed
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // make the swerve module doesn't ever turn more than 90 degrees instead of 180;
        state = SwerveModuleState.optimize(state, getState().angle);

        //driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        driveMotor.set(state.speedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getState().angle.getRadians(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    public double map(double inputMax, double inputMin, double outputMax, double outputMin, double value){
        return ((value - inputMin)/(inputMax - inputMin)) * (outputMax - outputMin) + outputMin;
    }

}
