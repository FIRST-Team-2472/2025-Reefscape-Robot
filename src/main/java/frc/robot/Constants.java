package frc.robot;

import java.lang.System.Logger.Level;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    /**
     * The LoggingConstants class holds the logging configuration for the robot.
     * 
     * <p>currentLogLevel: The current logging level for the robot's logging system.
     * The log level determines the severity of messages that will be logged.
     * Common log levels include DEBUG, INFO, WARN, and ERROR, with DEBUG being the most
     * verbose and ERROR being the least.
     */
    public static final class LoggingConstants{
        public static final Level CURRENT_LOG_LEVEL = Level.INFO;
    }
    
    public static final class ElevatorConstants {
        public static final int kLeftElevatorMotorID = 16;
        public static final int kRightElevatorMotorID = 40;

        public static final double kElevatorGearRatio = 1.0 /20; // 20 rotations of the motor to one shaft rotation the 1.0 is so it does decimal not integer division
        public static final double kSprocketCircumference = 5.538628;// slightly rounded and in inches
        public static final double kElevatorMotorRotationsToInches = kSprocketCircumference * kElevatorGearRatio * -2; // times two because its a two stage elevator negative because the encoder reads negative when going up

        public static final double kElevatorL4Height = 57;
        public static final double kElevatorL3Height = 33.5;
        public static final double kElevatorL2Height = 17.5;
        public static final double kElevatorL1Height = 9;

        public static final double kElevatorMaxHeight = 57;
    }
    public static final class ClimberConstants {
        public static final int kClimberMotorID = 19; // change later
        
        public static final double kClimberGearRatio = 1; // change later
        public static final double kClimberOutAngle = 90; // change later
        public static final double kClimberInAngle = 270; // change later
    }
    public static final class CoralDispenserConstants {
        public static final int kLeftMotorID = 17; 
        public static final int kRightMotorID = 44; 
    }
    public static final class AlgaeConstants {
        public static final int kPivotMotorID = 18;
        public static final double kPivotGearRatio = 1/2;
        public static final int kSpinMotorID = 10;
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = .0977;
        // gear ratio is inversed (1/gear ratio) so we can multiply instead of dividing
        // the Rot to meter funtion
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        // 2048 is pulses per rotation of the motor
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

        // CTRE mesures their velcity in 100ms, so we multiply it by 10 to get 1s
        public static final double kDriveEncoderRPMS2MeterPerSec = kDriveEncoderRot2Meter * 10;
        public static final double kTurningEncoderRPMS2RadPerSec = kTurningEncoderRot2Rad * 10;
        // use guess and check to find. when the module is overshooting this needs to be
        // fine tuned
        public static final double kPTurning = 0.2;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(19.5);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),//this is right dont mess it up
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 3;

        public static final int kFrontRightTurningMotorPort = 1;
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 6;

        // Positive should be counter clockwise
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;

        // Positive should be clockwise
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // To find set the motors forward record the value (don't inverse the value)
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegrees = 180;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDegrees = 0; 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDegrees = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetDegrees = 180;

        // Max physical speed of our motors. Required for motor speed caculations
        // To find set the modules to 100% and see what speed cap out at
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.72;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 7 * 2 * Math.PI;

    }

    public static final class TeleDriveConstants {
        // Motion constants for joystick drive mode -> dependant on max speed
        // Max x/y speed of robot in this drive mode
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 1; //used to be /4
        // Max rotation speed of robot in this drive mode
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 16;
        // Max x/y acceleration of robot in this drive mode
        public static final double kMaxAccelerationUnitsPerSecond = 8;
        // Max rotational acceleration of robot in this drive mode
        public static final double kMaxAngularAccelerationUnitsPerSecond = Math.PI * 2;
    }

    public static final class AutoConstants {
        // Motion constants for sequential path drive mode
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 9.44;//was 4
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 14;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;// was 5.5
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 1.5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final double TRANSLATION_KP = 0.7; 
        public static final double TRANSLATION_KI = 0; 
        public static final double TRANSLATION_KD = 0;
                    
        public static final double ROTATION_KP = 0.7;
        public static final double ROTATION_KI = 0;
        public static final double ROTATION_KD = 0;
                    
        public static final double MAX_MODULE_SPEED = 2;
        public static final double DRIVE_BASE_RADIUS_METERS = Math.hypot(DriveConstants.kTrackWidth/2,
            DriveConstants.kWheelBase/2);
            
        /*  PathPlanner Holonomic Controller - Not used anymore
        public static final PPHolonomicDriveController HOLONOMIC_PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
        new PIDConstants(
            TRANSLATION_KP,
            TRANSLATION_KI,
            TRANSLATION_KD), // Translation PID constants
        new PIDConstants(
            ROTATION_KP,
            ROTATION_KI,
            ROTATION_KD) //Time between code runs
        ); */
    }

    public static final class TargetPosConstants {
        // Motion constants for target position drive mode
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeed = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 16;

        public static final double kForwardMaxAcceleration = 2;
        public static final double kBackwardMaxAcceleration = -12;
        public static final double kMaxAngularAcceleration = Math.PI / 3;
        public static final double kBackwardAngularAcceleration = -Math.PI * 9;

        public static final double kMinAngluarSpeedRadians = Math.PI / 16;
        public static final double kMinSpeedMetersPerSec = .2;

        public static final double kPDriveController = .1;
        public static final double kPAngleController = .05;
        public static final double kAcceptableDistanceError = 0.04;
        public static final double kAcceptableAngleError = 1.5;
    }

    public static final class OperatorConstants {
        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kXboxControllerPort = 2;

        // left joystick
        public static final int kLeftDriverYAxis = 1;
        public static final int kLeftDriverXAxis = 0;

        // right joystick
        public static final int kRightDriverRotAxis = 0;

        // area were joysticks will not activate
        public static final double kXboxControllerDeadband = 0.1;
        public static final double kFlightControllerDeadband = 0.05;
    }

    public static final class SensorConstants {
        public static final int kPigeonID = 0;

        public static final double sizeOfFieldMetersX = 17.55;
        public static final double sizeOfFieldMetersY = 8;

        public static int kAlgeaABSEncoderDIOPort = 0; // set later
        public static int kClimberABSEncoderDIOPort = 1; // set later
    }

    public static class LEDConstants {
        public static int kChannel1 = 0;
        public static int kChannel2 = 1;
        public static int kChannel3 = 2;
      }
}
