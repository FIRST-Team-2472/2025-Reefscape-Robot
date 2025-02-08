package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.TargetPosConstants;
import frc.robot.extras.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

    private SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    // It's still going, oh my god.
    private SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private Pigeon2 gyro = new Pigeon2(SensorConstants.kPigeonID);
    // Sets the preliminary odometry. This gets refined by the PhotonVision class,
    // but this is the original.
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions());
    private GenericEntry headingShuffleBoard, odometerShuffleBoard, rollSB, pitchSB;


    private static final SendableChooser<String> colorChooser = new SendableChooser<>();
    private final String red = "Red", blue = "Blue";

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public SwerveSubsystem() {
        // Gets tabs from Shuffleboard
        ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");

        // Sets up the different displays on suffle board
        headingShuffleBoard = programmerBoard.add("Robot Heading", 0).getEntry();
        odometerShuffleBoard = programmerBoard.add("Robot Location", "").getEntry();
        rollSB = programmerBoard.add("Roll", 0).getEntry();
        pitchSB = programmerBoard.add("Pitch", 0).getEntry();
        programmerBoard.add("Pigeon Orientation", gyro.getAngle()).getEntry();

        // zeros heading after pigeon boots up)()
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    public SwerveSubsystem(Pigeon2 gyro, 
        //SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, 
        GenericEntry headingShuffleBoard, GenericEntry odometerShuffleBoard, GenericEntry rollSB, GenericEntry pitchSB) {
        this.gyro = gyro;
        /* 
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        */
        this.headingShuffleBoard = headingShuffleBoard;
        this.odometerShuffleBoard = odometerShuffleBoard;
        this.rollSB = rollSB;
        this.pitchSB = pitchSB;
    }
    public SwerveDriveOdometry getOdometer(){
        return odometer;
    }
    // Just a quick method that zeros the IMU.
    public void zeroHeading() {
        gyro.setYaw(0);
    }

    // Gets the yaw/heading of the robot. getting this right is very important for
    // swerve 
    public double getHeading() {
        // imu is backwards, so it is multiplied by negative one
        return gyro.getYaw().getValueAsDouble();
    }

    // Gets the roll of the robot based on the IMU.
    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    // Gets the pitch of the robot based on what the IMU percieves.
    public double getPitch() {
        return -gyro.getPitch().getValueAsDouble();
    }

    public static boolean isOnRed() {
        // gets the selected team color from the suffleboard
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.isPresent()){
            return ally.get() == Alliance.Red;
        }

        return false;
    }

    // gets our current velocity relative to the x of the field
    public double getXSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vxMetersPerSecond;
    }

    /* public ChassisSpeeds getChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getRotation2d());
    } */

    public ChassisSpeeds getChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getRotation2d());
    }

    // gets our current velocity relative to the x of the robot (front/back)
    public double getXSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vxMetersPerSecond;
    }

    // gets our current velocity relative to the y of the field
    public double getYSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vyMetersPerSecond;
    }

    // gets our current velocity relative to the y of the robot (left/right)
    public double getYSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vyMetersPerSecond;
    }

    // gets our current angular velocity
    public double getRotationalSpeed() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.omegaRadiansPerSecond;
    }

    // Gets our heading and translates it to Rotation2d
    // (all of swerve methods use Rotation2d)
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void zeroOdometry(){
        odometer.resetPosition(new Rotation2d(0), getModulePositions(), new Pose2d());
    }

    public void setOdometry(Pose2d odometryPose){
        odometer.resetPosition(getRotation2d(), getModulePositions(), odometryPose);
    }

    // Gets our drive position aka where the odometer thinks we are
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public SwerveModulePosition[] getModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        return temp;
    }

    public void runModulesFieldRelative(double xSpeed, double ySpeed, double turningSpeed) {
        // Converts robot speeds to speeds relative to field
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, getRotation2d());
        
        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public void runModulesRobotRelative(ChassisSpeeds chassisSpeeds) {
        // Converts robot speeds to speeds relative to field
        this.chassisSpeeds = chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds, getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public void stopModules() {
        // Stops all of the modules. Use in emergencies.
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }


    public boolean isAtPoint(Translation2d targetDrivePos) {
        return getPose().getTranslation().getDistance(targetDrivePos) //
                <= TargetPosConstants.kAcceptableDistanceError; //
    }

    public boolean isAtAngle(Rotation2d angle) {
        return Math.abs(getRotation2d().minus(angle).getDegrees()) //
                <= TargetPosConstants.kAcceptableAngleError;
    }
    


    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // if their speed is larger then the physical max speed, it reduces all speeds
        // until they are smaller than physical max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // sets the modules to desired states
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        // Resets all of the encoders on the robot.
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    @Override
    public void periodic() {
        // this method comes from the subsystem class we inherited. Runs constantly
        // while robot is on

        // changes heading and module states into an x,y coordinate.
        // Updates everything based on the new information it gathers.
        odometer.update(getRotation2d(), getModulePositions());

        headingShuffleBoard.setDouble(getHeading());
        odometerShuffleBoard.setString(getPose().getTranslation().toString());
        pitchSB.setDouble(getPitch());
        rollSB.setDouble(getRoll());
        SmartDashboard.putNumber("frontLeft Encoder", frontLeft.absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("frontRight Encoder", frontRight.absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("BackLeft Encoder", backLeft.absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("BackRight Encoder", backRight.absoluteEncoder.getAbsolutePosition().getValueAsDouble());

        SmartDashboard.putNumber("read frontLeft Encoder", frontLeft.getAbsolutePosition());
        SmartDashboard.putNumber("read frontRight Encoder", frontRight.getAbsolutePosition());
        SmartDashboard.putNumber("read BackLeft Encoder", backLeft.getAbsolutePosition());
        SmartDashboard.putNumber("read BackRight Encoder", backRight.getAbsolutePosition());
        SmartDashboard.putNumber("odometerX", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("odometerY", odometer.getPoseMeters().getY());
        SmartDashboard.putNumber("gyro Yaw", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putBoolean("isRed", isOnRed());
    }
}