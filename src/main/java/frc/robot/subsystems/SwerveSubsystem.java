package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.MotorPowerController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.TargetPosConstants;
import frc.robot.Constants.TeleDriveConstants;
import frc.robot.extras.AccelerationLimiter;
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

    private AccelerationLimiter xLimiter, yLimiter, turningLimiter;
    private PIDController xController, yController;
    public PIDController thetaController;

    public MotorPowerController xPowerController, yPowerController, turningPowerController;

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

        xLimiter = new AccelerationLimiter(TeleDriveConstants.kMaxSpeedMetersPerSecond);
        yLimiter = new AccelerationLimiter(TeleDriveConstants.kMaxAccelerationUnitsPerSecond);
        turningLimiter = new AccelerationLimiter(TeleDriveConstants.kMaxAngularAccelerationUnitsPerSecond);

        xPowerController = new MotorPowerController(0.2, 0.05, .5, 1, .2, 0, 1);
        yPowerController = new MotorPowerController(0.2, 0.05, .5, 1, .2, 0, 1);
        turningPowerController = new MotorPowerController(0.15, 0.1, .3, 1, .1, 0, 1);

        yController = new PIDController(TargetPosConstants.kPDriveController, 0, 0);
        thetaController = new PIDController(TargetPosConstants.kPAngleController, 0.08, 0.02);

        // zeros heading after pigeon boots up)()
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    public SwerveSubsystem(Pigeon2 gyro, //This constructor is used for testing
        SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, 
        GenericEntry headingShuffleBoard, GenericEntry odometerShuffleBoard, GenericEntry rollSB, GenericEntry pitchSB) {
        this.gyro = gyro;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
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

    public void initializeDriveToPointAndRotate(Pose2d targetPosition) {
        xPowerController.calculateMotorPowerController(getPose().getX(), targetPosition.getX());
        yPowerController.calculateMotorPowerController(getPose().getY(), targetPosition.getY());
        //xLimiter.setLimit(TargetPosConstants.kForwardMaxAcceleration,
          //      TargetPosConstants.kBackwardMaxAcceleration);
        //yLimiter.setLimit(TargetPosConstants.kForwardMaxAcceleration,
            //    TargetPosConstants.kBackwardMaxAcceleration);
        //xLimiter.reset(getXSpeedFieldRel());
        //yLimiter.reset(getYSpeedFieldRel());

        //xController.setPID(TargetPosConstants.kPDriveController, 0, 0.002);
        //xController.reset();
        //yController.setPID(TargetPosConstants.kPDriveController, 0, 0.002);
        //yController.reset();
        //thetaController.setPID(TargetPosConstants.kPAngleController, 0, 0);
        //thetaController.reset();
    }

    public void executeDriveToPointAndRotate(Pose2d targetPosition) {
        double xSpeed =  -xPowerController.calculateMotorPowerController(getPose().getX(), targetPosition.getX());
        double ySpeed =  -yPowerController.calculateMotorPowerController(getPose().getY(), targetPosition.getY());

        Rotation2d angleDifference = odometer.getPoseMeters().getRotation().minus(targetPosition.getRotation());
        double turningSpeed = -turningPowerController.calculateMotorPowerController(angleDifference.getRadians(), 0);
        //turningSpeed *= TargetPosConstants.kMaxAngularSpeed;
        //turningSpeed += Math.copySign(TargetPosConstants.kMinAngluarSpeedRadians, turningSpeed);

        //xSpeed = xLimiter.calculate(xSpeed * TargetPosConstants.kMaxSpeedMetersPerSecond);
        //ySpeed = yLimiter.calculate(ySpeed * TargetPosConstants.kMaxSpeedMetersPerSecond);

        //double unitCircleAngle = Math.atan2(ySpeed, xSpeed);
        //xSpeed += Math.copySign(TargetPosConstants.kMinSpeedMetersPerSec, xSpeed) * Math.abs(Math.cos(unitCircleAngle));
        //ySpeed += Math.copySign(TargetPosConstants.kMinSpeedMetersPerSec, ySpeed) * Math.abs(Math.sin(unitCircleAngle));
        
        
        runModulesFieldRelative(xSpeed, ySpeed, turningSpeed);
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
                xSpeed, ySpeed, turningSpeed, odometer.getPoseMeters().getRotation());
        
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
        SmartDashboard.putNumber("translation Error", getPose().getTranslation().getDistance(targetDrivePos));
        boolean isAtPose =  getPose().getTranslation().getDistance(targetDrivePos) <= TargetPosConstants.kAcceptableDistanceError; //

        SmartDashboard.putBoolean("isAtPose", isAtPose);
        return isAtPose;

        
    }

    public boolean isAtAngle(Rotation2d angle) {
        SmartDashboard.putNumber("angle error", Math.abs(odometer.getPoseMeters().getRotation().minus(angle).getDegrees()));
        boolean isAtAngle =  Math.abs(odometer.getPoseMeters().getRotation().minus(angle).getDegrees()) <= TargetPosConstants.kAcceptableAngleError;

        SmartDashboard.putBoolean("isAtAngle", isAtAngle);
        return isAtAngle;
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
        SmartDashboard.putNumber("odometerAngle", odometer.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("gyro Yaw", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putBoolean("isRed", isOnRed());
    }
}