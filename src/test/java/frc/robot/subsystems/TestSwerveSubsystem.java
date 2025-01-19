package frc.robot.subsystems;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import frc.robot.extras.SwerveModule;

public class TestSwerveSubsystem {
    private Pigeon2 pigeon;
    private StatusSignal<Angle> yawStatusSignal;
    private StatusSignal<Angle> pitchStatusSignal;
    private StatusSignal<Angle> rollStatusSignal;
    private StatusSignal<Angle> absEncodedrSignal;
    private SwerveSubsystem swerveSubsystem;
    //GenericEntrys are mocked to avoid null pointer exceptions
    private GenericEntry headingShuffleBoard;
    private GenericEntry odometerShuffleBoard;
    private GenericEntry rollSB;
    private GenericEntry pitchSB;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    private double frontLeftDrivePosition, frontRightDrivePosition, backLeftDrivePosition, backRightDrivePosition;

    //This is the setup for the tests @BeforeEach means that this method will run before each test
    @BeforeEach
    public void setUp() {
        headingShuffleBoard = mock();
        odometerShuffleBoard = mock();
        rollSB = mock();
        pitchSB = mock();

        pigeon = mock();
        yawStatusSignal = mock();
        when(pigeon.getYaw()).thenReturn(yawStatusSignal);
        pitchStatusSignal = mock();
        when(pigeon.getPitch()).thenReturn(pitchStatusSignal);
        rollStatusSignal = mock();
        when(pigeon.getRoll()).thenReturn(rollStatusSignal);

        frontLeft = mock();
        frontRight = mock();
        backLeft = mock();
        backRight = mock();

        frontLeft.absoluteEncoder = mock();
        frontRight.absoluteEncoder = mock();
        backLeft.absoluteEncoder = mock();
        backRight.absoluteEncoder = mock();
        absEncodedrSignal = mock();

        swerveSubsystem = new SwerveSubsystem(pigeon, 
        frontLeft, frontRight, backLeft, backRight, 
        headingShuffleBoard, odometerShuffleBoard, rollSB, pitchSB);
    }
    @Test
    public void testIsAtAngle(){ //runs test for isAtAngle method
        when(yawStatusSignal.getValueAsDouble()).thenReturn(0.0);
        assertTrue(swerveSubsystem.isAtAngle(new Rotation2d()));
        when(yawStatusSignal.getValueAsDouble()).thenReturn(1.6);
        assertFalse(swerveSubsystem.isAtAngle(new Rotation2d()));
    }
    @Test
    public void testIsAtPoint(){ //runs test for isAtPoint method
        Translation2d targetDrivePos = new Translation2d();
        assertTrue(swerveSubsystem.isAtPoint(targetDrivePos));
        targetDrivePos = new Translation2d(1,1);
        assertFalse(swerveSubsystem.isAtPoint(targetDrivePos));
    }
    @Test
    public void testOdometryAngle(){ //runs test for changing yaw and then updating odometry in periodic
        //first block to appease shuffleboard
        when(frontLeft.getAbsolutePosition()).thenReturn(0.0);
        when(frontRight.getAbsolutePosition()).thenReturn(0.0);
        when(backLeft.getAbsolutePosition()).thenReturn(0.0);
        when(backRight.getAbsolutePosition()).thenReturn(0.0);
        when(absEncodedrSignal.getValueAsDouble()).thenReturn(0.0);
        when(frontLeft.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(frontRight.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(backLeft.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(backRight.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);

        //needed to update odometry when mocking swerve modules
        when(frontLeft.getDrivePosition()).thenReturn(0.0);
        when(frontRight.getDrivePosition()).thenReturn(0.0);
        when(backLeft.getDrivePosition()).thenReturn(0.0);
        when(backRight.getDrivePosition()).thenReturn(0.0);
        frontLeftDrivePosition = frontLeft.getDrivePosition();
        frontRightDrivePosition = frontRight.getDrivePosition();
        backLeftDrivePosition = backLeft.getDrivePosition();
        backRightDrivePosition = backRight.getDrivePosition();
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(frontLeftDrivePosition, new Rotation2d(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(frontRightDrivePosition, new Rotation2d(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(backLeftDrivePosition, new Rotation2d(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(backRightDrivePosition, new Rotation2d(0)));// 0 = 100% x movement, 90 = 100% y movement

        when(yawStatusSignal.getValueAsDouble()).thenReturn(90.0);
        swerveSubsystem.periodic();

        var odometerPostUpdate = swerveSubsystem.getOdometer().getPoseMeters();
        assertEquals(90.0, odometerPostUpdate.getRotation().getDegrees());

        when(yawStatusSignal.getValueAsDouble()).thenReturn(0.0);
        swerveSubsystem.periodic();

        var odometerSecondPostUpdate = swerveSubsystem.getOdometer().getPoseMeters();
        assertNotEquals(90.0, odometerSecondPostUpdate.getRotation().getDegrees());
    }
    @Test
    public void testOdometryTranslation(){ //runs test for changing drive position and then updating odometry in periodic
        //first block to appease shuffleboard
        when(frontLeft.getAbsolutePosition()).thenReturn(0.0);
        when(frontRight.getAbsolutePosition()).thenReturn(0.0);
        when(backLeft.getAbsolutePosition()).thenReturn(0.0);
        when(backRight.getAbsolutePosition()).thenReturn(0.0);
        when(absEncodedrSignal.getValueAsDouble()).thenReturn(0.0);
        when(frontLeft.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(frontRight.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(backLeft.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(backRight.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);

        when(frontLeft.getDrivePosition()).thenReturn(0.0);
        when(frontRight.getDrivePosition()).thenReturn(0.0);
        when(backLeft.getDrivePosition()).thenReturn(0.0);
        when(backRight.getDrivePosition()).thenReturn(0.0);
        frontLeftDrivePosition = frontLeft.getDrivePosition();
        frontRightDrivePosition = frontRight.getDrivePosition();
        backLeftDrivePosition = backLeft.getDrivePosition();
        backRightDrivePosition = backRight.getDrivePosition();
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(frontLeftDrivePosition, new Rotation2d(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(frontRightDrivePosition, new Rotation2d(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(backLeftDrivePosition, new Rotation2d(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(backRightDrivePosition, new Rotation2d(0)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();

        var odometerPostUpdate = swerveSubsystem.getOdometer().getPoseMeters();
        assertEquals(0.0, odometerPostUpdate.getTranslation().getX());
        assertEquals(0.0, odometerPostUpdate.getTranslation().getY());
        assertEquals(0.0, odometerPostUpdate.getRotation().getDegrees());
        assertNotEquals(1.0, odometerPostUpdate.getTranslation().getX());
        assertNotEquals(1.0, odometerPostUpdate.getTranslation().getY());
        assertNotEquals(1.0, odometerPostUpdate.getRotation().getDegrees());

        when(frontLeft.getDrivePosition()).thenReturn(1.0);
        when(frontRight.getDrivePosition()).thenReturn(1.0);
        when(backLeft.getDrivePosition()).thenReturn(1.0);
        when(backRight.getDrivePosition()).thenReturn(1.0);
        frontLeftDrivePosition = frontLeft.getDrivePosition();
        frontRightDrivePosition = frontRight.getDrivePosition();
        backLeftDrivePosition = backLeft.getDrivePosition();
        backRightDrivePosition = backRight.getDrivePosition();
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(frontLeftDrivePosition, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(frontRightDrivePosition, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(backLeftDrivePosition, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(backRightDrivePosition, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();

        var odometerSecondPostUpdate = swerveSubsystem.getOdometer().getPoseMeters();
        assertEquals(1.0, odometerSecondPostUpdate.getTranslation().getX());
        assertEquals(0.0, odometerSecondPostUpdate.getTranslation().getY());
        assertEquals(0.0, odometerSecondPostUpdate.getRotation().getDegrees());
        assertNotEquals(0.0, odometerSecondPostUpdate.getTranslation().getX());
        assertNotEquals(1.0, odometerSecondPostUpdate.getTranslation().getY());
        assertNotEquals(90.0, odometerSecondPostUpdate.getRotation().getDegrees());
    }
}
