package frc.robot.subsystems;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opencv.core.Point;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.math.BigDecimal;
import java.math.RoundingMode;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import frc.robot.extras.SwerveModule;

public class TestSwerveSubsystem {
    private Pigeon2 pigeon;
    private StatusSignal<Angle> yawStatusSignal, pitchStatusSignal, rollStatusSignal, absEncodedrSignal;
    private SwerveSubsystem swerveSubsystem;

    //GenericEntrys are mocked to avoid null pointer exceptions
    private GenericEntry headingShuffleBoard, odometerShuffleBoard, rollSB, pitchSB;

    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private BigDecimal allowedVariation = new BigDecimal(0.01);
    private PositionFilteringSubsystem positionFilteringSubsystem;

    //This is the setup for the tests @BeforeEach means that this method will run before each test
    @BeforeEach
    public void setUp() {
        pigeon = mock();
        when(pigeon.getRotation2d()).thenReturn(Rotation2d.fromDegrees(0));

        frontLeft = mock();
        frontRight = mock();
        backLeft = mock();
        backRight = mock();
        
        positionFilteringSubsystem = mock();

        when(positionFilteringSubsystem.getFilteredBotPose(any())).thenReturn(new Pose2d(0, 0, new Rotation2d(0)));


        //this is to appease shuffleboard in periodic 
        mockSuffleBoard();

        swerveSubsystem = new SwerveSubsystem(pigeon, 
        frontLeft, frontRight, backLeft, backRight, 
        headingShuffleBoard, odometerShuffleBoard, rollSB, pitchSB, positionFilteringSubsystem);
    }
    private void mockSuffleBoard() {
        yawStatusSignal = mock();
        when(pigeon.getYaw()).thenReturn(yawStatusSignal);
        pitchStatusSignal = mock();
        when(pigeon.getPitch()).thenReturn(pitchStatusSignal);
        rollStatusSignal = mock();
        when(pigeon.getRoll()).thenReturn(rollStatusSignal);
        headingShuffleBoard = mock();
        odometerShuffleBoard = mock();
        rollSB = mock();
        pitchSB = mock();
        frontLeft.absoluteEncoder = mock();
        frontRight.absoluteEncoder = mock();
        backLeft.absoluteEncoder = mock();
        backRight.absoluteEncoder = mock();
        absEncodedrSignal = mock();
        when(frontLeft.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(frontRight.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(backLeft.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
        when(backRight.absoluteEncoder.getAbsolutePosition()).thenReturn(absEncodedrSignal);
    }
    @Test
    public void testIsAtAngle(){ //runs test for isAtAngle method
        assertTrue(swerveSubsystem.isAtAngle(new Rotation2d()));
        when(pigeon.getRotation2d()).thenReturn(Rotation2d.fromDegrees(1.6));
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
        //needed to update odometry when mocking swerve modules
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)));
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)));
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)));
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)));

        when(pigeon.getRotation2d()).thenReturn(Rotation2d.fromDegrees(90));
        swerveSubsystem.periodic();
        assertTest(0.0, 0.0, 90.0);

        when(pigeon.getRotation2d()).thenReturn(Rotation2d.fromDegrees(0));
        swerveSubsystem.periodic();
        assertTest(0.0, 0.0, 0.0);
    }
    @Test
    public void testOdometryTranslation(){ //runs test for changing drive position and then updating odometry in periodic
        //First test is with default values (Aka 0.0)
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        assertTest(0.0, 0.0, 0.0);

        //Second test is with the robot moving one meter in the x direction
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        assertTest(1.0, 0.0, 0.0);

        //Third test is with robot moving backwards one meter in the x direction
        setUp(); //Resetting the odometry to test the next test
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        assertTest(-1.0, 0.0, 0.0);

        //Attempted next test but it resulted in 6.123233995736766E-17 instead of 0.0 for x
        
        setUp();
        //Fourth test is with the robot moving one meter in the y direction
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        assertTest(0.0, 1.0, 0.0);
        
        
        //Fith test is with the robot moving one meter to the right in the y direction
        setUp();
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(-1.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        assertTest(0.0, -1.0, 0.0);
        

        //Sixth test is with the robot moving one meter in the x and then y direction
        setUp();
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        //if moving in multiple directions, distanceMeters should start at previous value and add/subtract the new value
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(2.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(2.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(2.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(2.0, Rotation2d.fromDegrees(90)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        assertTest(1.0, 1.0, 0.0);

        //* rotation is not perfect so some variability is allowed
        //Seventh test is with the robot moving two meters diagonally to the right
        setUp();
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(5.0, Rotation2d.fromDegrees(53.13)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(5.0, Rotation2d.fromDegrees(53.13)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(5.0, Rotation2d.fromDegrees(53.13)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(5.0, Rotation2d.fromDegrees(53.13)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        assertTest(3.0, 4.0, 0.0);
        
        
        
        //Eighth test is with math for a 45 degree angle
        setUp();
        when(frontLeft.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)));// 0 = 100% x movement, 90 = 100% y movement
        when(frontRight.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)));// 0 = 100% x movement, 90 = 100% y movement
        when(backLeft.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)));// 0 = 100% x movement, 90 = 100% y movement
        when(backRight.getPosition()).thenReturn(new SwerveModulePosition(1.0, Rotation2d.fromDegrees(45)));// 0 = 100% x movement, 90 = 100% y movement
        swerveSubsystem.periodic();
        double expectedXY = Math.sqrt(.5);
        assertTest(expectedXY, expectedXY, 0.0);
        
    }
    private void assertTest(double expectedXD, double expectedYD, double expectedRotationD){
        var odometerPostUpdate = swerveSubsystem.getOdometer().getPoseMeters();
        var expectedX = new BigDecimal(expectedXD).setScale(2, RoundingMode.HALF_UP);
        var expectedY = new BigDecimal(expectedYD).setScale(2, RoundingMode.HALF_UP);
        var expectedRotation = new BigDecimal(expectedRotationD).setScale(2, RoundingMode.HALF_UP);
        var realX = new BigDecimal(odometerPostUpdate.getTranslation().getX()).setScale(2, RoundingMode.HALF_UP);
        var realY = new BigDecimal(odometerPostUpdate.getTranslation().getY()).setScale(2, RoundingMode.HALF_UP);
        var realRotation = new BigDecimal(odometerPostUpdate.getRotation().getDegrees()).setScale(2, RoundingMode.HALF_UP);
        assertEquals(expectedX, realX);
        assertEquals(expectedY, realY);
        assertEquals(expectedRotation, realRotation);
        assertNotEquals(expectedX.add(allowedVariation), realX);
        assertNotEquals(expectedY.add(allowedVariation), realY);
        assertNotEquals(expectedRotation.add(allowedVariation), realRotation);
    }
}