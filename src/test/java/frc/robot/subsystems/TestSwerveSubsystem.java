package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestSwerveSubsystem {
  Pigeon2 pigeon;
  // needs to use typeparameter Angle to align with getYaw
  StatusSignal<Angle> yawStatusSignal;
  StatusSignal<Angle> pitchStatusSignal;
  StatusSignal<Angle> rollStatusSignal;
  private SwerveSubsystem swerveSubsystem;
  private GenericEntry headingShuffleBoard;
  private GenericEntry odometerShuffleBoard;
  private GenericEntry rollSB;
  private GenericEntry pitchSB;

  /*
  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;
  */

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

    /*
    frontLeft = mock();
    frontRight = mock();
    backLeft = mock();
    backRight = mock();
    */

    swerveSubsystem =
        new SwerveSubsystem(
            pigeon,
            // frontLeft, frontRight, backLeft, backRight,
            headingShuffleBoard,
            odometerShuffleBoard,
            rollSB,
            pitchSB);
  }

  @Test
  public void testIsAtAngle() {
    // we needed to use thenReturn
    when(yawStatusSignal.getValueAsDouble()).thenReturn(0.0);
    assertTrue(swerveSubsystem.isAtAngle(new Rotation2d()));
    when(yawStatusSignal.getValueAsDouble()).thenReturn(1.6);
    assertFalse(swerveSubsystem.isAtAngle(new Rotation2d()));
  }

  @Test
  public void testIsAtPoint() {
    Translation2d targetDrivePos = new Translation2d();
    assertTrue(swerveSubsystem.isAtPoint(targetDrivePos));
    targetDrivePos = new Translation2d(1, 1);
    assertFalse(swerveSubsystem.isAtPoint(targetDrivePos));
  }

  @Test
  public void testOdometryAngle() {
    when(yawStatusSignal.getValueAsDouble()).thenReturn(90.0);
    when(pitchStatusSignal.getValueAsDouble()).thenReturn(0.0);
    when(rollStatusSignal.getValueAsDouble()).thenReturn(0.0);
    swerveSubsystem.periodic();
    var odometerPostUpdate = swerveSubsystem.getOdometer().getPoseMeters();
    assertEquals(90.0, odometerPostUpdate.getRotation().getDegrees());
  }
  // TODO: testOdometryTranslation
  /*
  @Test
  public void testOdometryTranslation(){
      when(yawStatusSignal.getValueAsDouble()).thenReturn(0.0);
      when(pitchStatusSignal.getValueAsDouble()).thenReturn(0.0);
      when(rollStatusSignal.getValueAsDouble()).thenReturn(0.0);
      swerveSubsystem.periodic();
      var odometerPostUpdate = swerveSubsystem.getOdometer().getPoseMeters();
      assertEquals(0.0, odometerPostUpdate.getTranslation().getX());
      assertEquals(0.0, odometerPostUpdate.getTranslation().getY());
  }
  */
}
