package frc.robot.subsystems;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class TestSwerveSubsystem {
    Pigeon2 pigeon = mock();
    //needs to use typeparameter Angle to align with getYaw
    StatusSignal<Angle> statusSignal = mock();
    @Test
    public void testIsAtAngle(){
        // we needed to use thenReturn
        when(statusSignal.getValueAsDouble()).thenReturn(0.0);
        when(pigeon.getYaw()).thenReturn(statusSignal);
        var swerveSubsystem = new SwerveSubsystem(pigeon);
        assertTrue(swerveSubsystem.isAtAngle(new Rotation2d()));
        when(statusSignal.getValueAsDouble()).thenReturn(1.6);
        assertFalse(swerveSubsystem.isAtAngle(new Rotation2d()));
    }
    @Test
    public void testIsAtPoint(){

    }
}
