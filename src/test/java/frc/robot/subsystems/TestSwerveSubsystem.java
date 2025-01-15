package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class TestSwerveSubsystem {
    @Test
    public void testExampleCondition(){
        Pigeon2 pigeon = mock();
        StatusSignal statusSignal = mock();
        when(statusSignal.getValueAsDouble()).thenAnswer(0.0);
        when(pigeon.getYaw()).then(StatusSignal.)
        var swerveSubsystem = new SwerveSubsystem(pigeon);

        swerveSubsystem.isAtAngle(new Rotation2d());
    }
}
