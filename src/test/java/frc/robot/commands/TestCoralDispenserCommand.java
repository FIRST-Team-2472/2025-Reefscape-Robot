package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.function.Supplier;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.defaultCommands.CoralDispenserCommand;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TestCoralDispenserCommand {
    CoralDispenserSubsystem coralDispenserSubsystem = mock();
    Supplier<Double> xboxControllerRightTrigger = mock();
    Supplier<Double> xboxControllerLeftTrigger = mock();
    SwerveSubsystem swerveSubsystem = mock();
    CoralDispenserCommand coralDispenserCommand = new CoralDispenserCommand(coralDispenserSubsystem, xboxControllerRightTrigger, xboxControllerLeftTrigger, swerveSubsystem);

    @BeforeEach
    public void setup() {

    }
    @Test
    public void isNearCoralStationTrue() {
        when(swerveSubsystem.getPose()).thenReturn(new Pose2d(1,1,mock()));
        assertTrue(coralDispenserCommand.isNearCoralStation());
        when(swerveSubsystem.getPose()).thenReturn(new Pose2d(0,6,mock()));
        assertTrue(coralDispenserCommand.isNearCoralStation());
        when(swerveSubsystem.getPose()).thenReturn(new Pose2d(13,1,mock()));
        assertTrue(coralDispenserCommand.isNearCoralStation());
        when(swerveSubsystem.getPose()).thenReturn(new Pose2d(13,6,mock()));
        assertTrue(coralDispenserCommand.isNearCoralStation());
    }
    @Test
    public void isNearCoralStationFalse() {
        when(swerveSubsystem.getPose()).thenReturn(new Pose2d(1,4,mock()));
        assertFalse(coralDispenserCommand.isNearCoralStation());
        when(swerveSubsystem.getPose()).thenReturn(new Pose2d(13,4,mock()));
        assertFalse(coralDispenserCommand.isNearCoralStation());
        when(swerveSubsystem.getPose()).thenReturn(new Pose2d(4,0,mock()));
        assertFalse(coralDispenserCommand.isNearCoralStation());
        when(swerveSubsystem.getPose()).thenReturn(new Pose2d(11,6,mock()));
        assertFalse(coralDispenserCommand.isNearCoralStation());
    }
}