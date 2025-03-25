package frc.robot.auto;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;

public class AutoMap {
    public final Map<String, Command> autoCommands = new HashMap<>();
    public final Map<String, Command> testCommands = new HashMap<>();

    CommandSequences commandSequences;
    SwerveSubsystem swerveSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    CoralDispenserSubsystem coralDispenserSubsystem;

    {
        // Initialize the map with command sequences
        autoCommands.put("MiddleToH", new SequentialCommandGroup(commandSequences.MiddleToH(swerveSubsystem)));
        
        autoCommands.put("MiddleToHL1", new SequentialCommandGroup(
                commandSequences.MiddleToH(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        autoCommands.put("MiddleToHL4", new SequentialCommandGroup(
                commandSequences.MiddleToH(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        autoCommands.put("CageTwoToIL4", new SequentialCommandGroup(
                commandSequences.CageTwoToI(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        autoCommands.put("CageThreeToGL4", new SequentialCommandGroup(
                commandSequences.CageThreeToG(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        autoCommands.put("CageFiveToFL4", new SequentialCommandGroup(
                commandSequences.CageFiveToF(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        autoCommands.put("DriveForwardTest", new SequentialCommandGroup(
                commandSequences.driveForwardTest(swerveSubsystem)));

        testCommands.put("coralcollectionautotest", new SequentialCommandGroup(
                commandSequences.coralcollectionautotest(coralDispenserSubsystem)));

        testCommands.put("CageThreeToHL1", new SequentialCommandGroup(
                commandSequences.CageThreeToH(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("MiddleToGL1", new SequentialCommandGroup(
                commandSequences.MiddleToG(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("MiddleToGL4", new SequentialCommandGroup(
                commandSequences.MiddleToG(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        testCommands.put("CageThreeToHL4", new SequentialCommandGroup(
                commandSequences.CageThreeToH(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        testCommands.put("CageThreeToGL1", new SequentialCommandGroup(
                commandSequences.CageThreeToG(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("CageThreeToGL4", new SequentialCommandGroup(
                commandSequences.CageThreeToG(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        testCommands.put("CageFourToHL1", new SequentialCommandGroup(
                commandSequences.CageFourToH(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("CageFourToHL4", new SequentialCommandGroup(
                commandSequences.CageFourToH(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        testCommands.put("CageFourToGL1", new SequentialCommandGroup(
                commandSequences.CageFourToG(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("CageFourToGL4", new SequentialCommandGroup(
                commandSequences.CageFourToG(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        testCommands.put("CageOneToIL1", new SequentialCommandGroup(
                commandSequences.CageOneToI(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("CageOneToIL4", new SequentialCommandGroup(
                commandSequences.CageOneToI(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        testCommands.put("CageTwoToIL1", new SequentialCommandGroup(
                commandSequences.CageTwoToI(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("CageFiveToFL1", new SequentialCommandGroup(
                commandSequences.CageFiveToF(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("CageSixToFL1", new SequentialCommandGroup(
                commandSequences.CageSixToF(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL1Height)));

        testCommands.put("CageSixToFL4", new SequentialCommandGroup(
                commandSequences.CageSixToF(swerveSubsystem),
                commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                        ElevatorConstants.kElevatorL4Height)));

        testCommands.put("setOdodmetryToTest",
                new InstantCommand(() -> commandSequences.setOdodmeteryToTest(swerveSubsystem)));

        testCommands.put("CageFiveToFL4ToCL4", new SequentialCommandGroup(
                new SequentialCommandGroup(
                        commandSequences.CageFiveToF(swerveSubsystem),
                        commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                                ElevatorConstants.kElevatorL4Height)),
                new ParallelCommandGroup(
                        commandSequences.FToRightPlayer(swerveSubsystem),
                        commandSequences.setElevatorL0(elevatorSubsystem)),
                new SequentialCommandGroup(
                        commandSequences.collectCoral(coralDispenserSubsystem),
                        commandSequences.RightPlayerToD(swerveSubsystem),
                        commandSequences.placeOnReef(elevatorSubsystem, coralDispenserSubsystem,
                                ElevatorConstants.kElevatorL4Height))));
    }
}
