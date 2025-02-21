package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.AutoCoralDispenseCommand;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.CollectCoralCmd;
import frc.robot.commands.defaultCommands.SwerveDriveToPointCmd;
import frc.robot.commands.SwerveFollowTransitionCmd;
import frc.robot.extras.PosPose2d;
import frc.robot.extras.PositivePoint;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandSequences {
    PosPose2d[] cageNodes = new PosPose2d[6];
    PosPose2d[] reefNodes = new PosPose2d[12];
    PosPose2d leftHumanPlayer, rightHumanPlayer, middle, rightReefPassage, leftReefPassage;
    PosPose2d processor;

    public CommandSequences() {
        // x is centered on starting line
        cageNodes[0] = simplePose(7.58, 7.279, 180); //Cage on far left from driver POV
        cageNodes[1] = simplePose(7.58, 6.145, 180); //Cage Position 2
        cageNodes[2] = simplePose(7.58, 5.077, 180); //Cage Position 3
        cageNodes[3] = simplePose(7.58, 2.929, 180); //Cage Position 4
        cageNodes[4] = simplePose(7.58, 1.898, 180); //Cage Position 5
        cageNodes[5] = simplePose(7.58, 0.794, 180); //Cage on far right from driver POV

        reefNodes[0] = simplePose(3.15, 4.18, 0); //Reef Position A
        reefNodes[1] = simplePose(3.15, 3.85, 0); //Reef Position B
        reefNodes[2] = simplePose(3.7, 2.94, 60); //Reef Position C
        reefNodes[3] = simplePose(3.98, 2.78, 60); //Reef Position D
        reefNodes[4] = simplePose(5, 2.78, 120); //Reef Position E
        reefNodes[5] = simplePose(5.3, 2.960, 120); //Reef Position F
        reefNodes[6] = simplePose(5.8, 3.85, 180); //Reef Position G
        reefNodes[7] = simplePose(5.8, 4.18, 180); //Reef Position H
        reefNodes[8] = simplePose(5.3, 5.09, 240); //Reef Position I
        reefNodes[9] = simplePose(5.01, 5.25, 240); //Reef Position J // was 5, 
        reefNodes[10] = simplePose(3.99, 5.24, 300); //Reef Position K
        reefNodes[11] = simplePose(3.69, 5.1, 300); //Reef Position L

        rightHumanPlayer = simplePose(1.127, 0.962, 54);
        leftHumanPlayer = simplePose(1.097, 6.991, 306);
        middle =  simplePose(7.58, 4, 180);
        rightReefPassage = simplePose(5, 1, 75);
        leftReefPassage = simplePose(5, 7, 285);

        processor = simplePose(2, 7,90);
    }
    public Command driveAndSitFromMiddle(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(middle.toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('G'));
    }
    public Command driveAndPlaceOneFromMiddle(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem){
        swerveSubsystem.setOdometry(middle.toFieldPose2d());
        return new SequentialCommandGroup(
            new SwerveDriveToPointCmd(swerveSubsystem, reefNode('G')),
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem)
        );
    }
    public Command driveAndPlaceOneFromLeft(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem){
        swerveSubsystem.setOdometry(cageNodes[0].toFieldPose2d());
        return new SequentialCommandGroup(
            new SwerveDriveToPointCmd(swerveSubsystem, reefNode('J')),
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem),
            new AutoElevatorCommand(elevatorSubsystem, 0)
        );
    }
    public Command driveAndPlaceOneFromRight(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem){
        swerveSubsystem.setOdometry(cageNodes[5].toFieldPose2d());
        return new SequentialCommandGroup(
            new SwerveDriveToPointCmd(swerveSubsystem, reefNode('E')),
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem),
            new AutoElevatorCommand(elevatorSubsystem, 0)
        );
    }
    public Command driveAndPlaceTwoFromLeft(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem){
        swerveSubsystem.setOdometry(cageNodes[0].toFieldPose2d());
        return new SequentialCommandGroup(
            new SwerveDriveToPointCmd(swerveSubsystem, reefNode('J')),
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem),
            new AutoElevatorCommand(elevatorSubsystem, 0),
            new SwerveFollowTransitionCmd(swerveSubsystem, leftReefPassage, leftHumanPlayer, 1),
            new CollectCoralCmd(coralDispenserSubsystem),// this command is missing stuff
            new SwerveDriveToPointCmd(swerveSubsystem, reefNode('L')),
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem),
            new AutoElevatorCommand(elevatorSubsystem, 0)
        );
    }
    public Command driveAndPlaceTwoFromRight(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem){
        swerveSubsystem.setOdometry(cageNodes[0].toFieldPose2d());
        return new SequentialCommandGroup(
            new SwerveDriveToPointCmd(swerveSubsystem, reefNode('E')),
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem),
            new AutoElevatorCommand(elevatorSubsystem, 0),
            new SwerveFollowTransitionCmd(swerveSubsystem, rightReefPassage, rightHumanPlayer, 1),
            new CollectCoralCmd(coralDispenserSubsystem),// this command is missing stuff
            new SwerveDriveToPointCmd(swerveSubsystem, reefNode('C')),
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem),
            new AutoElevatorCommand(elevatorSubsystem, 0)
        );
    }
    // prob doesnt work anymore
    public Command twoCoralCfourRoneHoneRfive(SwerveSubsystem swerveSubsystem, 
    ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem) {

        swerveSubsystem.setOdometry(cageNodes[2].toFieldPose2d());

        return new SequentialCommandGroup(

            //new ParallelCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, reefNodes[0]),
                //new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height)
            //),
            //new AutoCoralDispenseCommand(coralDispenserSubsystem),
            new ParallelCommandGroup(
                new SwerveDriveToPointCmd(swerveSubsystem, simplePose(6.350, 6.245, 0))
                //new AutoElevatorCommand(elevatorSubsystem, 0)
            ),
            new SwerveDriveToPointCmd(swerveSubsystem, leftHumanPlayer),
            new ParallelCommandGroup(
                //new SwerveDriveToPointCmd(swerveSubsystem, reefNodes[4])
                //new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height)
            ),
            new AutoCoralDispenseCommand(coralDispenserSubsystem)
        );
    }
    public Command swerveFollowTransitionTest(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(simplePose(2, 2, 0).toFieldPose2d());

        return new SequentialCommandGroup(
            new SwerveFollowTransitionCmd(swerveSubsystem, simplePose(3.4, .6, 0), simplePose(5, 2, 0), 1)
        );
    }

    public PosPose2d reefNode(char NodeLetter){
        switch (NodeLetter) {
            case 'A':
                return reefNodes[0];
            case 'B':
                return reefNodes[1];
            case 'C':  
                return reefNodes[2];
            case 'D':
                return reefNodes[3];
            case 'E':
                return reefNodes[4];
            case 'F':
                return reefNodes[5];
            case 'G':
                return reefNodes[6];
            case 'H':
                return reefNodes[7];
            case 'I':
                return reefNodes[8];
            case 'J':
                return reefNodes[9];
            case 'K':
                return reefNodes[10];
            case 'L':
                return reefNodes[11];
            default:
                return reefNodes[0];
        }

    }

    public PosPose2d simplePose(double x, double y, double angleDegrees) {
        return new PosPose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
}
