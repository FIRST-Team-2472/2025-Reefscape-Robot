package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
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
import frc.robot.subsystems.PositionFilteringSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandSequences {
    PosPose2d[] cageNodes = new PosPose2d[6];
    PosPose2d[] reefNodes = new PosPose2d[12];
    PosPose2d leftHumanPlayer, rightHumanPlayer, middle, rightReefPassage, leftReefPassage;
    PosPose2d processor;

    public CommandSequences() {
        // x is centered on starting line
        cageNodes[0] = simplePose(7.114, 7.279, 180); //Cage on far left from driver POV
        cageNodes[1] = simplePose(7.114, 6.145, 180); //Cage Position 2
        cageNodes[2] = simplePose(7.114, 5.077, 180); //Cage Position 3
        cageNodes[3] = simplePose(7.114, 2.929, 180); //Cage Position 4
        cageNodes[4] = simplePose(7.114, 1.898, 180); //Cage Position 5
        cageNodes[5] = simplePose(7.114, 0.794, 180); //Cage on far right from driver POV


        reefNodes[0] = simplePose(3.154, 4.190, 0); //Reef Position A
        reefNodes[1] = simplePose(3.154, 3.860, 0); //Reef Position B
        reefNodes[2] = simplePose(3.676, 2.955, 60); //Reef Position C
        reefNodes[3] = simplePose(3.961, 2.788, 60); //Reef Position D
        reefNodes[4] = simplePose(5.012, 2.788, 120); //Reef Position E
        reefNodes[5] = simplePose(5.297, 2.951, 120); //Reef Position F
        reefNodes[6] = simplePose(5.823, 3.86, 180); //Reef Position G
        reefNodes[7] = simplePose(5.823, 4.192, 180); //Reef Position H
        reefNodes[8] = simplePose(5.298, 5.099, 240); //Reef Position I
        reefNodes[9] = simplePose(5.012, 5.262, 240); //Reef Position J // was 5, 
        reefNodes[10] = simplePose(3.962, 5.263, 300); //Reef Position K
        reefNodes[11] = simplePose(3.677, 5.1, 300); //Reef Position L


        leftHumanPlayer = simplePose(1.097, 6.991, 306);
        rightHumanPlayer = simplePose(1.127, 0.962, 54);

        middle =  simplePose(7.125, 4, 180);
        rightReefPassage = simplePose(5, 1, 75);
        leftReefPassage = simplePose(5, 7, 285);

        processor = simplePose(2, 7,90);
    }

    //test auto to show how coral collection works not to be used
    public Command coralcollectionautotest(CoralDispenserSubsystem coralDispenserSubsystem){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new CollectCoralCmd(coralDispenserSubsystem),
                coralDispenserSubsystem.seecoral == true ?
                    new SwerveDriveToPointCmd(null, reefNode('C')) : null
            )
        );
    }
    //Starting Position to reef

    public Command CageOneToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[0].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('H'));
    }

    public Command CageOneToI(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[0].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('I'));
    }

    public Command CageTwoToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[1].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('H'));
    }

    public Command CageTwoToI(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[1].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('I'));
    }

    public Command CageThreeToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[2].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('H'));
    }

    public Command CageThreeToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[2].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('G'));
    }

    public Command MiddleToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(middle.toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('H'));
    }
    
    public Command MiddleToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(middle.toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('G'));
    }

    public Command CageFourToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[3].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('H'));
    }

    public Command CageFourToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[3].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('G'));
    }

    public Command CageFourToF(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[3].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('F'));
    }

    public Command CageFiveToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[4].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('G'));
    }

    public Command CageFiveToF(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[4].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('F'));
    }

    public Command CageSixToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[5].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('G'));
    }

    public Command CageSixToF(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(cageNodes[5].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNode('F'));
    }

    //Reef to Source
    
    public Command HToLeftPlayer(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(reefNode('H').toFieldPose2d());
        return new SwerveFollowTransitionCmd(swerveSubsystem, leftReefPassage, leftHumanPlayer, 1);
    }

    public Command GToRightPlayer(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(reefNode('G').toFieldPose2d());
        return new SwerveFollowTransitionCmd(swerveSubsystem, rightReefPassage, rightHumanPlayer, 1);
    }

    public Command FToRightPlayer(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(reefNode('F').toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, leftHumanPlayer);
    }

    public Command IToRightPlayrt(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(reefNode('I').toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, leftHumanPlayer);
    }

    //Intake Coral

    public Command collectCoral(CoralDispenserSubsystem coralDispenserSubsystem){
        return new CollectCoralCmd(coralDispenserSubsystem);
    }

    //Elevator Commands

    public Command PlaceOnL4(ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem) {
        return new SequentialCommandGroup(
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem)
        );
    }

    public Command PlaceOnL3(ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem) {
        return new SequentialCommandGroup(
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL3Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem)
        );
    }

    public Command PlaceOnL2(ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem) {
        return new SequentialCommandGroup(
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL2Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem)
        );
    }

    public Command PlaceOnL1(ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem) {
        return new SequentialCommandGroup(
            new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL1Height),
            new AutoCoralDispenseCommand(coralDispenserSubsystem)
        );
    }

    //Coral Dispenser Command

    public Command dispenseCoral(CoralDispenserSubsystem coralDispenserSubsystem) {
        return new AutoCoralDispenseCommand(coralDispenserSubsystem);
    }

    //Things below this comment are not used in the code and need testing

    public Command swerveFollowTransitionTest(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setOdometry(simplePose(2, 2, 0).toFieldPose2d());

        return new SequentialCommandGroup(
            new SwerveFollowTransitionCmd(swerveSubsystem, simplePose(3.4, .6, 0), simplePose(5, 2, 0), 1)
        );
    }

    // TODO: Works on Blue but not Red
    public Command driveForward(SwerveSubsystem swerveSubsystem, PositionFilteringSubsystem positionFilteringSubsystem) {
        swerveSubsystem.setOdometry(middle.toFieldPose2d());
        swerveSubsystem.calibrateOdometry(0.0f);
        Pose2d currentPos = swerveSubsystem.getOdometer().getPoseMeters();
        return new SwerveDriveToPointCmd(swerveSubsystem, simplePose(currentPos.getX() - 0.9f, currentPos.getY(), currentPos.getRotation().getDegrees()));
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
