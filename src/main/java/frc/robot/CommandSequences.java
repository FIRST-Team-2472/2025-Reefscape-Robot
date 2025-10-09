package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.AutoCoralDispenseCommand;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.CollectCoralCmd;
import frc.robot.commands.HoldElevatorCommand;
import frc.robot.commands.defaultCommands.SwerveDriveToPointCmd;
import frc.robot.commands.SwerveFollowTransitionCmd;
import frc.robot.extras.PosPose2d;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandSequences {
    PosPose2d[] cageNodes = new PosPose2d[6];
    Map<Character, PosPose2d> reefNodesMap = new HashMap<>(); // replaced reefNodes

    PosPose2d leftHumanPlayer, rightHumanPlayer, middle, rightReefPassage, leftReefPassage;
    PosPose2d processor;

    public CommandSequences() {
        // x is centered on starting line
        cageNodes[0] = simplePose(7.114, 7.279, 180); //Cage on far left from driver POV
        cageNodes[1] = simplePose(7.114, 6.165, 180); //Cage Position 2
        cageNodes[2] = simplePose(7.114, 5.077, 180); //Cage Position 3
        cageNodes[3] = simplePose(7.114, 2.929, 180); //Cage Position 4
        cageNodes[4] = simplePose(7.114, 1.898, 180); //Cage Position 5
        cageNodes[5] = simplePose(7.114, 0.794, 180); //Cage on far right from driver POV


        reefNodesMap.put('A', simplePose(3.168, 4.190, 0));
        reefNodesMap.put('B', simplePose(3.168, 3.860, 0));
        reefNodesMap.put('C', simplePose(3.682, 2.958, 60));
        reefNodesMap.put('D', simplePose(3.97, 2.798, 60));
        reefNodesMap.put('E', simplePose(5.003, 2.796, 120));
        reefNodesMap.put('F', simplePose(5.294, 2.962, 120));
        reefNodesMap.put('G', simplePose(5.81, 3.86, 180));
        reefNodesMap.put('H', simplePose(5.79, 4.2, 180));
        reefNodesMap.put('I', simplePose(5.292, 5.094, 240)); 
        reefNodesMap.put('J', simplePose(5.006, 5.253, 240));
        reefNodesMap.put('K', simplePose(3.972, 5.249, 300));
        reefNodesMap.put('L', simplePose(3.686, 5.085, 300));


        leftHumanPlayer = simplePose(1.127, 6.982, 306);
        rightHumanPlayer = simplePose(1.127, 1.035, 54);

        middle =  simplePose(7.115, 4, 180);
        rightReefPassage = simplePose(5.8, 1.7, 75);
        leftReefPassage = simplePose(5.8, 7, 285);

        processor = simplePose(6.350, 0.550,270);
    }

    //test auto to show how coral collection works not to be used
    public Command coralcollectionautotest(CoralDispenserSubsystem coralDispenserSubsystem){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new CollectCoralCmd(coralDispenserSubsystem),
                RobotStatus.seeCoral == true ?
                    new SwerveDriveToPointCmd(null, reefNodesMap.get('C')) : null
            )
        );
    }
    //Starting Position to reef

    public Command CageOneToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[0].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('H'));
    }

    public Command setOdodmeteryToTest(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(simplePose(2.18, 4, 0));
        return null;
    }

    public Command CageOneToI(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[0].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('I'));
    }

    public Command CageTwoToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[1].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('H'));
    }

    public Command CageTwoToI(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[1].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('I'));
    }

    public Command CageThreeToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[2].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('H'));
    }

    public Command CageThreeToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[2].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('G'));
    }

    public Command MiddleToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(middle.toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('H'));
    }

    public Command MiddleToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(middle.toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('G'));
    }

    public Command CageFourToH(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[3].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('H'));
    }

    public Command CageFourToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[3].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('G'));
    }

    public Command CageFourToF(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[3].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('F'));
    }

    public Command CageFiveToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[4].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('G'));
    }

    public Command CageFiveToF(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[4].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('F'));
    }

    public Command CageSixToG(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[5].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('G'));
    }

    public Command CageSixToF(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(cageNodes[5].toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('F'));
    }

    //Reef to Source

    public Command HToLeftPlayer(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(reefNodesMap.get('H').toFieldPose2d());
        return new SwerveFollowTransitionCmd(swerveSubsystem, leftReefPassage, leftHumanPlayer, 1);
    }

    public Command GToRightPlayer(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(reefNodesMap.get('G').toFieldPose2d());
        return new SwerveFollowTransitionCmd(swerveSubsystem, rightReefPassage, rightHumanPlayer, 1);
    }

    public Command FToRightPlayer(SwerveSubsystem swerveSubsystem){
        //swerveSubsystem.setPoseEstimator(reefNodesMap.get('F').toFieldPose2d());
        return new SwerveFollowTransitionCmd(swerveSubsystem, rightReefPassage, rightHumanPlayer, 1);
    }

    public Command IToLeftPlayer(SwerveSubsystem swerveSubsystem){
        //swerveSubsystem.setPoseEstimator(reefNodesMap.get('I').toFieldPose2d());
        return new SwerveFollowTransitionCmd(swerveSubsystem, leftReefPassage, leftHumanPlayer, 1);
        //return new SwerveDriveToPointCmd(swerveSubsystem, leftHumanPlayer);
    }

    //Source to Reef

    public Command RightPlayerToD(SwerveSubsystem swerveSubsystem){
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('D'));
    }

    public Command LeftPlayerToK(SwerveSubsystem swerveSubsystem){
        return new SwerveFollowTransitionCmd(swerveSubsystem, leftReefPassage, reefNodesMap.get('K'), 1);
    }

    public Command RightPlayerToC(SwerveSubsystem swerveSubsystem){
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('C'));
    }

    public Command LeftPlayerToL(SwerveSubsystem swerveSubsystem){
        return new SwerveDriveToPointCmd(swerveSubsystem, reefNodesMap.get('L'));
    }

    //Intake Coral

    public Command collectCoral(CoralDispenserSubsystem coralDispenserSubsystem){
        return new CollectCoralCmd(coralDispenserSubsystem);
    }

    //Elevator Commands

    public Command placeOnReef(ElevatorSubsystem elevatorSubsystem, CoralDispenserSubsystem coralDispenserSubsystem, double elevatorHeight) {
        return new SequentialCommandGroup(
            new AutoElevatorCommand(elevatorSubsystem, elevatorHeight),
            new ParallelDeadlineGroup(
                new AutoCoralDispenseCommand(coralDispenserSubsystem)/* ,
                new HoldElevatorCommand(elevatorSubsystem)*/
            )
        );
    }

    public Command setElevatorL0(ElevatorSubsystem elevatorSubsystem){
        return new AutoElevatorCommand(elevatorSubsystem, 0);
    }

    public Command setElevatorL1(ElevatorSubsystem elevatorSubsystem){
        return new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL1Height);
    }

    public Command setElevatorL2(ElevatorSubsystem elevatorSubsystem){
        return new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL2Height);
    }

    public Command setElevatorL3(ElevatorSubsystem elevatorSubsystem){
        return new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL3Height);
    }

    public Command setElevatorL4(ElevatorSubsystem elevatorSubsystem){
        return new AutoElevatorCommand(elevatorSubsystem, ElevatorConstants.kElevatorL4Height);
    }

    //Coral Dispenser Command

    public Command dispenseCoral(CoralDispenserSubsystem coralDispenserSubsystem) {
        return new AutoCoralDispenseCommand(coralDispenserSubsystem);
    }

    //Things below this comment are not used in the code and need testing

    public Command swerveFollowTransitionTest(SwerveSubsystem swerveSubsystem){
        swerveSubsystem.setPoseEstimator(simplePose(2, 2, 0).toFieldPose2d());

        return new SequentialCommandGroup(
            new SwerveFollowTransitionCmd(swerveSubsystem, simplePose(3.4, .6, 0), simplePose(5, 2, 0), 1)
        );
    }

    public Command driveForwardTest(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.setPoseEstimator(simplePose(1, 0, 0).toFieldPose2d());
        return new SwerveDriveToPointCmd(swerveSubsystem, simplePose(3, 0, 0));
    }

    public PosPose2d simplePose(double x, double y, double angleDegrees) {
        return new PosPose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
}