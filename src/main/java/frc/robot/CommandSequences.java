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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.defaultCommands.SwerveDriveToPointCmd;
import frc.robot.extras.PosPose2d;
import frc.robot.extras.PositivePoint;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandSequences {
    PosPose2d[] startingNodes = new PosPose2d[6];
    PosPose2d[] reefNodes = new PosPose2d[6];
    PosPose2d[] hpStations = new PosPose2d[2];
    PosPose2d[] processor = new PosPose2d[1];

    public CommandSequences() {
        startingNodes[0] = simplePose(7.671, 7.279, 0); //Cage position 1
        startingNodes[1] = simplePose(7.671, 6.145, 0); //Cage Position 2
        startingNodes[2] = simplePose(7.671, 5.077, 0); //Cage Position 3
        startingNodes[3] = simplePose(7.671, 3.929, 0); //Cage Position 4
        startingNodes[4] = simplePose(7.671, 1.898, 0); //Cage Position 5
        startingNodes[5] = simplePose(7.671, 0.794, 0); //Cage Position 6

        reefNodes[0] = simplePose(6.055, 4.025, 0); //Reef Position 1
        reefNodes[1] = simplePose(5.329, 5.366, 60); //Reef Position 2
        reefNodes[2] = simplePose(3.676, 5.374, 120); //Reef Position 3
        reefNodes[3] = simplePose(2.950, 4.025, 180); //Reef Position 4
        reefNodes[4] = simplePose(3.706, 2.646, 240); //Reef Position 5
        reefNodes[5] = simplePose(5.277, 2.654, 300); //Reef Position 6

        hpStations[0] = simplePose(1.127, 0.962, 54);
        hpStations[1] = simplePose(1.118, 7.106, 306);

        processor[0] = simplePose(2, 7,90);
    }

    public Command test(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.setOdometry(simplePose(7.589, 3.929, 0));

        return new SequentialCommandGroup(
            generatePath(swerveSubsystem, startingNodes[0], List.of(), startingNodes[4]));
    }

    public Command swervePointTest(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.setOdometry(simplePose(0, 0, 0).toFieldPose2d()); //X = 7.671, Y = 5.077, angle = 0

        return new SequentialCommandGroup(
          new SwerveDriveToPointCmd(swerveSubsystem, simplePose(1, 2, 90))  //X = 6.055, Y = 4.025, angle = 0
        );
    }

    // generates a path via points
    private static Command generatePath(SwerveSubsystem swerveSubsystem, PosPose2d startPoint,
            List<PositivePoint> midPoints,
            PosPose2d endPoint) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Pose2d driveStartPoint = startPoint.toFieldPose2d();
        Pose2d driveEndPoint = endPoint.toFieldPose2d();
        List<Translation2d> driveMidPoints = new ArrayList<Translation2d>();
        for (int i = 0; i < midPoints.size(); i++)
            driveMidPoints.add(midPoints.get(i).toFieldPos());

        // 2. Generate trajectory
        // Generates trajectory. Need to feed start point, a series of inbetween points,
        // and end point
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                driveStartPoint,
                driveMidPoints,
                driveEndPoint,
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                // swerveSubsystm::getPose is same as () -> swerveSubsystem.getPose()
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        // creates a Command list that will reset the Odometry, then move the path, then
        // stop
        return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public PosPose2d simplePose(double x, double y, double angleDegrees) {
        return new PosPose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
    public static Rotation2d teamChangeAngle(double degrees){
        if(SwerveSubsystem.isOnRed())
                return  Rotation2d.fromDegrees(degrees+180);
        return  Rotation2d.fromDegrees(degrees);
    }
}
