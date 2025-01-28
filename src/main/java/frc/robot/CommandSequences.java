package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

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
import frc.robot.subsystems.SwerveSubsystem;

public class CommandSequences {

    Pose2d[] miscellaneousNodes = new Pose2d[4];
    Pose2d[] importantNodes = new Pose2d[6];
    Pose2d[] startingNodes = new Pose2d[5];
    Pose2d[] collectingNearNodes = new Pose2d[3];
    Pose2d[] shootingNearNodes = new Pose2d[3];

    public CommandSequences() {
    startingNodes[0] = simplePose(7.589, 3.929, 0);
    startingNodes[4] = simplePose(5.589, 3.929, 0);
    }

    public Command test(SwerveSubsystem swerveSubsystem) {

        swerveSubsystem.setOdometry(simplePose(7.589, 3.929, 0));

        return new SequentialCommandGroup(
            generatePath(swerveSubsystem, startingNodes[0], List.of(), startingNodes[4]));
    }

        // generates a path via points
        private static Command generatePath(SwerveSubsystem swerveSubsystem, Pose2d startPoint,
        List<Waypoint> midPoints,
        Pose2d endPoint) {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    Pose2d driveStartPoint = startPoint;
    Pose2d driveEndPoint = endPoint;
    List<Translation2d> driveMidPoints = new ArrayList<Translation2d>();
    for (int i = 0; i < midPoints.size(); i++)
        driveMidPoints.add(midPoints.get(i).anchor());

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

    public Pose2d simplePose(double x, double y, double angleDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
    public static Rotation2d teamChangeAngle(double degrees){
            if(SwerveSubsystem.isOnRed())
                return  Rotation2d.fromDegrees(-degrees+180);
        return  Rotation2d.fromDegrees(degrees);
    }

}
