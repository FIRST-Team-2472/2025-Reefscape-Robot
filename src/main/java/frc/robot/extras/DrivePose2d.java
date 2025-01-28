package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DrivePose2d extends Pose2d{
    // drive position -> only one that can be directly feed to the odometer. The
    // only postion the odometer can actually use

    public DrivePose2d() {
        super();
    }

    public DrivePose2d(double x, double y, Rotation2d angle) {
        super(x, y, angle);
    }

    public DrivePose2d(PositivePoint posPoint, Rotation2d angle) {
        super(posPoint, angle);
    }

    public DrivePoint getDrivePoint() {
        return new DrivePoint(getX(), getY());
    }

    public FieldPose2d toFieldPose2d() {
        if (SwerveSubsystem.isOnRed()) {
            return new FieldPose2d(SensorConstants.sizeOfFieldMeters - getX(), getY(), getRotation());
        }
        return new FieldPose2d(getX(), Math.abs(getY()), getRotation());
    }

    public PosPose2d toPosPose2d() {
        if (SwerveSubsystem.isOnRed()) {
            return new PosPose2d(getX(), getY(), getRotation());
        }
        return new PosPose2d(getX(), Math.abs(getY()), getRotation());
    }
}
