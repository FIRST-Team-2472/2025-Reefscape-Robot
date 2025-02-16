package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PosPose2d extends Pose2d {
    // positive/blue position -> simplfyed position for comparisions to auto points


    public PosPose2d() {
        super();
    }

    public PosPose2d(double x, double y, Rotation2d angle) {
        super(x, y, angle);
    }

    public PosPose2d(PositivePoint posPoint, Rotation2d angle) {
        super(posPoint, angle);
    }

    public PositivePoint getPositivePoint() {
        return new PositivePoint(getX(), getY());
    }

    public FieldPose2d toFieldPose2d() {
        if (SwerveSubsystem.isOnRed()) {
            return new FieldPose2d(SensorConstants.sizeOfFieldMetersX - getX(), SensorConstants.sizeOfFieldMetersY - getY(),  Rotation2d.fromDegrees((-getRotation().getDegrees())+180));
        }
        return new FieldPose2d(getX(), getY(), getRotation());
    }

    public DrivePose2d toDrivePose2d() {
        if (SwerveSubsystem.isOnRed()) {
            return new DrivePose2d(getX(), getY(), getRotation());
        }
        return new DrivePose2d(getX(), -Math.abs(getY()), getRotation());
    }
}
