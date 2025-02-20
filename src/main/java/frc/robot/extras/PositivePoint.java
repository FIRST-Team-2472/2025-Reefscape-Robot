package frc.robot.extras;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PositivePoint extends Translation2d {
    // positive/blue position -> simplfyed position for comparisions to auto points

    public PositivePoint() {
        super();
    }

    //These values are expected to be positive
    public PositivePoint(double m_x, double m_y) {
        super(m_x, m_y);
    }

    public PositivePoint(Translation2d point) {
        super(point.getX(), point.getY());
    }

    public FieldPoint toFieldPos() {
        if (SwerveSubsystem.isOnRed()) {
            return new FieldPoint(SensorConstants.sizeOfFieldMetersX - getX(), getY());
        }
        return new FieldPoint(getX(), getY());
    }

    public DrivePoint toDrivePos() {
        if (SwerveSubsystem.isOnRed()) {
            return new DrivePoint(getX(), getY());
        }
        return new DrivePoint(getX(), -Math.abs(getY()));
    }
}