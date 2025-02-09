package frc.robot.extras;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class FieldPoint extends Translation2d {
    // field position -> the actual postion of the robot on the field. photon vision
    // returns this

    public FieldPoint() {
        super();
    }

    public FieldPoint(double m_x, double m_y) {
        super(m_x, m_y);
    }

    public FieldPoint(Translation2d point) {
        super(point.getX(), point.getY());
    }

    public DrivePoint toDrivePos() {
        if (SwerveSubsystem.isOnRed()) {
            return new DrivePoint(SensorConstants.sizeOfFieldMeters - getX(), getY());
        }
        return new DrivePoint(getX(), -Math.abs(getY()));
    }

    public PositivePoint toPositivePos() {
        if (SwerveSubsystem.isOnRed()) {
            return new PositivePoint(SensorConstants.sizeOfFieldMeters - getX(), getY());
        }
        return new PositivePoint(getX(), getY());
    }
}