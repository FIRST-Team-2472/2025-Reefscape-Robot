package frc.robot.extras;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DrivePoint extends Translation2d {
    // drive position -> only one that can be directly feed to the odometer. The
    // only postion the odometer can actually use

    public DrivePoint() {
        super();
    }

    public DrivePoint(double m_x, double m_y) {
        super(m_x, m_y);
    }

    public DrivePoint(Translation2d point) {
        super(point.getX(), point.getY());
    }

    public FieldPoint toFieldPos() {
        if (SwerveSubsystem.isOnRed()) {
            return new FieldPoint(SensorConstants.sizeOfFieldMetersX - getX(), getY());
        }
        return new FieldPoint(getX(), Math.abs(getY()));
    }

    public PositivePoint toPositivePos() {
        if (SwerveSubsystem.isOnRed()) {
            return new PositivePoint(getX(), getY());
        }
        return new PositivePoint(getX(), Math.abs(getY()));
    }
}
