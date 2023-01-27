package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyroscope {
    public Rotation2d getPitch();

    public Rotation2d getYaw();

    public Rotation2d getRoll();
}
