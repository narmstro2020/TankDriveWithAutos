package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Motor{
    
    public Rotation2d getRotations();

    public Rotation2d getRotationsPerSecond();

    public void setRotationsPerSecond(Rotation2d rotationsPerSecond);

    public void setEncoderPosition(Rotation2d rotations);

    public int getDeviceID();

    public String getCANbus();

    

}
