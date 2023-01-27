// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.brushed;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Motor;

/** Add your docs here. */
public abstract class NoEncoder extends SubsystemBase implements Motor{

    protected final double maxVoltage = 12.0;
    protected double softwarePosition = 0.0;
    protected double softwareVelocity = 0.0;
    private double lastTime = Timer.getFPGATimestamp();


    @Override
    public void periodic() {
        SmartDashboard.putNumber("MotorRotations", softwarePosition);
        SmartDashboard.putNumber("MotorRotationsPerSecond", softwareVelocity);

        double currentTime = Timer.getFPGATimestamp();
        double deltaT = currentTime - lastTime;

        softwareVelocity = getRotationsPerSecond().getRotations();
        softwarePosition += softwareVelocity * deltaT;

        lastTime = currentTime;
    }

    @Override
    public Rotation2d getRotations() {
        return Rotation2d.fromRotations(softwarePosition);
    }

    @Override
    public Rotation2d getRotationsPerSecond() {
        return Rotation2d.fromRotations(this.softwareVelocity);
    }
    
    public void setEncoderPosition(Rotation2d rotations) {
        this.softwarePosition = rotations.getRotations();
    }

    public String getCANbus() {
        return "rio";
    }

}
