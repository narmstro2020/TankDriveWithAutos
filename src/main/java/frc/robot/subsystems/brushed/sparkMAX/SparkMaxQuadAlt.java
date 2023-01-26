// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.brushed.sparkMAX;

import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.brushed.BrushedMotorType;

/** Add your docs here. */
public class SparkMaxQuadAlt extends SparkMaxQuadEnc {

    public SparkMaxQuadAlt(BrushedMotorType brushedMotorType, int deviceID, boolean isInverted, double kP, double kI,
            double kD, int countsPerRev) {
        super(brushedMotorType, deviceID, isInverted, kP, kI, kD, countsPerRev);
    }

    @Override
    public Rotation2d getRotations() {
        double rotations = this.sparkMax
                .getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, this.countsPerRev)
                .getPosition();
        return Rotation2d.fromRotations(rotations);
    }

    @Override
    public Rotation2d getRotationsPerSecond() {
        double rotationsPerMinute = this.sparkMax
                .getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, this.countsPerRev).getVelocity();
        double rotationsPerSecond = rotationsPerMinute / 60.0;
        return Rotation2d.fromRotations(rotationsPerSecond);
    }

    @Override
    public void setEncoderPosition(Rotation2d rotations) {
        this.sparkMax.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, this.countsPerRev)
                .setPosition(rotations.getRotations());
    }
}
