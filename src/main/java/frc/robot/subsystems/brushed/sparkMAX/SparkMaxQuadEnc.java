// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.brushed.sparkMAX;

import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.brushed.BrushedMotorType;

/** Add your docs here. */
public class SparkMaxQuadEnc extends SparkMaxNoEnc {

    protected final int countsPerRev;

    public SparkMaxQuadEnc(
            BrushedMotorType brushedMotorType,
            int deviceID,
            boolean isInverted,
            double kP,
            double kI,
            double kD,
            int countsPerRev) {

        super(deviceID, isInverted, brushedMotorType);

        sparkMax.setInverted(isInverted);
        sparkMax.getPIDController().setP(kP);
        sparkMax.getPIDController().setI(kI);
        sparkMax.getPIDController().setD(kD);
        this.countsPerRev = countsPerRev;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("MotorRotations", getRotations().getRotations());
        SmartDashboard.putNumber("MotorRotationsPerSecond", getRotationsPerSecond().getRotations());
    }

    @Override
    public Rotation2d getRotations() {
        double rotations = this.sparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, this.countsPerRev)
                .getPosition();
        return Rotation2d.fromRotations(rotations);
    }

    @Override
    public Rotation2d getRotationsPerSecond() {
        double rotationsPerMinute = this.sparkMax
                .getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, this.countsPerRev).getVelocity();
        double rotationsPerSecond = rotationsPerMinute / 60.0;
        return Rotation2d.fromRotations(rotationsPerSecond);
    }

    @Override
    public void setRotationsPerSecond(Rotation2d rotationsPerSecond) {
        double rotationsPerSecondValue = rotationsPerSecond.getRotations();
        double rotationsPerMinute = rotationsPerSecondValue * 60.0;
        this.sparkMax.getPIDController().setReference(rotationsPerMinute,
                com.revrobotics.CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setEncoderPosition(Rotation2d rotations) {
        this.sparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, this.countsPerRev)
                .setPosition(rotations.getRotations());
    }

    @Override
    public int getDeviceID() {
        return this.sparkMax.getDeviceId();
    }

    @Override
    public String getCANbus() {
        return "rio";
    }



}
