// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.brushed.sparkMAX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.brushed.BrushedMotorType;
import frc.robot.subsystems.brushed.NoEncoder;

/** Add your docs here. */
public class SparkMaxNoEnc extends NoEncoder {

    protected final CANSparkMax sparkMax;
    protected final int rps;

    public SparkMaxNoEnc(
            int deviceID,
            boolean isInverted,
            BrushedMotorType brushedMotorType) {
        this.sparkMax = new CANSparkMax(deviceID, MotorType.kBrushed);
        this.sparkMax.setInverted(isInverted);
        this.rps = brushedMotorType.getRPS();
    }

    @Override
    public void setRotationsPerSecond(Rotation2d rotationsPerSecond) {
        double rotationsPerSecondValue = rotationsPerSecond.getRotations();
        if (rotationsPerSecondValue > rps) {
            rotationsPerSecondValue = rps;
        } else if (rotationsPerSecondValue < -rps) {
            rotationsPerSecondValue = -rps;
        }

        this.softwareVelocity = rotationsPerSecondValue;
        double voltagePercent = rotationsPerSecondValue / rps;
        double voltage = voltagePercent * this.maxVoltage;

        if (RobotBase.isReal()) {
            sparkMax.setVoltage(voltage);
        }
    }

    @Override
    public int getDeviceID() {
        return this.sparkMax.getDeviceId();
    }

    @Override
    public void stop() {
        this.sparkMax.setVoltage(0);
        this.sparkMax.disable();

    }

}
