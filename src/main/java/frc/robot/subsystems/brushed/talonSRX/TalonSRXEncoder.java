// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.brushed.talonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.brushed.BrushedMotorType;

/** Add your docs here. */
public class TalonSRXEncoder extends TalonSRXNoEnc {

    protected TalonSRXEncoder(
            int deviceID,
            boolean isInverted,
            BrushedMotorType brushedMotorType,
            TalonSRXFeedbackDevice talonSRXFeedbackDevice) {
        super(deviceID, isInverted, brushedMotorType);

        this.talonSRX.configSelectedFeedbackSensor(
                talonSRXFeedbackDevice,
                0,
                30);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("MotorRotations", getRotations().getRotations());
        SmartDashboard.putNumber("MotorRotationsPerSecond", getRotationsPerSecond().getRotations());
    }

    @Override
    public Rotation2d getRotations() {
        return Rotation2d.fromRotations(talonSRX.getSelectedSensorPosition() / 4096.0);
    }

    @Override
    public Rotation2d getRotationsPerSecond() {
        return Rotation2d.fromRotations(talonSRX.getSelectedSensorVelocity() * 10.0 / 4096.0);
    }

    @Override
    public void setRotationsPerSecond(Rotation2d rotationsPerSecond) {
        double nativeUnitsPer100ms = rotationsPerSecond.getRotations() * 4096.0 / 10.0;
        talonSRX.set(ControlMode.Velocity, nativeUnitsPer100ms);
    }

    @Override
    public void setEncoderPosition(Rotation2d rotations) {
        double nativeUnits = rotations.getRotations() * 4096;
        talonSRX.setSelectedSensorPosition(nativeUnits);
    }

}
