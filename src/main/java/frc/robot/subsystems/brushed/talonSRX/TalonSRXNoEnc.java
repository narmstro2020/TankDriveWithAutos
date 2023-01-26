// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.brushed.talonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.brushed.BrushedMotorType;
import frc.robot.subsystems.brushed.NoEncoder;

/** Add your docs here. */
public abstract class TalonSRXNoEnc extends NoEncoder{

    protected final TalonSRX talonSRX;
    protected final int rps;

    protected TalonSRXNoEnc(
        int deviceID,
        boolean isInverted,
        BrushedMotorType brushedMotorType
    ){
        this.talonSRX = new TalonSRX(deviceID);
        this.talonSRX.setInverted(isInverted);
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

        double voltagePercent = rotationsPerSecondValue / rps;

        talonSRX.set(ControlMode.PercentOutput, voltagePercent);
    }

    @Override
    public int getDeviceID() {
        return this.talonSRX.getDeviceID();
    }

    @Override
    public void stop() {
        this.talonSRX.set(ControlMode.PercentOutput, 0);        
    }
}
