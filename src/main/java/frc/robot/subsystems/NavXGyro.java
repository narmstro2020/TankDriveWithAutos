// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Gyroscope;
import frc.robot.subsystems.sim.NavXPhysicsSim;

/** Add your docs here. */
public class NavXGyro extends SubsystemBase implements Gyroscope {
    private final AHRS gyro;

    public NavXGyro(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();
        NavXPhysicsSim.getInstance().addAHRS(gyro, chassisSpeedsSupplier);

    }

    public Rotation2d getPitch() {
        double pitchDegrees = -gyro.getPitch();
        pitchDegrees = pitchDegrees < 0 ? pitchDegrees + 360 : pitchDegrees;
        return Rotation2d.fromDegrees(pitchDegrees);
    }

    public Rotation2d getYaw() {
        double yawDegrees = -gyro.getYaw();
        yawDegrees = yawDegrees < 0 ? yawDegrees + 360 : yawDegrees;
        return Rotation2d.fromDegrees(yawDegrees);
    }

    public Rotation2d getRoll() {
        double rollDegrees = -gyro.getRoll();
        rollDegrees = rollDegrees < 0 ? rollDegrees + 360 : rollDegrees;
        return Rotation2d.fromDegrees(rollDegrees);
    }

    @Override
    public void periodic() {
        DecimalFormat df = new DecimalFormat("###.###");

        SmartDashboard.putString("GyroYaw",
                df.format(getYaw().getDegrees()));

        SmartDashboard.putString("GyroPitch",
                df.format(getPitch().getDegrees()));

        SmartDashboard.putString("GyroRoll",
                df.format(getRoll().getDegrees()));
    }

}
