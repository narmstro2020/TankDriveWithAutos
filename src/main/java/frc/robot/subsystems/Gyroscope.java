// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.sim.NavXPhysicsSim;

/** Add your docs here. */
public class Gyroscope extends SubsystemBase {

    private final Supplier<Rotation2d> yawSupplier;
    private final Supplier<Rotation2d> pitchSupplier;
    private final Supplier<Rotation2d> rollSupplier;
    private final Supplier<Double> aXSupplier;
    private final Supplier<Double> aYSupplier;
    private final Supplier<Double> aZSupplier;
    private final Supplier<Double> dXSupplier;
    private final Supplier<Double> dYSupplier;
    private final Supplier<Double> dZSupplier;

    private Gyroscope(
            Supplier<Rotation2d> yawSupplier,
            Supplier<Rotation2d> pitchSupplier,
            Supplier<Rotation2d> rollSupplier,
            Supplier<Double> aXSupplier,
            Supplier<Double> aYSupplier,
            Supplier<Double> aZSupplier,
            Supplier<Double> dXSupplier,
            Supplier<Double> dYSupplier,
            Supplier<Double> dZSupplier) {
        this.yawSupplier = yawSupplier;
        this.pitchSupplier = pitchSupplier;
        this.rollSupplier = rollSupplier;
        this.aXSupplier = aXSupplier;
        this.aYSupplier = aYSupplier;
        this.aZSupplier = aZSupplier;
        this.dXSupplier = dXSupplier;
        this.dYSupplier = dYSupplier;
        this.dZSupplier = dZSupplier;

    }

    public Rotation2d getPitch() {
        return this.pitchSupplier.get();
    }

    public Rotation2d getYaw() {
        return this.yawSupplier.get();
    }

    public Rotation2d getRoll() {
        return this.rollSupplier.get();
    }

    public double getDisplacementXMeters() {
        return dXSupplier.get();
    }

    public double getDisplacementYMeters() {
        return dYSupplier.get();
    }

    public double getDisplacementZMeters() {
        return dZSupplier.get();
    }

    public static Gyroscope getNavxMXP2(
            Rotation2d gyroOffset,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            String botName) {

        if (botName == "829") {
            AHRS mxp2Gyroscope = new AHRS(SPI.Port.kMXP);
            mxp2Gyroscope.reset();
            NavXPhysicsSim.getInstance().addAHRS(mxp2Gyroscope, chassisSpeedsSupplier, botName);
            return new Gyroscope(
                    () -> {
                        double yawDegrees = -mxp2Gyroscope.getYaw();
                        yawDegrees = yawDegrees < 0 ? yawDegrees + 360 : yawDegrees;
                        return Rotation2d.fromDegrees(yawDegrees);
                    },
                    () -> {
                        double pitchDegrees = -mxp2Gyroscope.getPitch();
                        pitchDegrees = pitchDegrees < 0 ? pitchDegrees + 360 : pitchDegrees;
                        return Rotation2d.fromDegrees(pitchDegrees);
                    },
                    () -> {
                        double rollDegrees = -mxp2Gyroscope.getRoll();
                        rollDegrees = rollDegrees < 0 ? rollDegrees + 360 : rollDegrees;
                        return Rotation2d.fromDegrees(rollDegrees);
                    },
                    () -> (double) mxp2Gyroscope.getWorldLinearAccelX() * 9.8,
                    () -> (double) mxp2Gyroscope.getWorldLinearAccelY() * 9.8,
                    () -> (double) mxp2Gyroscope.getWorldLinearAccelZ() * 9.8,
                    () -> (double) mxp2Gyroscope.getDisplacementX(),
                    () -> (double) mxp2Gyroscope.getDisplacementY(),
                    () -> (double) mxp2Gyroscope.getDisplacementZ());
        } else {
            WPI_Pigeon2 pigeon = new WPI_Pigeon2(23);
            pigeon.reset();

            NavXPhysicsSim.getInstance().addPigeon(pigeon, chassisSpeedsSupplier, botName);
            return new Gyroscope(
                    () -> {
                        double yawDegrees = -pigeon.getYaw();
                        yawDegrees = yawDegrees < 0 ? yawDegrees + 360 : yawDegrees;
                        return Rotation2d.fromDegrees(yawDegrees);
                    },
                    () -> {
                        double pitchDegrees = -pigeon.getPitch();
                        pitchDegrees = pitchDegrees < 0 ? pitchDegrees + 360 : pitchDegrees;
                        return Rotation2d.fromDegrees(pitchDegrees);
                    },
                    () -> {
                        double rollDegrees = -pigeon.getRoll();
                        rollDegrees = rollDegrees < 0 ? rollDegrees + 360 : rollDegrees;
                        return Rotation2d.fromDegrees(rollDegrees);
                    },
                    () -> 0.0,
                    () -> 0.0,
                    () -> 0.0,
                    () -> 0.0,
                    () -> 0.0,
                    () -> 0.0);
        }

    }

    @Override
    public void periodic() {
        DecimalFormat df = new DecimalFormat("###.###");

        SmartDashboard.putString("GyroYaw",
                df.format(this.yawSupplier.get().getDegrees()));

        SmartDashboard.putString("GyroPitch",
                df.format(this.pitchSupplier.get().getDegrees()));

        SmartDashboard.putString("GyroRoll",
                df.format(this.rollSupplier.get().getDegrees()));

        SmartDashboard.putString("GyroRobotAccX",
                df.format(this.aXSupplier.get()));

        SmartDashboard.putString("GyroRobotAccY",
                df.format(this.aYSupplier.get()));

        SmartDashboard.putString("GyroRobotAccZ",
                df.format(this.aZSupplier.get()));

        SmartDashboard.putString("GyroRobotDispX",
                df.format(this.getDisplacementXMeters()));

        SmartDashboard.putString("GyroRobotDispY",
                df.format(getDisplacementYMeters()));

        SmartDashboard.putString("GyroRobotDispZ",
                df.format(getDisplacementZMeters()));
    }

}
