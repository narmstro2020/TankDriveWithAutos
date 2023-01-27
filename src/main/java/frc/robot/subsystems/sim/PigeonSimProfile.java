// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sim;

import java.util.function.Supplier;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** Add your docs here. */
public class PigeonSimProfile extends SimProfile {

    private final Supplier<Double> yawRadiansPerSecondSupplier;

    private final SimDouble simulatedYawDegrees;

    public PigeonSimProfile(
            Supplier<ChassisSpeeds> chassisSpeedsSupplier, String botName) {

        this.yawRadiansPerSecondSupplier = () -> chassisSpeedsSupplier.get().omegaRadiansPerSecond;
        SimDeviceSim simPigeon = new SimDeviceSim("CANGyro:Pigeon 2[23]");

        this.simulatedYawDegrees = simPigeon.getDouble("rawYawInput");
        this.simulatedYawDegrees.set(0.0);

    }

    @Override
    public void run() {
            double deltaT = getPeriod() / 1000.0;

            double currentSimulatedYaw = this.simulatedYawDegrees.get();
            this.simulatedYawDegrees
                            .set(currentSimulatedYaw - Math.toDegrees(yawRadiansPerSecondSupplier.get()) * deltaT);
            
    }

}
