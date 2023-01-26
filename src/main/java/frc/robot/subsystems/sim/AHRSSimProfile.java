// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sim;

import java.util.function.Supplier;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class AHRSSimProfile extends SimProfile {

    private final Supplier<Double> yawRadiansPerSecondSupplier;
    private final Supplier<Double> rollRadiansPerSecondSupplier;
    private final Supplier<Double> pitchRadiansPerSecondSupplier;

    private final SimDouble simulatedYawDegrees;
    private final SimDouble simulatedPitchDegrees;
    private final SimDouble simulatedRollDegrees;


    public AHRSSimProfile(
            Supplier<ChassisSpeeds> chassisSpeedsSupplier) {

        this.yawRadiansPerSecondSupplier = () -> chassisSpeedsSupplier.get().omegaRadiansPerSecond;
        this.rollRadiansPerSecondSupplier = () -> 0.0;
        this.pitchRadiansPerSecondSupplier = () -> 0.0;

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        this.simulatedYawDegrees = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        this.simulatedYawDegrees.set(0.0);
        this.simulatedPitchDegrees = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Pitch"));
        this.simulatedPitchDegrees.set(0.0);
        this.simulatedRollDegrees = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Roll"));
        this.simulatedRollDegrees.set(0.0);
    }

    @Override
    public void run() {
        double deltaT = getPeriod() / 1000.0;

        double currentSimulatedYaw = this.simulatedYawDegrees.get();
        this.simulatedYawDegrees
                .set(currentSimulatedYaw - Math.toDegrees(yawRadiansPerSecondSupplier.get()) * deltaT);
        double currentSimulatedPitch = this.simulatedPitchDegrees.get();
        this.simulatedPitchDegrees
                .set(currentSimulatedPitch
                        - Math.toDegrees(pitchRadiansPerSecondSupplier.get()) * deltaT);
        double currentSimulatedRoll = this.simulatedRollDegrees.get();
        this.simulatedRollDegrees
                .set(currentSimulatedRoll - Math.toDegrees(rollRadiansPerSecondSupplier.get()) * deltaT);

    }

}
