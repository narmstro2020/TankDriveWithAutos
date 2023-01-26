// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sim;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class NavXPhysicsSim {

    private static final NavXPhysicsSim sim = new NavXPhysicsSim();
    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /**
     * Gets the robot simulator instance.
     */
    public static NavXPhysicsSim getInstance() {
        return sim;
    }

    public void addAHRS(
            AHRS ahrs,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        if (ahrs != null) {
            AHRSSimProfile simAHRS = new AHRSSimProfile(
                chassisSpeedsSupplier);
            _simProfiles.add(simAHRS);
        }
    }

    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }
}
