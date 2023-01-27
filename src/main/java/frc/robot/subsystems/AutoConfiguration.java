// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class AutoConfiguration {

    public final String pathName;
    public final HashMap<String, Command> eventMap;
    public final RamseteController ramseteController;
    public final PathConstraints firstPathConstraint;
    public final PathConstraints[] remainingPathConstraints;

    public AutoConfiguration(
            String pathName,
            HashMap<String, Command> eventMap,
            RamseteController ramseteController,
            PathConstraints firstPathConstraint,
            PathConstraints... remainingPathConstraints) {

        this.pathName = pathName;
        this.eventMap = eventMap;
        this.ramseteController = ramseteController;
        this.firstPathConstraint = firstPathConstraint;
        this.remainingPathConstraints = remainingPathConstraints;

    }

}