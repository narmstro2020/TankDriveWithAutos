// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class AutoControl {

    public static Pair<Command, List<Pose2d>> getAutoCommand(AutoConfiguration autoConfiguration, TankDrive tankDrive) {

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
                autoConfiguration.pathName,
                autoConfiguration.firstPathConstraint,
                autoConfiguration.remainingPathConstraints);

        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
                tankDrive::getFieldCentricPosition2D,
                tankDrive::resetRobotPosition,
                autoConfiguration.ramseteController,
                tankDrive.getDifferentialDriveKinematics(),
                (leftSpeed, rightSpeed) -> tankDrive.setRobotPosition(leftSpeed, rightSpeed),
                autoConfiguration.eventMap,
                true,
                tankDrive);

        List<Pose2d> poses = new ArrayList<Pose2d>();

        for (var path : pathGroup) {
            PathPlannerTrajectory adjustedPath = PathPlannerTrajectory.transformTrajectoryForAlliance(path,
                    DriverStation.getAlliance());
            for (var state : adjustedPath.getStates()) {
                poses.add(state.poseMeters);
            }
        }

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        return new Pair<Command, List<Pose2d>>(fullAuto, poses);
    }

}