// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class TankDrive extends SubsystemBase {

    private final LinearVelocityMechanism leftWheels;
    private final LinearVelocityMechanism rightWheels;
    private final Gyroscope gyroscope;
    private final Pose2d initialPosition;

    private final DifferentialDriveKinematics differentialDriveKinematics;
    private final DifferentialDrivePoseEstimator differentialDrivePoseEstimator;
    private final Field2d theField = new Field2d();

    private ChassisSpeeds robotCentricChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private DifferentialDriveWheelSpeeds[] differentialDriveWheelSpeeds;

    public TankDrive(
            Pose2d initialPosition,
            LinearVelocityMechanism leftWheels,
            LinearVelocityMechanism rightWheels,
            double trackWidthMeters) {
                
        this.differentialDriveWheelSpeeds = new DifferentialDriveWheelSpeeds[2];
        this.initialPosition = initialPosition;
        this.leftWheels = leftWheels;
        this.rightWheels = rightWheels;

        Supplier<ChassisSpeeds> robotChassisSpeedsSupplier = () -> robotCentricChassisSpeeds;

        this.gyroscope = Gyroscope.getNavxMXP2(
                initialPosition.getRotation(),
                robotChassisSpeedsSupplier);

        this.differentialDriveKinematics = new DifferentialDriveKinematics(trackWidthMeters);

        this.differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(
                differentialDriveKinematics,
                this.gyroscope.getYaw(),
                this.leftWheels.getPositionMeters(),
                this.rightWheels.getPositionMeters(),
                initialPosition);

        this.theField.setRobotPose(initialPosition);
        SmartDashboard.putData("The Field", theField);

    }

    public DifferentialDriveKinematics getDifferentialDriveKinematics() {
        return this.differentialDriveKinematics;
    }

    public Pose2d getFieldCentricPosition2D() {
        return this.differentialDrivePoseEstimator.getEstimatedPosition();
    }

    public void setRobotPosition(ManualControl manualControl) {
    }

    public void setRobotPosition(ChassisSpeeds chassisSpeeds) {
        this.robotCentricChassisSpeeds = chassisSpeeds;

    }

    public void updateRobotPosition() {

        this.differentialDrivePoseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                this.gyroscope.getYaw(),
                this.leftWheels.getPositionMeters(),
                this.rightWheels.getPositionMeters());

    }

    public void resetRobotPosition(Pose2d robotPosition) {
        robotCentricChassisSpeeds = new ChassisSpeeds(0, 0, 0);

        this.differentialDrivePoseEstimator.resetPosition(
                this.gyroscope.getYaw(),
                this.leftWheels.getPositionMeters(),
                this.rightWheels.getPositionMeters(),
                robotPosition);
    }

    public void stopDrive() {
        leftWheels.stopMechanism();
        rightWheels.stopMechanism();
    }

    public void setTrajectory(List<Pose2d> poses) {
        this.theField.getObject("trajectory").setPoses(poses);
    }

    @Override
    public void periodic() {

    }

}
