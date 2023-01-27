// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class TankDrive extends SubsystemBase {

    private final LinearVelocityMechanism leftWheels;
    private final LinearVelocityMechanism rightWheels;
    private final Gyroscope gyroscope;

    private final DifferentialDriveKinematics differentialDriveKinematics;
    private final DifferentialDrivePoseEstimator differentialDrivePoseEstimator;

    private ChassisSpeeds robotCentricChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private DifferentialDriveWheelSpeeds differentialDriveWheelSpeeds;
    private final Field2d theField;

    private final String botName;

    public TankDrive(
            Pose2d initialPosition,
            LinearVelocityMechanism leftWheels,
            LinearVelocityMechanism rightWheels,
            double trackWidthMeters, Field2d theField, String botName) {
        this.botName = botName;

        this.theField = theField;
        this.differentialDriveWheelSpeeds = new DifferentialDriveWheelSpeeds();
        this.leftWheels = leftWheels;
        this.rightWheels = rightWheels;

        Supplier<ChassisSpeeds> robotChassisSpeedsSupplier = () -> robotCentricChassisSpeeds;

        this.gyroscope = Gyroscope.getNavxMXP2(
                initialPosition.getRotation(),
                robotChassisSpeedsSupplier, 
                botName);

        this.differentialDriveKinematics = new DifferentialDriveKinematics(trackWidthMeters);

        this.differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(
                differentialDriveKinematics,
                this.gyroscope.getYaw(),
                this.leftWheels.getPositionMeters(),
                this.rightWheels.getPositionMeters(),
                initialPosition);

    }

    public DifferentialDriveKinematics getDifferentialDriveKinematics() {
        return this.differentialDriveKinematics;
    }

    public Pose2d getFieldCentricPosition2D() {
        return this.differentialDrivePoseEstimator.getEstimatedPosition();
    }

    public void setRobotPosition(ManualControl manualControl) {
        this.robotCentricChassisSpeeds = new ChassisSpeeds(
                manualControl.getRobotCentricForwardSpeed(),
                0,
                manualControl.getRotationSpeed());

        setRobotPosition(this.robotCentricChassisSpeeds);
        if (this.robotCentricChassisSpeeds.vxMetersPerSecond == 0
                && this.robotCentricChassisSpeeds.vyMetersPerSecond == 0
                && this.robotCentricChassisSpeeds.omegaRadiansPerSecond == 0) {
            stopDrive();
        }
    }

    public void setRobotPosition(ChassisSpeeds chassisSpeeds) {
        this.robotCentricChassisSpeeds = chassisSpeeds;

        this.differentialDriveWheelSpeeds = differentialDriveKinematics.toWheelSpeeds(chassisSpeeds);

        setRobotPosition(this.differentialDriveWheelSpeeds.leftMetersPerSecond,
                this.differentialDriveWheelSpeeds.rightMetersPerSecond);

    }

    public void setRobotPosition(double leftMetersPerSecond, double rightMetersPerSecond) {
        this.differentialDriveWheelSpeeds = new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
        this.robotCentricChassisSpeeds = this.differentialDriveKinematics
                .toChassisSpeeds(this.differentialDriveWheelSpeeds);

        this.leftWheels.setVelocityMetersPerSecond(leftMetersPerSecond);
        this.rightWheels.setVelocityMetersPerSecond(rightMetersPerSecond);
        updateRobotPosition();
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

    @Override
    public void periodic() {
        if (botName == "829") {
            this.theField.setRobotPose(this.differentialDrivePoseEstimator.getEstimatedPosition());
        }
        else{
            this.theField.getObject(botName).setPose(this.differentialDrivePoseEstimator.getEstimatedPosition());
        }
    }

}
