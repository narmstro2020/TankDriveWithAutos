// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Motor;

/** Add your docs here. */
public class LinearVelocityMechanism extends SubsystemBase{
    private final Motor motor;
    private final double motorRotationsToMetersConstant;

    public LinearVelocityMechanism(
            Motor motor,
            double motorRotationsToMetersConstant) {
        this.motor = motor;
        this.motorRotationsToMetersConstant = motorRotationsToMetersConstant;
    }

    public double getVelocityMetersPerSecond(){
        Rotation2d rotationsPerSecond = motor.getRotationsPerSecond();
        return rotationsPerSecond.getRotations() * motorRotationsToMetersConstant;
    }

    public double getPositionMeters(){
        Rotation2d rotations = motor.getRotations();
        return rotations.getRotations() * motorRotationsToMetersConstant;
    }

    public void setVelocityMetersPerSecond(double velocityMetersPerSecond){
        Rotation2d motorRotationsPerSecond = Rotation2d.fromRotations(velocityMetersPerSecond / motorRotationsToMetersConstant);
        this.motor.setRotationsPerSecond(motorRotationsPerSecond);
    }

    public void stopMechanism(){
        motor.stop();
    }

    @Override
    public void periodic() {
        
    }

}
