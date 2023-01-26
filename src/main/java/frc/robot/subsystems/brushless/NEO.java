// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.brushless;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Motor;

public class NEO extends SubsystemBase implements Motor {
  private final CANSparkMax sparkMax;

  /** Creates a new NEO. */
  // You will have to tune kP, kI, kD so that the setVelocity function actually
  // sets the
  // motor to the right rotationsPerSecond
  // the periodic method has SmartDashboard entries
  // already setup to accomplish this.
  public NEO(
      int deviceID,
      boolean isInverted,
      double kP,
      double kI,
      double kD) {

    this.sparkMax = new CANSparkMax(deviceID, MotorType.kBrushless);
    this.sparkMax.setInverted(isInverted);
    this.sparkMax.getPIDController().setP(kP);
    this.sparkMax.getPIDController().setI(kI);
    this.sparkMax.getPIDController().setD(kD);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("MotorRotations", getRotations().getRotations());
    SmartDashboard.putNumber("MotorRotationsPerSecond", getRotationsPerSecond().getRotations());
  }

  @Override
  public Rotation2d getRotations() {
    double rotations = this.sparkMax.getEncoder().getPosition();
    return Rotation2d.fromRotations(rotations);
  }

  @Override
  public Rotation2d getRotationsPerSecond() {
    double rotationsPerMinute = this.sparkMax.getEncoder().getVelocity();
    double rotationsPerSecond = rotationsPerMinute / 60.0;
    return Rotation2d.fromRotations(rotationsPerSecond);
  }

  @Override
  public void setRotationsPerSecond(Rotation2d rotationsPerSecond) {
    double rotationsPerSecondValue = rotationsPerSecond.getRotations();
    double rotationsPerMinute = rotationsPerSecondValue * 60.0;
    this.sparkMax.getPIDController().setReference(rotationsPerMinute, ControlType.kVelocity);
  }

  @Override
  public void setEncoderPosition(Rotation2d rotations) {
    this.sparkMax.getEncoder().setPosition(rotations.getRotations());
  }

  @Override
  public int getDeviceID() {
    return this.sparkMax.getDeviceId();
  }

  @Override
  public String getCANbus() {
    return "rio";
  }

  @Override
  public void stop() {
    this.sparkMax.setVoltage(0);
    this.sparkMax.disable();    
  }
}
