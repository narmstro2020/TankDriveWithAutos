// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Motor;

public class CIMSparkMax extends SubsystemBase implements Motor {
  /** Creates a new CIMSparkMax. */

  private final CANSparkMax sparkMax;
  private final Optional<RelativeEncoder> encoder;
  private final Rotation2d maxRotationsPerSecond = Rotation2d.fromRadians(DCMotor.getCIM(1).freeSpeedRadPerSec);
  private final double maxVoltage = 12.0;
  private Optional<Double> softwareEncoderPosition;
  private Optional<Double> softwareEncoderVelocity;
  private double lastTime = Timer.getFPGATimestamp();

  public enum SparkMaxEncoderUsed {
    QuadratureMain,
    QuadratureAlternate,
  }

  
  public CIMSparkMax(
      int deviceID,
      boolean isInverted) {
    this.sparkMax = new CANSparkMax(deviceID, MotorType.kBrushed);
    this.sparkMax.setInverted(isInverted);
    this.encoder = Optional.empty();
    this.softwareEncoderPosition = Optional.of(0.0);
    this.softwareEncoderVelocity = Optional.of(0.0);

  }

  public CIMSparkMax(
      int deviceID,
      boolean isInverted,
      SparkMaxAlternateEncoder.Type sparkMaxAlternateEncoderType,
      int countsPerRev,
      double kP,
      double kI,
      double kD) {

    this.sparkMax = new CANSparkMax(deviceID, MotorType.kBrushed);
    this.sparkMax.setInverted(isInverted);
    this.sparkMax.getPIDController().setP(kP);
    this.sparkMax.getPIDController().setI(kI);
    this.sparkMax.getPIDController().setD(kD);
    this.softwareEncoderPosition = Optional.empty();
    this.softwareEncoderVelocity = Optional.empty();

    this.encoder = Optional
        .of(this.sparkMax.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, countsPerRev));

  }

  public CIMSparkMax(
      int deviceID,
      boolean isInverted,
      int countsPerRev,
      double kP,
      double kI,
      double kD) {

    this.sparkMax = new CANSparkMax(deviceID, MotorType.kBrushed);
    this.sparkMax.setInverted(isInverted);
    this.sparkMax.getPIDController().setP(kP);
    this.sparkMax.getPIDController().setI(kI);
    this.sparkMax.getPIDController().setD(kD);
    this.softwareEncoderPosition = Optional.empty();
    this.softwareEncoderVelocity = Optional.empty();

    this.encoder = Optional
        .of(this.sparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, countsPerRev));

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("MotorRotations", getRotations().getRotations());
    SmartDashboard.putNumber("MotorRotationsPerSecond", getRotationsPerSecond().getRotations());

    if (this.softwareEncoderPosition.isPresent()) {
      double currentTime = Timer.getFPGATimestamp();
      double deltaT = currentTime - lastTime;
      double currentRotationsPerSecond = this.softwareEncoderVelocity.get();
      this.softwareEncoderPosition = Optional
          .of(this.softwareEncoderPosition.get() + currentRotationsPerSecond * deltaT);
      lastTime = currentTime;
    }

  }

  @Override
  public Rotation2d getRotations() {
    if (this.encoder.isPresent()) {
      return Rotation2d.fromRotations(this.encoder.get().getPosition());
    } else {
      return Rotation2d.fromRotations(this.softwareEncoderPosition.get());
    }
  }

  @Override
  public Rotation2d getRotationsPerSecond() {
    if (this.encoder.isPresent()) {
      return Rotation2d.fromRotations(this.encoder.get().getVelocity() / 60.0);
    } else {
      return Rotation2d.fromRotations(this.softwareEncoderVelocity.get());
    }
  }

  @Override
  public void setRotationsPerSecond(Rotation2d rotationsPerSecond) {
    if (this.encoder.isPresent()) {
      this.sparkMax.getPIDController().setReference(rotationsPerSecond.getRotations() * 60.0, ControlType.kVelocity);
    } else {
      double rotationsPerSecondValue = rotationsPerSecond.getRotations();
      double maxRotationsPerSecondValue = this.maxRotationsPerSecond.getRotations();
      if (rotationsPerSecondValue > maxRotationsPerSecondValue) {
        rotationsPerSecondValue = maxRotationsPerSecondValue;
      } else if (rotationsPerSecondValue < maxRotationsPerSecondValue) {
        rotationsPerSecondValue = -maxRotationsPerSecondValue;
      }

      double voltage = maxVoltage * (rotationsPerSecondValue / maxRotationsPerSecondValue);
      this.softwareEncoderVelocity = Optional.of(rotationsPerSecondValue);
      sparkMax.setVoltage(voltage);

    }

  }

  @Override
  public void setEncoderPosition(Rotation2d rotations) {
    if (this.encoder.isPresent()) {
      this.encoder.get().setPosition(rotations.getRotations());
    } else {
      this.softwareEncoderPosition = Optional.of(rotations.getRotations());
    }
  }

  @Override
  public int getDeviceID() {
    return this.sparkMax.getDeviceId();
  }

  @Override
  public String getCANbus() {
    return "rio";
  }
}
