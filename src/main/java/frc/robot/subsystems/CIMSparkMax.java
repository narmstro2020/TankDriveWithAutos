// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Motor;

public class CIMSparkMax extends SubsystemBase implements Motor{
  /** Creates a new CIM. */

  private final CANSparkMax sparkMax;

  public enum CIMEncoderType{
    QuadratureMain,
    QuadratureAlternate,
    None
  }

    
  public CIMSparkMax(
    int deviceID,
    double kP, 
    double kI, 
    double kD) {

      this.sparkMax = new CANSparkMax(deviceID, MotorType.kBrushed);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public Rotation2d getRotations() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Rotation2d getRotationsPerSecond() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void setRotationsPerSecond(Rotation2d rotationsPerSecond) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void setEncoderPosition(Rotation2d rotations) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public int getDeviceID() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public String getCANbus() {
    // TODO Auto-generated method stub
    return null;
  }
}
