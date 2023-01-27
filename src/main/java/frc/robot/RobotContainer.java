// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AutoControl;
import frc.robot.subsystems.LinearVelocityMechanism;
import frc.robot.subsystems.ManualControl;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.brushed.BrushedMotorType;
import frc.robot.subsystems.brushed.sparkMAX.SparkMaxNoEnc;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final TankDrive tankDrive;

  private final ManualControl manualControl = ManualControl.getManualControl(m_driverController, 0.1, 1);

  private final SendableChooser<Pair<Command, List<Pose2d>>> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Field2d theField, String botName, int leftCan, int rightCan, Pose2d initialPosition) {

    LinearVelocityMechanism leftWheels = new LinearVelocityMechanism(
        new SparkMaxNoEnc(leftCan, false, BrushedMotorType.CIM),
        1.0 / 6.0);

    LinearVelocityMechanism rightWheels = new LinearVelocityMechanism(
        new SparkMaxNoEnc(rightCan, true, BrushedMotorType.CIM),
        1.0 / 6.0);



    if (botName == "829") {
      theField.setRobotPose(initialPosition);
    } else {
      theField.getObject(botName).setPose(initialPosition);
    }

    this.tankDrive = new TankDrive(
        initialPosition,
        leftWheels,
        rightWheels,
        Constants.DifferentialDriveConstants.trackWidthMeters,
        theField, 
        botName);

    this.tankDrive.setDefaultCommand(
        Commands.run(
            () -> {
              this.tankDrive.setRobotPosition(manualControl);
            },
            this.tankDrive));

    Pair<Command, List<Pose2d>> autoForward1 = AutoControl.getAutoCommand(Constants.Autos.forward1, tankDrive);
    Pair<Command, List<Pose2d>> autoForward2 = AutoControl.getAutoCommand(Constants.Autos.forward2, tankDrive);

    m_chooser.setDefaultOption("Forward1", autoForward1);
    m_chooser.addOption("Forward2", autoForward2);
    SmartDashboard.putData(botName + " Auto choices", m_chooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Field2d theField, String botName) {
    Pair<Command, List<Pose2d>> autoChoice = this.m_chooser.getSelected();
    setTrajectory(autoChoice.getSecond(), theField, botName);
    return autoChoice.getFirst();
  }

  public void setTrajectory(List<Pose2d> poses, Field2d theField, String botName) {
    theField.getObject(botName + " trajectory").setPoses(poses);
  }
}