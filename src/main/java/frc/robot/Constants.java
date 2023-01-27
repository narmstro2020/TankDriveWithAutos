// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FieldConstants {
    public static double fieldLengthMeters = 16.5;
    public static double fieldWidthMeters = 8.0;
    public static double fieldCenterX = fieldLengthMeters / 2;
    public static double fieldCenterY = fieldWidthMeters / 2;
  }

  public static class DifferentialDriveConstants {
    public static double trackWidthMeters = 1;
    public static double maxTranslationalSpeed = 2;
    public static double maxRotationalSpeed = 2;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Autos {

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    private static final HashMap<String, Command> globalHashMap = new HashMap<>() {
      {

      }
    };

    public static final PathConstraints globalFirstPathConstraints = new PathConstraints(4, 3);
    public static final PathConstraints globalRemainingPathConstraints = new PathConstraints(4, 3);
    public static final RamseteController globalRamsetteController = new RamseteController(2, 0.7);

    public static final AutoConfiguration forward1 = new AutoConfiguration(
        "Forward1",
        globalHashMap,
        globalRamsetteController,
        globalFirstPathConstraints,
        globalRemainingPathConstraints);
  }
}
