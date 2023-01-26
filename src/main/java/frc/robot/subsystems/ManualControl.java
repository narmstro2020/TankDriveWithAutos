package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualControl extends SubsystemBase {
    private final Supplier<Double> robotCentricForwardSpeedSupplier;
    private final Supplier<Double> rotationSpeedSupplier;

    private ManualControl(
            Supplier<Double> robotCentricForwardSpeedSupplier,
            Supplier<Double> rotationSpeedSupplier) {
        this.robotCentricForwardSpeedSupplier = robotCentricForwardSpeedSupplier;
        this.rotationSpeedSupplier = rotationSpeedSupplier;
    }


    public double getRobotCentricForwardSpeed() {
        return this.robotCentricForwardSpeedSupplier.get();
    }

    public double getRobotCentricSpeed() {
        return Math.sqrt(
                this.robotCentricForwardSpeedSupplier.get() * this.robotCentricForwardSpeedSupplier.get());
    }

    public double getRotationSpeed() {
        return this.rotationSpeedSupplier.get();
    }

    public static ManualControl getManualControl(
            CommandXboxController commandXboxController,
            double controllerDeadband,
            double controllerAxisMax) {

        Supplier<Double> robotCentricForwardSpeedSupplier = () -> {
            double robotCentricForwardSpeed = -MathUtil.applyDeadband(commandXboxController.getLeftY(),
                    controllerDeadband, controllerAxisMax);
            double robotCentricSpeed = Math.sqrt(robotCentricForwardSpeed * robotCentricForwardSpeed);
            return robotCentricSpeed > 1 ? robotCentricForwardSpeed / robotCentricSpeed : robotCentricForwardSpeed;

        };

        Supplier<Double> rotationSupplier = () -> {
            double rotationSpeed = -MathUtil.applyDeadband(commandXboxController.getRightX(),
            controllerDeadband, controllerAxisMax);
            return rotationSpeed > 1 ? rotationSpeed / rotationSpeed : rotationSpeed;
        };

        return new ManualControl(
                robotCentricForwardSpeedSupplier,
                rotationSupplier);

    }

    @Override
    public void periodic() {

        DecimalFormat df = new DecimalFormat("###.###");

        SmartDashboard.putString("RobotCentricSpeedControl", df.format(getRobotCentricSpeed()));
        SmartDashboard.putString("RobotCentricForwardSpeedControl", df.format(getRobotCentricForwardSpeed()));
        SmartDashboard.putString("RotationalSpeedControl", df.format(getRotationSpeed()));
    }

}
