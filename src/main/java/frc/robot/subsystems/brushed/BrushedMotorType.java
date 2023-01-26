package frc.robot.subsystems.brushed;

import edu.wpi.first.math.system.plant.DCMotor;

public enum BrushedMotorType {
    CIM((int)(DCMotor.getCIM(1).freeSpeedRadPerSec / 2 / Math.PI));

    private int RPS;

    private BrushedMotorType(int RPS){
        this.RPS = RPS;
    }

    public int getRPS() {
        return RPS;
    }
    
}
