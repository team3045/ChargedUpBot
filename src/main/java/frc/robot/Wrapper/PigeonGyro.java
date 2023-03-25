package frc.robot.Wrapper;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PigeonGyro implements Sendable{
    public final Pigeon2 pigeon;
    public PigeonGyro(Pigeon2 pigeon){
        this.pigeon = pigeon;
    }
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", pigeon::getYaw, null);
    }
}
