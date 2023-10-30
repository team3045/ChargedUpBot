// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Jaguar;

/** Add your docs here. */
public class WheelDrive {
    private Jaguar angleMotor;
    private Jaguar speedMotor;
    private PIDController pidController;
    private final double MAX_VOLTS = 4.95;

    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new Jaguar (angleMotor);
        this.speedMotor = new Jaguar (speedMotor);
        pidController = new PIDController (1, 0, 0);
    
        pidController.setIntegratorRange(-1, 1);
        pidController.isContinuousInputEnabled();
        //Issues with code here
    }

    public void drive (double speed, double angle) {
    speedMotor.set (speed);

    double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
    if (setpoint < 0) {
        setpoint = MAX_VOLTS + setpoint;
    }
    if (setpoint > MAX_VOLTS) {
        setpoint = setpoint - MAX_VOLTS;
    }

    pidController.setSetpoint (setpoint);
}
}
