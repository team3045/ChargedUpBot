// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance extends CommandBase {
  private final DriveTrain drivetrain;
  private double previousPitch;
  
  public Balance(DriveTrain drivetrain) {
    this.drivetrain = drivetrain;
    // Our code is written for pitch that increases as the robot tilts backwards
    previousPitch = -drivetrain.pigeon.getPitch();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Our code is written for pitch that increases as the robot tilts backwards
    double pitch = -drivetrain.pigeon.getPitch();
    
    // Will only act if the robot is not balanced (deadspot is currently set to 2 degrees)
    if (Math.abs(pitch) > Constants.kPitchDeadspot) {
      if (Math.abs(pitch - previousPitch) > 3) {
        // Stops movement if the charge station rapidly drops
        drivetrain.stop();
      } else {
        // Math.copySign results in backwards movement to counteract a negative pitch and vice versa
        // 0.075 value should be changed for best effect with your team's robot
        // Our differential drive is stored under the drivetrain class as robotDrive
        // kSpeedMult is used for slow-speed testing and is generally set to 1
        drivetrain.tankDrive(0, Math.copySign(Constants.kSpeedMult * 0.075, pitch));
        // In our experience, the parameters of arcadeDrive() are swapped from what is in the docs.
        // The non-0 value is meant to be in the parameter dictating forward/backward robot movement
      }
    }
    previousPitch = pitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // No lasting movement after command ends (just in case)
    drivetrain.stop();
  }
}