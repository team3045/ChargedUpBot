// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance extends CommandBase {
  private final Timer timer;
  private final DriveTrain drivetrain;
  private double previousPitch;
  private double prevTime;
  
  public Balance(DriveTrain drivetrain) {
    timer = new Timer();
    timer.start();
    this.drivetrain = drivetrain;
    // Our code is written for pitch that increases as the robot tilts backwards
    previousPitch = -drivetrain.pigeon.getPitch();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = timer.get();
    double dt = time -prevTime;
    // Our code is written for pitch that increases as the robot tilts backwards
    double pitch = -drivetrain.pigeon.getPitch();
    
    System.out.println((Math.abs(pitch-previousPitch) / dt));
    // Will only act if the robot is not balanced (deadspot is currently set to 2 degrees)
    if (Math.abs(pitch) > Constants.kPitchDeadspot && (Math.abs(pitch-previousPitch) / dt < Constants.kRotationPerSecondMax)) {
      if (Math.abs(pitch - previousPitch) > 3) {
        // Stops movement if the charge station rapidly drops
        drivetrain.stop();
      } else {
        // Math.copySign results in backwards movement to counteract a negative pitch and vice versa
        // 0.075 value should be changed for best effect with your team's robot
        // Our differential drive is stored under the drivetrain class as robotDrive
        // kSpeedMult is used for slow-speed testing and is generally set to 1
        drivetrain.tankDrive(Math.copySign(Constants.kSpeedMult * 0.1, pitch), Math.copySign(Constants.kSpeedMult * 0.1, pitch));
        // In our experience, the parameters of arcadeDrive() are swapped from what is in the docs.
        // The non-0 value is meant to be in the parameter dictating forward/backward robot movement
      }
    } else if ((Math.abs(pitch-previousPitch) / dt > Constants.kRotationPerSecondMax)) {
      drivetrain.tankDrive(Math.copySign(Constants.kSpeedMult * 0.05, -pitch), Math.copySign(Constants.kSpeedMult * 0.05, -pitch));
    }
    previousPitch = pitch;
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // No lasting movement after command ends (just in case)
    drivetrain.stop();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}