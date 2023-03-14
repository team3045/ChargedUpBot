// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * A modified version of the RamseteCommand class that comes with WPILib, that allows for other commands to interrupt at given times
 */
public class AutonomousCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private final boolean m_usePID;
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final RamseteController m_follower;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
  private final PIDController m_leftController;
  private final PIDController m_rightController;
  private final BiConsumer<Double, Double> m_output;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  private double lastInterrupt = 0;
  private double routineTime = 0;
  private boolean interrupted = false;

  private int interruptStage = 0;
  private Command[] interruptCommands;
  private double[] interruptTimes;

  private Command interruptingCommand;

  /**
   * Constructs a new AutonomousCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param wheelSpeeds A function that supplies the speeds of the left and right sides of the robot
   *     drive.
   * @param leftController The PIDController for the left side of the robot drive.
   * @param rightController The PIDController for the right side of the robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param requirements The subsystems to require.
   */
  public AutonomousCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      PIDController leftController,
      PIDController rightController,
      BiConsumer<Double, Double> outputVolts,
      Subsystem requirements, 
      Command[] interruptCommands,
      double[] interruptTimes) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "AutonomousCommand");
    m_pose = requireNonNullParam(pose, "pose", "AutonomousCommand");
    m_follower = requireNonNullParam(controller, "controller", "AutonomousCommand");
    m_feedforward = feedforward;
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "AutonomousCommand");
    m_speeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "AutonomousCommand");
    m_leftController = requireNonNullParam(leftController, "leftController", "AutonomousCommand");
    m_rightController = requireNonNullParam(rightController, "rightController", "AutonomousCommand");
    m_output = requireNonNullParam(outputVolts, "outputVolts", "AutonomousCommand");

    this.interruptCommands = interruptCommands;
    this.interruptTimes = interruptTimes;

    m_usePID = true;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
      m_prevTime = -1;
      var initialState = m_trajectory.sample(0);
      m_prevSpeeds = m_kinematics.toWheelSpeeds(
              new ChassisSpeeds(
                      initialState.velocityMetersPerSecond,
                      0,
                      initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
      m_timer.restart();
      if (m_usePID) {
          m_leftController.reset();
          m_rightController.reset();
      }
  }

  public void interrupt(Command... with) {
      lastInterrupt = m_timer.get();
      interrupted = true;

      CommandScheduler.getInstance().schedule(with);
  }

  @Override
  public void execute() {
      if (interrupted) {
          interrupted = !interruptingCommand.isFinished();
          if (interrupted)
              return;

          routineTime += m_timer.get() - lastInterrupt;
      }
    double curTime = m_timer.get() - routineTime;
    double dt = curTime - m_prevTime;

    if (interruptStage < interruptTimes.length && curTime > interruptTimes[interruptStage]) {
        interrupt(interruptCommands[interruptStage]);
        interruptStage += 1;
    }

    if (m_prevTime < 0) {
      m_output.accept(0.0, 0.0);
      m_prevTime = curTime;
      return;
    }

    var targetWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (m_usePID) {
      double leftFeedforward =
          m_feedforward.calculate(
              leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

      double rightFeedforward =
          m_feedforward.calculate(
              rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

      leftOutput =
          leftFeedforward
              + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

      rightOutput =
          rightFeedforward
              + m_rightController.calculate(
                  m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    m_output.accept(leftOutput, rightOutput);
    m_prevSpeeds = targetWheelSpeeds;
    m_prevTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();

    if (interrupted) {
      m_output.accept(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds() + routineTime);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("leftVelocity", () -> m_prevSpeeds.leftMetersPerSecond, null);
    builder.addDoubleProperty("rightVelocity", () -> m_prevSpeeds.rightMetersPerSecond, null);
  }
}
