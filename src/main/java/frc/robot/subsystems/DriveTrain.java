// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.PhotonPoseEstimator;
import com.ctre.phoenix.motorcontrol.ControlMode;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

public WPI_TalonFX leftMotor1;
public WPI_TalonFX leftMotor2;
public WPI_TalonFX leftMotor3;
public WPI_TalonFX rightMotor1;
public WPI_TalonFX rightMotor2;
public WPI_TalonFX rightMotor3;
// private DifferentialDrive differentialDrive;
public Pigeon2 pigeon;

public PhotonPoseEstimator photonPoseEstimator;
public DifferentialDriveOdometry poseEstimator;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  public DifferentialDrive m_drive;

  private DriveMode driveMode;
  
  public enum DriveMode {
      TANK, CHEEZY, ARCADE;
    }
  
    /**
    *
    */
    public DriveTrain() {
      // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
      leftMotor1 = new WPI_TalonFX(Constants.lMotor1, "Canivore 3045");
      leftMotor2 = new WPI_TalonFX(Constants.lMotor2, "Canivore 3045");
      leftMotor3 = new WPI_TalonFX(Constants.lMotor3, "Canivore 3045");
      rightMotor1 = new WPI_TalonFX(Constants.rMotor1, "Canivore 3045");
      rightMotor2 = new WPI_TalonFX(Constants.rMotor2, "Canivore 3045");
      rightMotor3 = new WPI_TalonFX(Constants.rMotor3, "Canivore 3045");

      leftMotor2.follow(leftMotor1);
      leftMotor2.setInverted(InvertType.FollowMaster);
      leftMotor3.follow(leftMotor1);
      leftMotor3.setInverted(InvertType.FollowMaster);
      rightMotor2.follow(rightMotor1);
      rightMotor2.setInverted(InvertType.FollowMaster);
      rightMotor3.follow(rightMotor1);
      rightMotor3.setInverted(InvertType.FollowMaster);

      pigeon = new Pigeon2(Constants.pigeonCanId, "Canivore 3045");

      //m_drive = new DifferentialDrive(leftMotor1, rightMotor1);
      //m_drive.setDeadband(0.05);

      leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
      rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
      leftMotor1.setNeutralMode(NeutralMode.Brake);
      leftMotor2.setNeutralMode(NeutralMode.Brake);
      leftMotor3.setNeutralMode(NeutralMode.Brake);

      rightMotor1.setNeutralMode(NeutralMode.Brake);
      rightMotor2.setNeutralMode(NeutralMode.Brake);
      rightMotor3.setNeutralMode(NeutralMode.Brake);

      /*differentialDrive = new DifferentialDrive(leftMotor1, rightMotor1);
      addChild("Differential Drive", differentialDrive);
      differentialDrive.setSafetyEnabled(true);
      differentialDrive.setExpiration(0.1);
      differentialDrive.setMaxOutput(1.0);*/

      driveMode = DriveMode.TANK;

      // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

      poseEstimator = new DifferentialDriveOdometry(new Rotation2d(pigeon.getYaw()), getLeftWheelPosition(),
          getRightWheelPosition());

      photonPoseEstimator = null;
      /*try{
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(Filesystem.getDeployDirectory().toPath().resolve(Constants.layoutPATH).toString()), PoseStrategy.LOWEST_AMBIGUITY, RobotContainer.m_robotContainer.m_photoVision.camera, new Transform3d());
      } catch (Exception e) {
        DriverStation.reportError("Unable to open AprilTagFieldLayout: " + Filesystem.getDeployDirectory().toPath().resolve(Constants.layoutPATH).toString(), e.getStackTrace());
      }*/
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(leftMotor1.getSelectedSensorVelocity() * 10 / 2048 * 10.5 * 0.1523,
          rightMotor1.getSelectedSensorVelocity() * 10 / 2048 * 10.5 * 0.1523);
    }

    public double getLeftWheelSpeed() {
      return leftMotor1.getSelectedSensorVelocity() * 10 / 2048 * 10.5 * 0.1523;
    }
    
    public double getRightWheelSpeed() {
      return rightMotor1.getSelectedSensorVelocity() * 10 / 2048 * 10.5 * 0.1523;
    }

    public double getLeftWheelPosition() {
      return leftMotor1.getSelectedSensorPosition() * 10 / 2048 * 10.5 * 0.1523;
    }
    
    public double getRightWheelPosition() {
      return rightMotor1.getSelectedSensorPosition() * 10 / 2048 * 10.5 * 0.1523;
    }

    public Pose2d getPose() {
      poseEstimator.update(new Rotation2d(pigeon.getYaw()), getLeftWheelPosition(), getRightWheelPosition());
      Pose2d pos = poseEstimator.getPoseMeters();
      return pos;
    }

    public void resetOdometry(Pose2d pose) {
      pigeon.setYaw(pose.getRotation().getDegrees());
      poseEstimator.resetPosition(new Rotation2d(pigeon.getYaw()), getLeftWheelPosition(),
          getRightWheelPosition(), pose);
    }

    public void tankDriveVolts(double left, double right) {
      leftMotor1.set(ControlMode.PercentOutput, left / 12);
      rightMotor1.set(ControlMode.PercentOutput, -right / 12);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("thing",pigeon.getPitch());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void tankDrive(double leftpower, double rightpower) {
        //m_drive.tankDrive(leftpower, rightpower);
        //m_drive.feed();
        leftMotor1.set(ControlMode.PercentOutput, Math.copySign(leftpower * leftpower, leftpower));
        rightMotor1.set(ControlMode.PercentOutput, -Math.copySign(rightpower * rightpower, rightpower));
      }
    
      public void cheezyDrive(double straight, double turn) {
        m_drive.curvatureDrive(straight, -turn, false);
      }
    
      public void arcadeDrive(double straight, double turn) {
        m_drive.arcadeDrive(straight, -turn);
      }
    
      public void stopDrive() {
        leftMotor1.set(ControlMode.PercentOutput, 0);
        rightMotor1.set(ControlMode.PercentOutput, 0);
      }
        
      public void resetEncoders() {
        leftMotor1.setSelectedSensorPosition(0);
        rightMotor1.setSelectedSensorPosition(0);
      }
    
      public double getLeftEncoder() {
          return (leftMotor1.getSelectedSensorPosition());
      }
    
      public double getRightEncoder() {
        return (rightMotor1.getSelectedSensorPosition());
    }
    
      public double getAverageEncoder(){
        return ((getLeftEncoder()+getRightEncoder())/2);
      }
    
      public void setMaxOutput(double maxOutput) {
          m_drive.setMaxOutput(maxOutput);
      }
      
      public double getRawEncoder() {
        return leftMotor1.getSelectedSensorPosition(); //temp method
      }
    
      public DriveMode getDriveMode(){
        return driveMode;
      }
    
      public void setDriveMode(DriveMode driveMode) {
          this.driveMode = driveMode;

          // Put methods for controlling this subsystem
          // here. Call these from Commands.
      }

      public void stop() {
        leftMotor1.set(0.0);
        rightMotor1.set(0.0);
      }
}
