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

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private PWMVictorSPX leftMotor1;
private PWMTalonSRX leftMotor2;
private PWMVictorSPX leftMotor3;
private MotorControllerGroup leftMotors;
private PWMVictorSPX rightMotor1;
private PWMVictorSPX rightMotor2;
private PWMVictorSPX rightMotor3;
private MotorControllerGroup rightMotors;
private DifferentialDrive differentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
    *
    */
    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftMotor1 = new PWMVictorSPX(3);
 addChild("LeftMotor1",leftMotor1);
 leftMotor1.setInverted(false);

leftMotor2 = new PWMTalonSRX(4);
 addChild("LeftMotor2",leftMotor2);
 leftMotor2.setInverted(false);

leftMotor3 = new PWMVictorSPX(5);
 addChild("LeftMotor3",leftMotor3);
 leftMotor3.setInverted(false);

leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2 , leftMotor3 );
 addChild("LeftMotors",leftMotors);
 

rightMotor1 = new PWMVictorSPX(0);
 addChild("RightMotor1",rightMotor1);
 rightMotor1.setInverted(false);

rightMotor2 = new PWMVictorSPX(1);
 addChild("RightMotor2",rightMotor2);
 rightMotor2.setInverted(false);

rightMotor3 = new PWMVictorSPX(2);
 addChild("RightMotor3",rightMotor3);
 rightMotor3.setInverted(false);

rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2 , rightMotor3 );
 addChild("RightMotors",rightMotors);
 

differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
 addChild("Differential Drive",differentialDrive);
 differentialDrive.setSafetyEnabled(true);
differentialDrive.setExpiration(0.1);
differentialDrive.setMaxOutput(1.0);



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
