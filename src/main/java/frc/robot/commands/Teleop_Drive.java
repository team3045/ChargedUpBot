// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.RobotContainer;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.DriveTrain;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Teleop_Drive extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private final DriveTrain m_driveTrain;
    private final RobotContainer m_RobotContainer;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public Teleop_Drive(DriveTrain subsystem, RobotContainer m_RobotContainer) {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

        this.m_RobotContainer = m_RobotContainer;
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    private void Drive() {
        m_driveTrain.leftMotor1.set(ControlMode.PercentOutput, m_RobotContainer.drive_controller.getLeftY());
        m_driveTrain.leftMotor2.set(ControlMode.PercentOutput, m_RobotContainer.drive_controller.getLeftY());
        m_driveTrain.leftMotor3.set(ControlMode.PercentOutput, m_RobotContainer.drive_controller.getLeftY());

        m_driveTrain.rightMotor1.set(ControlMode.PercentOutput, m_RobotContainer.drive_controller.getRightY());
        m_driveTrain.rightMotor2.set(ControlMode.PercentOutput, m_RobotContainer.drive_controller.getRightY());
        m_driveTrain.rightMotor3.set(ControlMode.PercentOutput, m_RobotContainer.drive_controller.getRightY());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
