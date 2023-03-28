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


import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.FourBar;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class FourBar_MidPos extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    public final FourBar m_fourBar;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public FourBar_MidPos(FourBar subsystem) {


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_fourBar = subsystem;
        addRequirements(m_fourBar);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
         // Checks which button is pressed
        
            // Sets target position
            //m_fourBar.fourBarM1.set(ControlMode.Position, Constants.midPos + Constants.armZero); 
            //m_fourBar.fourBarfollower.follow(m_fourBar.fourBarM1);


            System.out.println("Going to mid");
            m_fourBar.fourBarM1.set(ControlMode.Velocity, -Constants.targetFBVel);
        }
        
    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_fourBar.fourBarM1.set(ControlMode.Position, m_fourBar.fourBarM1.getSelectedSensorPosition());
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
