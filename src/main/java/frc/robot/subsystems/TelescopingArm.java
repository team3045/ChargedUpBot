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
import frc.robot.commands.*;
import frc.robot.math.Vector2;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class TelescopingArm extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private PWMVictorSPX pivotMotor1;
    private PWMVictorSPX pivotMotor2;
    private PWMVictorSPX telescopingWristMotor;
    private PWMVictorSPX telescopingMotor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
    *
    */
    public TelescopingArm() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        pivotMotor1 = new PWMVictorSPX(8);
        addChild("PivotMotor1",pivotMotor1);
        pivotMotor1.setInverted(false);

        pivotMotor2 = new PWMVictorSPX(9);
        addChild("PivotMotor2",pivotMotor2);
        pivotMotor2.setInverted(false);

        telescopingWristMotor = new PWMVictorSPX(10);
        addChild("TelescopingWristMotor",telescopingWristMotor);
        telescopingWristMotor.setInverted(false);

        telescopingMotor = new PWMVictorSPX(11);
        addChild("TelescopingMotor",telescopingMotor);
        telescopingMotor.setInverted(false);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
        // PivotMotor moves to set position to move Telescoping Arm
    public void goToPos(Vector2 pos) {
        pivotMotor1.set(ControlMode.Position, Constants.armZero + pos.angle() * (1024 / Math.PI));
        pivotMotor2.set(ControlMode.Follower, 9);

        telescopingMotor.set(ControlMode.Position, Constants.armZero + pos.magnitude() * Constants.armLenghtToUnits);
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
