package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import java.util.function.DoubleSupplier;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.DriveTrain;

import java.util.Timer;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Balance extends CommandBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private final DriveTrain m_driveTrain;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public Balance(DriveTrain subsystem) {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pigeon2 pigeon = new Pigeon2(0);
        boolean balanced = false;


        /*public motors(time) {
                double start = System.currentTimeMillis();
                while (((System.currentTimeMillis() - start)/1000.0) < (time + 1)) {
                }
        }*/
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        while (m_driveTrain.gyro.getPitch() > -3.5 && m_driveTrain.gyro.getPitch() < 3.5) {
            if (m_driveTrain.gyro.getPitch() < -3.5 && m_driveTrain.gyro.getPitch() > -10) {
                // tell the motors to run for 1 second backwards
            } else if (m_driveTrain.gyro.getPitch() < -10 && m_driveTrain.gyro.getPitch() > -20) {
                // run the motors for 2 seconds backwards
            } else if (m_driveTrain.gyro.getPitch() < -20 && m_driveTrain.gyro.getPitch() >-30) {
                // run the motors for 3 seconds backwards
            }

        }
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
    /*
     * In DriveTrain Subsystem, declare a public Pigeon2 called gyro
     * Create Constant in Constants.java called speed per degree
     * Set the motors to the hyperbolic tangent of the angle times the constant using the Motor.Set() method
     */
    }