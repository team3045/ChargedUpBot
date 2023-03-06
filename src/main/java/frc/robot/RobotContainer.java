// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveTrain;



  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  public static RobotContainer m_robotContainer = new RobotContainer();

  // Config pigeon
  // Last two parameters should be changed to reflect offsets in pitch and roll, respectively
  
  
  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems
    public final PhotoVision m_photoVision = new PhotoVision();
    //public final TelescopingArm m_telescopingArm = new TelescopingArm();
    public final FourBar m_fourBar = new FourBar();
    public final DriveTrain m_driveTrain = new DriveTrain();
    public final Claw m_claw = new Claw();
    public final Joystick LJoystick = new Joystick(1);
    public final Joystick RJoystick = new Joystick(0);
    public final Joystick buttonboard = new Joystick(2);
    


// Joysticks
    private final XboxController controller = new XboxController(0);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems


    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous_Command", new Autonomous_Command());
    SmartDashboard.putData("Teleop_Drive", new Teleop_Drive( m_driveTrain ));
    SmartDashboard.putData("GoToPosition_FourBar", new FourBar_LowPos( m_fourBar ));
    //SmartDashboard.putData("GoToPosition_TelescopingArm", new GoToPosition_TelescopingArm( m_telescopingArm ));
    SmartDashboard.putData("Intake", new Intake( m_claw ));
    SmartDashboard.putData("Outtake", new Outtake( m_claw ));
    SmartDashboard.putData("ToggleGrab", new ToggleGrab( m_claw ));
    SmartDashboard.putData("ConeCubeSwap", new ConeCubeSwap());

    switch (m_driveTrain.getDriveMode()) {
      case TANK:
        m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.tankDrive(-LJoystick.getRawAxis(1), RJoystick.getRawAxis(1)), m_driveTrain));
        break;
      //case CHEEZY:
        //m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.cheezyDrive(getLeftY(), getRightX()), m_driveTrain));
        //break;
      //case ARCADE:
        //m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.arcadeDrive(getLeftY(), getRightX()), m_driveTrain));
        //break;
      default:
        // tank 
        m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.tankDrive(-LJoystick.getRawAxis(0), -RJoystick.getRawAxis(1)), m_driveTrain));
        

        m_driveTrain.pigeon.configMountPose(0, 0, 0);
        m_driveTrain.pigeon.setYaw(0);
        break;
    }
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    // Create some buttons

    Balance balance = new Balance(m_driveTrain);

    final JoystickButton balanceBtn = new JoystickButton(buttonboard, 3);
    balanceBtn.whileTrue(balance);

    FourBar_LowPos LowPos = new FourBar_LowPos(m_fourBar);

    final Trigger lowPosBtn = new JoystickButton(buttonboard, 4);
    lowPosBtn.onTrue(LowPos);

    FourBar_MidPos MidPos = new FourBar_MidPos(m_fourBar);

    final Trigger midPosBtn = new JoystickButton(buttonboard, 5);
    midPosBtn.onTrue(MidPos);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public XboxController getController() {
      return controller;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

}
