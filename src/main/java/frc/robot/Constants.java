// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot;

//import com.fasterxml.jackson.databind.ser.std.StaticListSerializerBase;

//import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.math.Vector2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
   //Claw Constants
   public static double clawSpeed = 0;

   //Fourbar target position
   public static double highPos=0;
   public static double midPos=0;
   public static double lowPos=0;

   //IDs for buttons
   public static int highPosButtonID=XboxController.Button.kY.value;
   public static int midPosButtonID=XboxController.Button.kB.value;
   public static int lowPosButtonID=XboxController.Button.kA.value;

   //Telescoping arm target positions
   public static Vector2 lowPosTArm = new Vector2(0, 0);
   public static Vector2 midPosTArm = new Vector2(0, 0);
   public static Vector2 highPosTArm = new Vector2(0, 0);
   public static double armLenghtToUnits=0;
   public static double armLengthZero = 0;

   //Zero position to calibrate position of both fourbar or telescoping arm
   public static double armZero=0;
}

