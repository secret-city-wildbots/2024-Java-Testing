package frc;

import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Translation2d;

public class universalVariables {
    //Store all variables that will be used in files, add variables when trying to refer to a variable

    //Insturctions on using in scripts
        //To access the Universal Variables in a new script/file for the first line(Does not have to be first just recommended) of the class put "static universalVariables uV(recommendation for how you call it) = new universalVariables();"
        //For variables without "public static final" you must type universalVariables.<variable you want to call...>
        //any other variables just add uV(Or whatever you refer to Universal Variables).<variable you want to call...>
        //Any questions ask Alan Longcoy or Casey Miller(Add to list for any new mentors)
        //PS for adding a new variable it has to start with "public" not "private"


    //SwerveModule variables
    public static final double kWheelRadius = 0.0636; // Wheel radius in Meters (2.5 inches)
      // Constants for Swerve Module Characteristics
    public static final double kModuleMaxAngularVelocity = universalVariables.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    //Drivetrain variabls
    public static final double multiplyValue = 2;

    public Pose3d position;
    public double translation3D_X = 20.0;
    public double translation3D_Y = 80.0;
    public double translation3D_Z = 0.0;
    public double rotation3D_roll = 0.0;
    public double rotation3D_pitch = 0.0;
    public double rotation3D_yaw = 0.0;
    public static final double kMaxSpeed = 1.0; // meters per second
    public static final double kMaxAngularSpeed = multiplyValue * Math.PI; // 1/2 rotation per second
    public static final double driveGearRatio = 7;
    public static final double azimuthGearRatio = 16;

    

}
//Benicio :)