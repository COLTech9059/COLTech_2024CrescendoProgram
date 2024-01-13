// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

 //Drive train motor IDs
 public static int leftPID = 2;
 public static int rightPID = 3;
 public static int leftFID = 4;
 public static int rightFID = 5;

 //Manipulator motor IDs
 public static int leftBaseID = 6;
 public static int rightBaseID = 7;
 public static int ampID = 8;
 public static int intakeID = 9;
 
 //Digital inputs DIO ports
 public static int beamSensorID = 0;
 public static int magneticSensorID = 1;
}
