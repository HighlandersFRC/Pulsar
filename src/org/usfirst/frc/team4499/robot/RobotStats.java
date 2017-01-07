package org.usfirst.frc.team4499.robot;

public class RobotStats {
	
	public static double robotWeight = 45; // robot weight in lb's
	public static double driveDiameter = 6; // robot drive wheel diameters in inches
	public static double driveWheelFriction = 0; // coeffecient of friction on drive wheels
	public static double floorFriction = 0;
	public static double referenceVoltage = 12;
	
	public static int endTicks = 0; //Used to hold position in the Lifter subsystem
	public static double maxLifterAccelerationLoad = 450; //Ticks/second squared, under 6 totes. Originally 513, scaled down for motor capability.
	public static double maxLifterAccelerationUnload = 789; //Ticks/second squared, under no load
}
