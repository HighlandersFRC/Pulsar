package org.usfirst.frc.team4499.robot.commands;

import org.usfirst.frc.team4499.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;

public class StallProtection {
	public static String whichstall;
	public static double time;
	
	public static boolean checkstall(){
		
		if(RobotMap.leftintake.getOutputCurrent()>=5.5 &&RobotMap.rightintake.getOutputCurrent()>=5.5){
			whichstall = "both";
			time = Timer.getFPGATimestamp();
			return true;
			
		}
		else if(RobotMap.leftintake.getOutputCurrent()>=5.5){
			whichstall = "left";
			time = Timer.getFPGATimestamp();
			return true;
		}
		else if(RobotMap.rightintake.getOutputCurrent()>=5.5 ) {
			whichstall = "right";
			time = Timer.getFPGATimestamp();
			return true;
		
	}
		else{
			return false;
		}
		
}
	public static boolean delay(double t){
		if(Timer.getFPGATimestamp() <= time + t){
			return false;
		}
		return true;
		
		
		
	}
	public static void reset(){
		if(delay(0.075)){
			  RobotMap.leftintake.set(-0.5);
  		    RobotMap.rightintake.set(0.5);
		}
		else{
			  RobotMap.leftintake.set(0.5);
  		    RobotMap.rightintake.set(-0.5);
		}
	}

}
