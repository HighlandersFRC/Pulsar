package org.usfirst.frc.team4499.robot.tools;

import org.usfirst.frc.team4499.robot.OI;
import org.usfirst.frc.team4499.robot.RobotMap;
import org.usfirst.frc.team4499.robot.commands.StallProtection;

public class IntakeCorrection {
	private static double nspeed;
	private static double mspeed;
	public static void intakecrate(){
	String stall = "";
	
    if(StallProtection.checkstall()){
    	System.out.println(StallProtection.whichstall);
    	stall = StallProtection.whichstall;
    }
    	
    	
    if(StallProtection.delay(0.1)){
    	if(OI.intakeinbutton.get()){
    		
    		
    		RobotMap.rightintake.set(nspeed);
    		RobotMap.leftintake.set(-nspeed);
    		System.out.println(RobotMap.leftintake.getOutputCurrent());
    		System.out.println(RobotMap.rightintake.getOutputCurrent());
    		
    		
    	}
    	else if(OI.intakeoutbutton.get()){
    		
    		    RobotMap.leftintake.set(nspeed);
    		    RobotMap.rightintake.set(-nspeed);
    		    System.out.println(RobotMap.leftintake.getOutputCurrent());
        		System.out.println(RobotMap.rightintake.getOutputCurrent());
    		
    	        }
    	else{
    		RobotMap.leftintake.set(0);
    		RobotMap.rightintake.set(0);
    	}
                }
    else if(stall.equals("both")&& OI.intakeinbutton.get()){
    	
		RobotMap.leftintake.set(-nspeed);
		RobotMap.rightintake.set(nspeed);
	
		
			StallProtection.reset();
			
		
		
		 
		
    	}
    	else if(stall.equals("right")&& OI.intakeinbutton.get()){
    		RobotMap.leftintake.set(-nspeed);
    		RobotMap.rightintake.set(mspeed);
    		System.out.println(RobotMap.leftintake.getOutputCurrent());
    		System.out.println(RobotMap.rightintake.getOutputCurrent());
    	}
    	else if( stall.equals("left")&& OI.intakeinbutton.get()){
    		RobotMap.leftintake.set(-mspeed);
    		RobotMap.rightintake.set(nspeed);
    		System.out.println(RobotMap.leftintake.getOutputCurrent());
    		System.out.println(RobotMap.rightintake.getOutputCurrent());
    	}
    	else if(stall.equals("both")&& OI.intakeoutbutton.get()){
        	
    		RobotMap.leftintake.set(nspeed);
    		RobotMap.rightintake.set(-nspeed);
    		
    		
        	}
        else if(stall.equals("both")){
        	
    		RobotMap.leftintake.set(0);
    		RobotMap.rightintake.set(0);
    		
        	}
    	else if(stall.equals("right")&& OI.intakeoutbutton.get()){
    		RobotMap.leftintake.set(nspeed);
    		RobotMap.rightintake.set(-nspeed);
    		System.out.println(RobotMap.leftintake.getOutputCurrent());
    		System.out.println(RobotMap.rightintake.getOutputCurrent());
    	}
    	else if( stall.equals("left")&& OI.intakeoutbutton.get()){
    		RobotMap.leftintake.set(nspeed);
    		RobotMap.rightintake.set(-nspeed);
    		System.out.println(RobotMap.leftintake.getOutputCurrent());
    		System.out.println(RobotMap.rightintake.getOutputCurrent());
    	}
else if(stall.equals("left")){
        	
    		RobotMap.leftintake.set(0);
    		RobotMap.rightintake.set(0);
    		
        	}
else if(stall.equals("right")){
	
	RobotMap.leftintake.set(0);
	RobotMap.rightintake.set(0);
	
	}
	}

}
