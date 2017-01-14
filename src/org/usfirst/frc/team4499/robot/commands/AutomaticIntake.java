package org.usfirst.frc.team4499.robot.commands;

import org.usfirst.frc.team4499.robot.OI;
import org.usfirst.frc.team4499.robot.RobotMap;

import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutomaticIntake extends Command {
	private double time;
	private double nspeed;
	private double mspeed;

    public AutomaticIntake() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	time = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
     	String stall = "";
    	
        if(StallProtection.checkstall()){
        	System.out.println(StallProtection.whichstall);
        	stall = StallProtection.whichstall;
        }
    	RobotMap.robotDrive.mecanumDrive_Cartesian(0, -0.2, 0 ,0);
    
    	if(StallProtection.checkstall()){
    	    	System.out.println(StallProtection.whichstall);
    	    	stall = StallProtection.whichstall;
    	    }
    	    	
    	    //	RobotMap.robotDrive.mecanumDrive_Cartesian(OI.controllerOne.getRawAxis(4), OI.controllerOne.getRawAxis(5),0,0);
    	    	
    	    if(StallProtection.delay(0.1)){
    	    	
    	    		
    	    		
    	    		RobotMap.rightintake.set(nspeed);
    	    		RobotMap.leftintake.set(-nspeed);
    	    		//System.out.println(RobotMap.leftintake.getOutputCurrent());
    	    		//System.out.println(RobotMap.rightintake.getOutputCurrent());}
    	    		
    	    }
    	    	
    	    	
    	    	
    	                
    	  else if(stall.equals("both")){
    	    	
    			RobotMap.leftintake.set(-nspeed);
    			RobotMap.rightintake.set(nspeed);
    			 StallProtection.reset();
    			
    	    }
    	    	else if(stall.equals("right")){
    	    		RobotMap.leftintake.set(-nspeed);
    	    		RobotMap.rightintake.set(mspeed);
    	    		//System.out.println(RobotMap.leftintake.getOutputCurrent());
    	    		//System.out.println(RobotMap.rightintake.getOutputCurrent());
    	    	}
    	    	else if( stall.equals("left")){
    	    		RobotMap.leftintake.set(-mspeed);
    	    		RobotMap.rightintake.set(nspeed);
    	    		//System.out.println(RobotMap.leftintake.getOutputCurrent());
    	    		//System.out.println(RobotMap.rightintake.getOutputCurrent());
    	    	}
    	    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("Timer: " + Timer.getFPGATimestamp() + " " + time);
    	if(Timer.getFPGATimestamp()>= time+1){
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	RobotMap.rightintake.set(nspeed);
		RobotMap.leftintake.set(-nspeed);
    	RobotMap.leftintake.set(0);
		RobotMap.rightintake.set(0);
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
