package org.usfirst.frc.team4499.robot.commands;

import org.usfirst.frc.team4499.robot.OI;
import org.usfirst.frc.team4499.robot.Robot;
import org.usfirst.frc.team4499.robot.RobotMap;
import org.usfirst.frc.team4499.robot.RobotStats;
import org.usfirst.frc.team4499.robot.commands.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ControlLifter extends Command {
	
	ZeroLifterVelocity stopLifter = new ZeroLifterVelocity();
	
    public ControlLifter() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.lifter);
    	
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.lifterMotorMaster.set(-OI.controllerOne.getRawAxis(1));
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//RobotStats.endTicks = Math.abs(RobotMap.lifterMotorMaster.getEncPosition());
    	//System.out.println("Running ControlLifter");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(-OI.controllerOne.getRawAxis(1)) < .1;
    	//return Math.abs(RobotMap.lifterMotorMaster.getEncVelocity()) < 50;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//endingTicks = Math.abs(RobotMap.lifterMotorMaster.getEncPosition());
    	//RobotStats.endTicks = endingTicks;
    	stopLifter.start();
    	
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
