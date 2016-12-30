package org.usfirst.frc.team4499.robot.commands;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4499.robot.*;
import org.usfirst.frc.team4499.robot.subsystems.*;

import org.usfirst.frc.team4499.robot.Robot;
//import org.usfirst.team4499.robot.subsystems.Electronics;

/**
 *
 */

public class LifterGoToTicks extends Command {

	int ticks;
	//Lifter lifter = new Lifter();
	
	
	
	
	
    public LifterGoToTicks(int ticks) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.lifter);
    	this.ticks = ticks;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.lifterMotorMaster.changeControlMode(TalonControlMode.Position);
    	//System.out.println("Started LifterGoToTicks, the default command");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RobotMap.lifterMotorMaster.set(RobotStats.endTicks);
    	//System.out.println("Running LifterGoToTicks");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //return ( (ticks + 50) < Math.abs(RobotMap.lifterMotorMaster.getPosition()) ) || ( (ticks - 50) < Math.abs(RobotMap.lifterMotorMaster.getPosition()) );
    	//return false;
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//RobotMap.lifterMotorMaster.changeControlMode(TalonControlMode.PercentVbus);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
    protected void interrupted() {
    	System.out.println("I JUST GOT INTERRUPTED");
    }
}
