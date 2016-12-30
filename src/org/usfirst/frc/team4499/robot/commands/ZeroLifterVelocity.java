package org.usfirst.frc.team4499.robot.commands;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4499.robot.*;
/**
 *
 */
public class ZeroLifterVelocity extends Command {

	int endingTicks;
	
    public ZeroLifterVelocity() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.lifter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//RobotMap.lifterMotorMaster.clearIAccum();
    	RobotMap.lifterMotorMaster.changeControlMode(TalonControlMode.Speed);
    	//RobotMap.lifterMotorMaster.
    	//RobotMap.lifterMotorMaster.setPID(p, i, d, f, izone, closeLoopRampRate, profile);
    	RobotMap.lifterMotorMaster.setPID(1, 0.0008, 200, 0, 1000, 0, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RobotMap.lifterMotorMaster.set(0);
    	System.out.println(RobotMap.lifterMotorMaster.getError());
    	//System.out.println(RobotMap.lifterMotorMaster.GetIaccum());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Math.abs(RobotMap.lifterMotorMaster.getEncVelocity()) < 5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	endingTicks = Math.abs(RobotMap.lifterMotorMaster.getEncPosition());
    	RobotStats.endTicks = endingTicks;
    	RobotMap.lifterMotorMaster.setPID(0.4, 0.0008, 30, 0, 1000, 0, 0);
    	System.out.println("Velocity should be about 0 right now.");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
