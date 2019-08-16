package org.usfirst.frc4499.robot.commands;

import org.usfirst.frc4499.robot.Robot;
import org.usfirst.frc4499.robot.subsystems.Slider;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SliderSetPositionPID extends Command {
	
	private double targetPositionInches;

    public SliderSetPositionPID(double targetPositionInches) {
    	this.targetPositionInches = targetPositionInches;
        requires(Robot.slider);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.slider.setPositionPID(targetPositionInches);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(Robot.slider.getPositionInches() - this.targetPositionInches) < Slider.PID_ERROR_INCHES;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
