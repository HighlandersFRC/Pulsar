package org.usfirst.frc4499.robot.commands;

import org.usfirst.frc4499.robot.Robot;
import org.usfirst.frc4499.robot.subsystems.Slider.SliderControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SliderSetMode extends Command {

	private SliderControlMode controlMode;
	
    public SliderSetMode(SliderControlMode controlMode) {
    	this.controlMode = controlMode;
        requires(Robot.slider);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (controlMode == SliderControlMode.JOYSTICK_PID) {
    		Robot.slider.setPositionPID(Robot.slider.getPositionInches());
    	}
    	else if (controlMode == SliderControlMode.JOYSTICK_MANUAL) {
    		Robot.slider.setSpeedJoystick(0);
        }
        else if (controlMode == SliderControlMode.MOTION_MAGIC){
            Robot.slider.setPositionMM(Robot.slider.getPositionInches());
        }

    	else {
    		Robot.slider.setSpeed(0.0);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
