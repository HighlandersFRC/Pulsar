package org.usfirst.frc.team4499.robot.commands;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4499.robot.RobotMap;

/**
 *
 */
public class ConfigureTalons extends Command {

    public ConfigureTalons() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	/*
    	RobotMap.motorRightOne.setInverted(true);
    	RobotMap.motorRightTwo.setInverted(true);
        
    	RobotMap.lifterMotorMaster.enableBrakeMode(true);
    	RobotMap.lifterMotorMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	RobotMap.lifterMotorMaster.reverseSensor(true);
    	RobotMap.lifterMotorMaster.setEncPosition(0);
    	RobotMap.lifterMotorSlave.changeControlMode(TalonControlMode.Follower);
    	RobotMap.lifterMotorMaster.changeControlMode(TalonControlMode.PercentVbus);
    	RobotMap.lifterMotorMaster.setPID(0.4, 0.0003, 30, 0, 1000, 0, 0);
    	RobotMap.lifterMotorMaster.setAllowableClosedLoopErr(100);
    	RobotMap.lifterMotorMaster.setVoltageRampRate(6);
    	
    	RobotMap.lifterMotorMaster.setForwardSoftLimit(7850);
    	RobotMap.lifterMotorMaster.enableForwardSoftLimit(true);
    	
    	System.out.println("Configured Talons");
    	
    	*/
    	
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Ended Talon Config");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
