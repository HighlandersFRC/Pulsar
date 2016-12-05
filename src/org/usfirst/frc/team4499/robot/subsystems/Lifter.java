package org.usfirst.frc.team4499.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4499.robot.RobotMap;
import edu.wpi.first.wpilibj.*;


/**
 *
 */
public class Lifter extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public void stop() {
		RobotMap.lifterMotorMaster.set(0);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    	//Hold current position
    }
}

