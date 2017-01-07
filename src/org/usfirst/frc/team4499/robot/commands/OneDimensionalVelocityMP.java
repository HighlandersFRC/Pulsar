package org.usfirst.frc.team4499.robot.commands;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team4499.robot.RobotMap;
import org.usfirst.frc.team4499.robot.RobotStats;

/**
 *
 */
public class OneDimensionalVelocityMP extends Command {

	CANTalon.TrajectoryPoint currentTrajPoint = new CANTalon.TrajectoryPoint();
	CANTalon MPTalon;
	
	//double [][]pointsArray;
	double previousVelocity;
	double initialVelocity;
	double goalVelocity;
	double startTime;
	double previousTime;
	double trajPointDurationMS;
	double numTrajPoints;
	boolean biggerThanMPB;
	CANTalon.MotionProfileStatus talonStatus;
	
    public OneDimensionalVelocityMP(double finalVelocity, CANTalon sensorTalon, Subsystem talonSubsystem, double numberOfTrajectoryPoints, double trajectoryPointDurationMS) {
    	requires(talonSubsystem);
    	
    	MPTalon = sensorTalon;
    	initialVelocity = sensorTalon.getEncVelocity(); // Store the initial velocity to construct the trajectory points
    	previousVelocity = sensorTalon.getEncVelocity();
    	goalVelocity = finalVelocity;
    	trajPointDurationMS = trajectoryPointDurationMS;
    	numTrajPoints = numberOfTrajectoryPoints;
    //	MPTalon.getMotionProfileStatus(talonStatus);
    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	MPTalon.changeControlMode(TalonControlMode.Follower);
    	System.out.println("Initializing MP");
    	
    	startTime = Timer.getFPGATimestamp();
    	
    	// Calculate the number of points required for the MP
    	
    	//System.out.println("goalVelocity " + goalVelocity);
    	//System.out.println("initialVelocity " + initialVelocity);
    	//System.out.println("trajPointDuration " + trajPointDurationMS);
    	
    	
    	
    	
    	//System.out.println("Calculated... " + (goalVelocity - initialVelocity) / (trajPointDurationMS / 1000 ));
    	//numTrajPoints = Math.abs((int) (Math.round( (goalVelocity - initialVelocity) / (trajPointDurationMS / 1000 ) )));
    	
    	System.out.println("numTrajPoints: " + numTrajPoints);
    	
    	// Talon SRX can store 128 trajectory points
    	if (numTrajPoints > 127) biggerThanMPB = true;
    	
    	//if (talonStatus.hasUnderrun) {
    		//System.out.println("Talon has underrun.");
    		//MPTalon.clearMotionProfileHasUnderrun();
    	//}
    	
    	
    	
    	
    	
    	
    	// Generate trajectory points and feed them to the Talon SRX's MPB
    	
    //	if (!biggerThanMPB) {
    	for (int i = 0; i < numTrajPoints; i++) {
    		
    		// Calculate the current trajectory point's velocity, and store it to calculate the next point's velocity
    		System.out.println(trajPointDurationMS);
    		System.out.println(RobotStats.maxLifterAccelerationLoad);
    		System.out.println(trajPointDurationMS / 1000);
    		System.out.println((trajPointDurationMS / 1000) * RobotStats.maxLifterAccelerationLoad);
    		System.out.println("previousVelocity " + previousVelocity);
    		currentTrajPoint.velocity = previousVelocity + ((trajPointDurationMS / 1000) * RobotStats.maxLifterAccelerationLoad); //fix for acceleration
    		previousVelocity = previousVelocity + ((trajPointDurationMS / 1000) * RobotStats.maxLifterAccelerationLoad);
    		//previousVelocity = currentTrajPoint.velocity;
    		
    		currentTrajPoint.timeDurMs = (int) trajPointDurationMS;
    		currentTrajPoint.profileSlotSelect = 0;
    		currentTrajPoint.velocityOnly = true;
    		
    		// If this is the first point, set the zeroPos value to true
    		currentTrajPoint.zeroPos = false;
    		if (i == 0) 
    			currentTrajPoint.zeroPos = true;
    			currentTrajPoint.velocity = initialVelocity;
    		
    		// If this is the last point, set the isLastPoint value to true
    		currentTrajPoint.isLastPoint = false;
    		if ((i+1) == numTrajPoints) 
    			currentTrajPoint.isLastPoint = true;
    		
    		// Push the trajectory point to the Talon SRX's MPB
    		MPTalon.pushMotionProfileTrajectory(currentTrajPoint);
    		
    		System.out.println("Trajectory point number " + i + " with a velocity of " + currentTrajPoint.velocity);
    		
    		}
    	
/*    	
  		} else {
        	for (int i = 0; i < 128; i++) {
        		
        	}
        }
*/
    	
    	
    	MPTalon.set(1);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	System.out.println("Executing MP");
    	
    	if (biggerThanMPB) {
    		
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(goalVelocity - MPTalon.getEncVelocity()) < 5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Finished motion profile");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
