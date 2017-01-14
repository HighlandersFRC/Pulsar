
package org.usfirst.frc.team4499.robot;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team4499.robot.subsystems.ExampleSubsystem;
import org.usfirst.frc.team4499.robot.tools.IntakeCorrection;

import org.usfirst.frc.team4499.robot.commands.ExampleCommand;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import org.usfirst.frc.team4499.robot.commands.*;
import org.usfirst.frc.team4499.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;
	
	public static Lifter lifter;
	
	OneDimensionalVelocityMP testMP;

	//LifterGoToTicks lifterSet = new LifterGoToTicks(4000);
    Command autonomousCommand;
    SendableChooser chooser;

    AutomaticIntake intakeAuto;
    

    ConfigureTalons configTalons = new ConfigureTalons();
    
int endingTicks;
double startTime;
double previousTime;
double currentVelocity;
double previousVelocity;
boolean firstTime = true;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	lifter = new Lifter();
		oi = new OI();
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", new ExampleCommand());

        intakeAuto = new AutomaticIntake();

        
        endingTicks = Math.abs(RobotMap.lifterMotorMaster.getEncPosition());
    	RobotStats.endTicks = endingTicks;
    	
    	testMP = new OneDimensionalVelocityMP(-50, RobotMap.lifterMotorMaster, lifter, 100, 1);
        
        //Lifter = new Lifter();
        

        //Configure Talons
        
        //RobotMap.motorRightOne.setInverted(true);
    	//RobotMap.motorRightTwo.setInverted(true);
        
    	RobotMap.lifterMotorMaster.enableBrakeMode(true);
    	RobotMap.lifterMotorMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	RobotMap.lifterMotorMaster.reverseSensor(true);
    	RobotMap.lifterMotorMaster.setEncPosition(0);
    	RobotMap.lifterMotorSlave.changeControlMode(TalonControlMode.Follower);
    	RobotMap.lifterMotorMaster.changeControlMode(TalonControlMode.PercentVbus);
    	RobotMap.lifterMotorMaster.setPID(0.4, 0.0006, 30, 0, 1000, 0, 0);
    	//RobotMap.lifterMotorMaster.setAllowableClosedLoopErr(0);
    	RobotMap.lifterMotorMaster.setVoltageRampRate(6);
    	RobotMap.lifterMotorMaster.setAllowableClosedLoopErr(0);    	
    	RobotMap.lifterMotorMaster.setForwardSoftLimit(7850);
    	RobotMap.lifterMotorMaster.enableForwardSoftLimit(true);
        
        //System.out.println("Configuring Talons Test");
        //configTalons.start();
        //System.out.println("Configured Talons");
        
        
//        chooser.addObject("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", chooser);
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		RobotStats.endTicks = Math.abs(RobotMap.lifterMotorMaster.getEncPosition());
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
        autonomousCommand = (Command) chooser.getSelected();
        
		/* String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} */
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
        Scheduler.getInstance().run();
    }

    
    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
    	firstTime = true;
    	//lifterSet.start();
<<<<<<< HEAD
//<<<<<<< HEAD
=======

>>>>>>> origin/Dev
    	
    	testMP.start();
    	
    	//RobotMap.lifterMotorMaster.set(0);
    	
    	RobotMap.lifterMotorMaster.setVoltageRampRate(100);
    	
    	//RobotMap.lifterMotorMaster.setVoltageRampRate(16);
<<<<<<< HEAD
//=======
    
//>>>>>>> origin/Dev
    	
    	
    	
=======

    	startTime = Timer.getFPGATimestamp();
    	previousVelocity = RobotMap.lifterMotorMaster.getEncVelocity();
>>>>>>> origin/Dev
    	
    	
        if (autonomousCommand != null) autonomousCommand.cancel();
    }
   

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	//System.out.println("firstTime: " + firstTime);

    	  //OI.autointake.whenPressed(new Automaticintake());
    	RobotMap.robotDrive.mecanumDrive_Cartesian(OI.controllerOne.getRawAxis(4), OI.controllerOne.getRawAxis(4),0,0);
    	
    	if(OI.autointake.get()){
    		intakeAuto.start();
    	}
    	IntakeCorrection.intakecrate();
    	//System.out.println(RobotMap.lifterMotorMaster.getEncPosition());
    	//System.out.println(RobotMap.lifterMotorMaster.getOutputVoltage());
    	//RobotMap.lifterMotorMaster.set(-OI.controllerOne.getRawAxis(5)); //Negative because the controller has up -> negative return
    	//RobotMap.lifterMotorSlave.set(RobotMap.lifterMotorMaster.getDeviceID());        
        //SmartDashboard.putNumber("Pressure", RobotMap.lifterMotorMaster.getEncPosition());    

    	
    	//RobotMap.robotDrive.mecanumDrive_Cartesian(OI.controllerOne.getRawAxis(4), OI.controllerOne.getRawAxis(5),0,0);
       
    	
    	//System.out.println(-OI.controllerOne.getRawAxis(5));
    	
    	if (Math.abs(-OI.controllerOne.getRawAxis(1)) > .1) {
    		lifter.controlLifter();
    	} else {
    		lifter.stop();
    	}
    	
    	//System.out.println("Time: " + Timer.getFPGATimestamp());
    	
    	// currentAcceleration = (currentVelocity - previousVelocity) / (currentTime - previousTime);
    	//System.out.println("Current acceleration: " + ((RobotMap.lifterMotorMaster.getEncVelocity() - previousVelocity) / (Timer.getFPGATimestamp() - previousTime)) );
    	
    	//if ((Math.abs(-OI.controllerOne.getRawAxis(1)) > 0.1) && (firstTime == true)) { 
    		//System.out.println("Started timer");
    		//startTime = Timer.getFPGATimestamp();
    		//firstTime = false;
    	//}
    	
    	
    	//System.out.println(RobotMap.lifterMotorMaster.getEncVelocity());
    	
    	//if (Math.abs(RobotMap.lifterMotorMaster.getEncVelocity()) > 400) {
    		//System.out.println("Ended timer");
    		//System.out.println("Timer: " + Timer.getFPGATimestamp());
    		//System.out.println("Current acceleration: " + ((RobotMap.lifterMotorMaster.getEncVelocity() - previousVelocity) / (Timer.getFPGATimestamp() - previousTime)) );
    		//System.out.println(startTime - Timer.getFPGATimestamp());
    	//}
    	previousVelocity = RobotMap.lifterMotorMaster.getEncVelocity();
    	previousTime = Timer.getFPGATimestamp();
    	
    	//System.out.println(RobotMap.lifterMotorMaster.getOutputVoltage());
    	//RobotMap.lifterMotorMaster.set(-OI.controllerOne.getRawAxis(5)); //Negative because the controller has up -> negative return
    	RobotMap.lifterMotorSlave.set(RobotMap.lifterMotorMaster.getDeviceID());
        
        //SmartDashboard.putNumber("Pressure", RobotMap.lifterMotorMaster.getEncPosition());
    	
    	
    	
    	
    	
    	

      // RobotMap.robotDrive.setInvertedMotor(RobotMap.robotDrive.1, 1);
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
