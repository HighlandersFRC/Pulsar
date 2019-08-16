package org.usfirst.frc4499.robot.subsystems;
import java.util.ArrayList;

import org.usfirst.frc4499.robot.Constants;
import org.usfirst.frc4499.robot.Robot;
import org.usfirst.frc4499.robot.RobotMap;
import org.usfirst.frc4499.utility.ControlLoopable;
import org.usfirst.frc4499.utility.Loop;
import org.usfirst.frc4499.utility.MPTalonPIDController;
import org.usfirst.frc4499.utility.PIDParams;
import org.usfirst.frc4499.utility.TalonSRXEncoder;
import org.usfirst.frc4499.utility.TalonSRXFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Slider extends Subsystem implements ControlLoopable
{
	private static Slider instance;

	public static enum SliderControlMode { MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL, MOTION_MAGIC };

	// One revolution of the 30T Drive 1.880" PD pulley = Pi * PD inches = 36/24 revs due to pulleys * 34/24 revs due to gears * 36/12 revs due mag encoder gear on ball shifter * 4096 ticks
	public static final double ENCODER_TICKS_TO_INCHES = (1);


	private double periodMs;
	private double lastControlLoopUpdatePeriod = 0.0;
	private double lastControlLoopUpdateTimestamp = 0.0;
	// Defined speeds
	public static final double CLIMB_SPEED = -1.0;
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.3;
	public static final double AUTO_ZERO_SPEED = -0.3;
	public static final double JOYSTICK_INCHES_PER_MS_HI = 30;
	public static final double JOYSTICK_INCHES_PER_MS_LO = 15;

	// Defined positions
	public static final double ZERO_POSITION_AUTON_FORWARD_INCHES = 8.0;
	public static final double ZERO_POSITION_INCHES = -0.25;
	public static final double NEAR_ZERO_POSITION_INCHES = 0.0;
	public static final double MIN_POSITION_INCHES = 10;
	public static final double MAX_POSITION_INCHES = 2000;
	public static final double AFTER_INTAKE_POSITION_INCHES = 4.0;

	public static final double SWITCH_POSITION_INCHES = 24.0;
	public static final double SWITCH_POSITION_HIGH_INCHES = 36.0; //Switch Position for First Cube APR
	public static final double SCALE_LOW_POSITION_INCHES = 56.0;
	public static final double SCALE_FIRST_CUBE_POSITION_INCHES = 78.0; //72.0
	public static final double SCALE_SECOND_CUBE_POSITION_INCHES = 77.0;
	public static final double SCALE_HIGH_POSITION_INCHES = MAX_POSITION_INCHES;
	public static final double CLIMB_BAR_POSITION_INCHES = 70.0;
	public static final double CLIMB_HIGH_POSITION_INCHES = 10.0;
	public static final double CLIMB_ASSIST_POSITION_INCHES = 50.0;

	// Motion profile max velocities and accel times
	public static final double MP_MAX_VELOCITY_INCHES_PER_SEC =  60;
	public static final double MP_T1 = 400;  // Fast = 300
	public static final double MP_T2 = 150;  // Fast = 150

	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();

	public TalonSRXEncoder sliderMotor1;

	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MM_SLOT = 1;
	public static int MP_SLOT = 2;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);
	public static final double KF_UP = 0.00;
	public static final double KF_DOWN = 0.0;
	public static final double P_Value = 1.4;
	public static final double I_Value = 0.001;
	public static final double D_Value = 1400;
	public static final double F_Value = 0.1;
	public static final int CV_value = 1200;
	public static final int A_value = 600;
	public static final double RampRate = 0.0;
	public static final double maxGravityComp = 0;
	private PIDParams sliderPIDParams = new PIDParams(P_Value, I_Value, D_Value, KF_DOWN);	// KF gets updated later
	public static final double PID_ERROR_INCHES = 50;
	LimitSwitchSource limitSwitchSource;
	// Pneumatics
	private Solenoid speedShift;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;
	private boolean isFinished;
	private SliderControlMode sliderControlMode = SliderControlMode.MOTION_MAGIC;
	public double targetPositionInchesPID = 0;
	public double targetPositionInchesMM = 0;
	private boolean firstMpPoint;
	private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
	private double p = 0;


	public Slider() {
		try {
			sliderMotor1 = TalonSRXFactory.createTalonEncoder(RobotMap.TOTE_GRABER_CAN_ID, (ENCODER_TICKS_TO_INCHES), false, FeedbackDevice.QuadEncoder);
			sliderMotor1.setInverted(false);


//	        if (sliderMotor1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            Driver.reportError("Could not detect elevator motor 1 encoder encoder!", false);
//	        }

			sliderMotor1.configNominalOutputForward(0, TalonSRXEncoder.TIMEOUT_MS);
			sliderMotor1.configNominalOutputReverse(0, TalonSRXEncoder.TIMEOUT_MS);
			sliderMotor1.configPeakOutputForward(1, TalonSRXEncoder.TIMEOUT_MS);
			sliderMotor1.configPeakOutputReverse(-1, TalonSRXEncoder.TIMEOUT_MS);

			sliderMotor1.selectProfileSlot(MM_SLOT, 0);
			sliderMotor1.config_kF(MM_SLOT, F_Value, TalonSRXEncoder.TIMEOUT_MS);
			sliderMotor1.config_kP(MM_SLOT, P_Value, TalonSRXEncoder.TIMEOUT_MS);
			sliderMotor1.config_kI(MM_SLOT, I_Value, TalonSRXEncoder.TIMEOUT_MS);
			sliderMotor1.config_kD(MM_SLOT, D_Value, TalonSRXEncoder.TIMEOUT_MS);

			sliderMotor1.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			sliderMotor1.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			sliderMotor1.setNeutralMode(NeutralMode.Brake);
			sliderMotor1.enableCurrentLimit(true);
			//sliderMotor1.setSensorPhase(true);
			motorControllers.add(sliderMotor1);
		}
		catch (Exception e) {
			System.err.println("An error occurred in the Slider constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}

	public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	}
	public void resetencoder(){
		sliderMotor1.setPosition(0);
		targetPositionInchesPID = 0;
	}


	private synchronized void setSliderControlMode(SliderControlMode controlMode) {
		this.sliderControlMode = controlMode;
	}

	private synchronized SliderControlMode getSliderControlMode() {
		return this.sliderControlMode;
	}

	public void setSpeed(double speed) {
		sliderMotor1.set(ControlMode.PercentOutput, speed);
		setSliderControlMode(SliderControlMode.MANUAL);
	}

	public void setSpeedJoystick(double speed) {
		sliderMotor1.set(ControlMode.PercentOutput, speed);
		setSliderControlMode(SliderControlMode.JOYSTICK_MANUAL);
	}
	public void setPositionMM(double targetPositionInches){
		sliderMotor1.selectProfileSlot(MM_SLOT, 0);
		sliderMotor1.set(ControlMode.MotionMagic, targetPositionInches);
		//System.err.println(sliderMotor1.getControlMode());
		
		setSliderControlMode(SliderControlMode.MOTION_MAGIC);
		updatePositionMM(targetPositionInches);
		setFinished(false);
	}

	public double calcGravityCompensationAtCurrentPosition() {
		int ticks = sliderMotor1.getSelectedSensorPosition();
		double degreesFromDown = (ticks+920)*(360.0/(4096*3));
		double compensation = maxGravityComp * Math.sin(degreesFromDown*Math.PI/180);
		//System.err.println("comp(" + degreesFromDown + "^) = " + compensation);
		return compensation;
	}
	public void updatePositionMM(double targetPositionInches){
		targetPositionInchesMM = limitPosition(targetPositionInches);
		//double startPositionInches = motor1.getPositionWorld();
		//System.err.println("compensation = " + compensation);
		sliderMotor1.set(ControlMode.MotionMagic, targetPositionInches);
		//sliderMotor1.set(ControlMode.MotionMagic, targetPositionInchesMM, DemandType.ArbitraryFeedForward, 0);
		//System.err.println(motor1.getControlMode());
		sliderMotor1.configMotionCruiseVelocity(CV_value, TalonSRXEncoder.TIMEOUT_MS);
		sliderMotor1.configMotionAcceleration(A_value, TalonSRXEncoder.TIMEOUT_MS);


	}


	public void setPositionPID(double targetPositionInches) {
		sliderMotor1.set(ControlMode.Position, targetPositionInches);
		mpController.setPIDSlot(PID_SLOT);	//TODO: verify that motor's selectProfileSlot() should be called AFTER its control mode is set
		updatePositionPID(targetPositionInches);
		setSliderControlMode(SliderControlMode.JOYSTICK_PID);
		setFinished(false);
	}

	public void updatePositionPID(double targetPositionInches) {
 		targetPositionInchesPID = limitPosition(targetPositionInches);
		double startPositionInches = sliderMotor1.getPositionWorld();
		//mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN);
		sliderMotor1.set(ControlMode.Position, targetPositionInches);
		sliderMotor1.configClosedloopRamp(.02);	//TODO: Elevator has this set to zero
		//sliderMotor1.configPeakCurrentLimit(5);
		sliderMotor1.configContinuousCurrentLimit(2);
		sliderMotor1.config_kP(0, P_Value, TalonSRXEncoder.TIMEOUT_MS);
		sliderMotor1.config_kI(0, I_Value, TalonSRXEncoder.TIMEOUT_MS);
		sliderMotor1.config_kD(0, D_Value, TalonSRXEncoder.TIMEOUT_MS);
		sliderMotor1.config_kF(0, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN, TalonSRXEncoder.TIMEOUT_MS);
		//System.err.println(sliderMotor1.getControlMode());
		//System.err.print(sliderMotor1.getClosedLoopError());
	}

	public void setPositionMP(double targetPositionInches) {
		double startPositionInches = sliderMotor1.getPositionWorld();
		mpController.setMPTarget(startPositionInches, limitPosition(targetPositionInches), MP_MAX_VELOCITY_INCHES_PER_SEC, MP_T1, MP_T2);
		setFinished(false);
		firstMpPoint = true;
		setSliderControlMode(SliderControlMode.MOTION_PROFILE);
 	}

	private double limitPosition(double targetPosition) {

		if (targetPosition < MIN_POSITION_INCHES) {
			return MIN_POSITION_INCHES;
		}
		else if (targetPosition > MAX_POSITION_INCHES) {
			return MAX_POSITION_INCHES;
		}

		return targetPosition;
	}
	@Override
	public void setPeriodMs(long periodMs) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(sliderPIDParams, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		this.periodMs = periodMs;
	}
	/*@Override
	public void onStart(double timestamp) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Slider.this) {
			switch( getElevatorControlMode() ) {
				case JOYSTICK_PID:
					controlPidWithJoystick();
					break;
				case JOYSTICK_MANUAL:
					controlManualWithJoystick();
					break;
				case MOTION_PROFILE:
					if (!isFinished()) {
						if (firstMpPoint) {
							mpController.setPIDSlot(MP_SLOT);
							firstMpPoint = false;
						}
						setFinished(mpController.controlLoopUpdate());
						if (isFinished()) {
							mpController.setPIDSlot(PID_SLOT);
						}
					}
					break;
				default:
					break;
			}
		}
	}

*/





	public synchronized void controlLoopUpdate() {
		// Measure *actual* update period
		double currentTimestamp = Timer.getFPGATimestamp();
		if (lastControlLoopUpdateTimestamp > 0.001)	{	// ie, if this is NOT the first time
			lastControlLoopUpdatePeriod = currentTimestamp - lastControlLoopUpdateTimestamp;
		}
		lastControlLoopUpdateTimestamp = currentTimestamp;

		// Do the update
		if (sliderControlMode == SliderControlMode.JOYSTICK_MANUAL) {
			controlManualWithJoystick();
		}
		else if (!isFinished) {
			if (sliderControlMode == SliderControlMode.MOTION_PROFILE) {
				isFinished = mpController.controlLoopUpdate(getPositionInches());

			}
			if (sliderControlMode == SliderControlMode.JOYSTICK_PID){
				controlPidWithJoystick();
				//System.err.println(sliderMotor1.getControlMode());
			}
			if (sliderControlMode == SliderControlMode.MOTION_MAGIC){
				controlMMWithJoystick();
				//System.err.println(sliderMotor1.getControlMode());
			}

			/*else if (sliderControlMode == SliderControlMode.MP_PATH_VELOCITY) {
				isFinished = mpPathVelocityController.controlLoopUpdate(getGyroAngleDeg());
			}
			else if (sliderControlMode == SliderControlMode.ADAPTIVE_PURSUIT) {
				updatePose();
				isFinished = adaptivePursuitController.controlLoopUpdate(currentPose);
			}*/
		}
	}


	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getRightYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
	}
	private void controlMMWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getRightYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesMM = targetPositionInchesMM + deltaPosition;
		updatePositionMM(targetPositionInchesMM);
		//Robot.slider.targetPositionInchesPID = targetPositionInchesPID - (deltaPosition/3);
	}

	private void ControlWithJoystickhold(){
		double holdPosition = sliderMotor1.getPositionWorld();
		updatePositionPID(holdPosition);

	}

	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getRightYAxis();
		setSpeedJoystick(joyStickSpeed);
	}
	/*
	public void setShiftState(ElevatorSpeedShiftState state) {

			joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_HI;
			speedShift.set(true);
			mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		}
		else if(state == ElevatorSpeedShiftState.LO) {
			joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
			speedShift.set(false);
			mpController.setPID(pidPIDParamsLoGear, PID_SLOT);
		}
	}

	public ElevatorSpeedShiftState getShiftState() {
		return shiftState;
	}
*/
	public double getPositionInches() {
		return sliderMotor1.getPositionWorld();
	}

//	public double getAverageMotorCurrent() {
//		return (Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID)) / 3;
//	}



	public synchronized boolean isFinished() {
		return isFinished;
	}

	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}

	public double getPeriodMs() {
		return periodMs;
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		//System.err.println("the encoder is right after this");
			try {

				SmartDashboard.putNumber("Slider Ticks", sliderMotor1.getPositionWorld());
				SmartDashboard.putNumber("Slider Amps", sliderMotor1.getOutputCurrent());
				SmartDashboard.putNumber("slider error", sliderMotor1.getClosedLoopError());
				SmartDashboard.putNumber("slider motor output", sliderMotor1.getMotorOutputPercent());
				SmartDashboard.putNumber("slider velocity", sliderMotor1.getSelectedSensorVelocity());
				SmartDashboard.putNumber("Slider Target MM", targetPositionInchesMM);
//				SmartDashboard.putNumber("Elevator Motor 1 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 2 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 3 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				SmartDashboard.putNumber("Slider Target PID", targetPositionInchesPID);
			}
			catch (Exception e) {
				System.err.println("Drivetrain update status error" +e.getMessage());
			}

	}

	public static Slider getInstance() {
		if(instance == null) {
			instance = new Slider();
		}
		return instance;
	}
}
