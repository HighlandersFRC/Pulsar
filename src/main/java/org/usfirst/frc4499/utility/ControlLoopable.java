package org.usfirst.frc4499.utility;

public interface ControlLoopable 
{
	public void controlLoopUpdate();
	public void setPeriodMs(long periodMs);
}