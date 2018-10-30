package ca.mcgill.ecse211.ARR;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;

public class RingSet extends Thread {


	private static Odometer odometer;

	public static final EV3LargeRegulatedMotor leftMotor = Navigation.leftMotor;
	public static final EV3LargeRegulatedMotor rightMotor = Navigation.rightMotor;
	

	public RingSet() throws OdometerExceptions {
		RingSet.odometer = Odometer.getOdometer();

	}

	
	/**
	 * 
	 * @param x: x position of robot
	 * @param y: y position of robot
	 * @return: return the coordinates of the center of the closest Tile adjacent to the ring set from the robot
	 */
	private double[] findClosestTileCenter (double x, double y) {
		// TODO 
		double [] xy = {x,y};
		return xy;
	}
	
	private void detectRings() {
		
	}
	
	private void changeSide () {
		
	}
		
	
	/**
	 * Assume we are the red player
	 */
	public void run() {
		// out of the tunnel, it should be nice if the robot could localize again.
		double x = odometer.getXYT()[0];
		double y =odometer.getXYT()[1];
		Navigation.travelTo(findClosestTileCenter(x, y)[0], findClosestTileCenter(x,y)[1]);
		// at this point, the robot should be at the center of a Tile adjacent to the ring set.
		for (int i=0; i<=3; i++) {
			detectRings();
			changeSide();
		}
		// navigate to enter of tunnel
	}



}

