package ca.mcgill.ecse211.ARR;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.sensors.USController;

/**
 * 
 * This class handles the travel to the ring set and the ring pick up.
 *
 */
public class RingSet {

	private static Odometer odometer;

	public static final EV3LargeRegulatedMotor leftMotor = Navigation.leftMotor;
	public static final EV3LargeRegulatedMotor rightMotor = Navigation.rightMotor;
	public static final NXTRegulatedMotor ringPickUpMotor = new NXTRegulatedMotor(
			LocalEV3.get().getPort("C"));
	

	public RingSet() throws OdometerExceptions {
		RingSet.odometer = Odometer.getOdometer();

	}

	/**
	 * 
	 * @param x:
	 *            x position of robot
	 * @param y:
	 *            y position of robot
	 * @return: return the coordinates of the center of the closest Tile adjacent to
	 *          the ring set from the robot
	 */
	private static double[] findClosestTileCenter(double x, double y) {
		double[] xy = { x, y };
		return xy;
	}

	/**
	 * Handles the picking up of a ring
	 */
	public static void pickUpRing() {
		ringPickUpMotor.setSpeed(100);
		ringPickUpMotor.rotate(90);
		ringPickUpMotor.stop();
		Navigation.leftMotor.setSpeed(100);
		Navigation.rightMotor.setSpeed(100);
		Navigation.leftMotor.rotate(200, true);
		Navigation.rightMotor.rotate(200, false);
		Navigation.leftMotor.rotate(-200, true);
		Navigation.rightMotor.rotate(-200, false);
		ringPickUpMotor.rotate(-80);

	}
	
	

	/**
	 * Detect the rings and their color
	 */
	private static void detectRings() {

	}

	/**
	 * The robot moves to the center of the next tile adjacent to the ring set in
	 * order to be ready to detect a ring on a new side of the ring set
	 */
	private static void changeSide() {

	}

	public static void ringSetMain() {
		double x = odometer.getXYT()[0];
		double y = odometer.getXYT()[1];
		Navigation.travelTo(findClosestTileCenter(x, y)[0], findClosestTileCenter(x, y)[1]);
		// at this point, the robot should be at the center of a Tile adjacent to the
		// ring set.
		for (int i = 0; i <= 3; i++) {
			detectRings();
			changeSide();
		}
		// navigate to enter of tunnel
	}
	

	public static void testRingSet() {
		pickUpRing();

		
	}
	

}
