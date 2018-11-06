package ca.mcgill.ecse211.ARR;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;

/**
 * 
 * This class handles the travel to the ring set and the ring pick up.
 *
 */
public class RingSet {

	private static Odometer odometer;

	private static final EV3LargeRegulatedMotor leftMotor = Navigation.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = Navigation.rightMotor;
	private static final NXTRegulatedMotor ringPickUpMotor = new NXTRegulatedMotor(
			LocalEV3.get().getPort("C"));
	private static final EV3MediumRegulatedMotor lightSensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	

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
		ringPickUpMotor.setSpeed(80);
		ringPickUpMotor.rotate(-1,false);
		ringPickUpMotor.rotate(55);
		ringPickUpMotor.stop();
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(280, true);
		rightMotor.rotate(280, false);
		ringPickUpMotor.rotate(-10,false);
		leftMotor.rotate(-200, true);
		rightMotor.rotate(-200, false);
		ringPickUpMotor.rotate(-50);
		ringPickUpMotor.stop();
	}
	
	public static void dropRings() {
		Navigation.leftMotor.setSpeed(100);
		Navigation.rightMotor.setSpeed(100);
		ringPickUpMotor.setSpeed(200);
		ringPickUpMotor.rotate(85);
		Navigation.leftMotor.rotate(-400, true);
		Navigation.rightMotor.rotate(-400, true);
		for (int i=0; i<= 5; i++) {
			ringPickUpMotor.rotate(10);
			ringPickUpMotor.rotate(-10);		
		}
	}
	
	

	/**
	 * Detect the rings and their color
	 */
	private static void detectRings() {
		while (true) {	
			int colorDetected = RingDetection.colorDetection();
			String colorDetectedString = Integer.toString(colorDetected);
			AutonomousRetrievalRobot.lcd.drawString(colorDetectedString, 0, 0);
		}
		

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
		dropRings();

		
	}
	

}
