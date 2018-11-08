package ca.mcgill.ecse211.ARR;

import lejos.hardware.Sound;
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
	private static final int[] RS_LL = {0,0}; //RingSet Lower Left coordinates {x,y}
	private static final int[] RS_UR = {0,0}; //RingSet Upper Right coordinates {x,y}
	
	public RingSet() throws OdometerExceptions {
		RingSet.odometer = Odometer.getOdometer();

	}


	/**
	 * 
	 * @param ringColor
	 * @param topLevel
	 */
	public static void pickUpRing(int ringColor, boolean topLevel) {
		ringPickUpMotor.setSpeed(80);
		ringPickUpMotor.rotate(-1,false);
		ringPickUpMotor.rotate(55);
		ringPickUpMotor.stop();
		Navigation.leftMotor.setSpeed(100);
		Navigation.rightMotor.setSpeed(100);
		Navigation.leftMotor.rotate(280, true);
		Navigation.rightMotor.rotate(280, true);
		ringPickUpMotor.setSpeed(80);
		ringPickUpMotor.rotate(-10,false);
		Navigation.leftMotor.rotate(-200, true);
		Navigation.rightMotor.rotate(-200, false);
		ringPickUpMotor.rotate(-50);
		ringPickUpMotor.stop();
	}
	
	/**
	 * drop the rings stored on the claw
	 */
	public static void dropRings() {
		Navigation.leftMotor.setSpeed(100);
		Navigation.rightMotor.setSpeed(100);
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
		int distanceToTravel = 10;
		Navigation.moveStraight(distanceToTravel, true, true);
		double xStart = odometer.getXYT()[0];
		double yStart = odometer.getXYT()[1];
		double xEnd = odometer.getXYT()[0];
		double yEnd = odometer.getXYT()[1];
		int colorDetected;
		boolean ringDetected = false;
		while ((Math.abs(xStart-xEnd) < distanceToTravel) || (Math.abs(yStart-yEnd) < distanceToTravel) || !ringDetected ) {
			xEnd = odometer.getXYT()[0];
			yEnd = odometer.getXYT()[1];
			colorDetected = RingDetection.colorDetection();
			switch (colorDetected) {
				case 1:
					Sound.beep();
					ringDetected = true;
					break;
				case 2:
					Sound.beep();
					Sound.beep();
					ringDetected = true;
					break;
				case 3:
					Sound.beep();
					Sound.beep();
					Sound.beep();
					ringDetected = true;
					break;
				case 4:
					Sound.beep();
					Sound.beep();
					Sound.beep();
					Sound.beep();
					ringDetected = true;
					break;
				default:
					break;
			}
				
			String colorDetectedString = Integer.toString(colorDetected);
			AutonomousRetrievalRobot.lcd.drawString(colorDetectedString, 0, 0);
		}
		

	}


	public static void ringSetMain() {
		for (int i = 0; i <= 3; i++) {
			detectRings();
			Navigation.turnTo(90);
		}

	}
	

	public static void testRingSet() {
		pickUpRing(1,true);

	}
	

}
