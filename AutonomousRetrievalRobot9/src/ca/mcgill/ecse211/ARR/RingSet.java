package ca.mcgill.ecse211.ARR;

import lejos.hardware.Sound;
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

	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static NXTRegulatedMotor ringPickUpMotor;
	private static EV3MediumRegulatedMotor lightSensorMotor;
	
	
	private static final int[] RS_LL = {0,0}; //RingSet Lower Left coordinates {x,y}
	private static final int[] RS_UR = {0,0}; //RingSet Upper Right coordinates {x,y}
	
	public RingSet(EV3LargeRegulatedMotor lMotor, EV3LargeRegulatedMotor rMotor, Odometer odo, 
			NXTRegulatedMotor pickUpMotor, EV3MediumRegulatedMotor sensorMotor) throws OdometerExceptions {
		leftMotor = lMotor;
		rightMotor = rMotor;
		ringPickUpMotor = pickUpMotor;
		lightSensorMotor = sensorMotor;
		odometer = odo;
	}


	/**
	 * 
	 * @param ringColor
	 * @param topLevel
	 */
	public static void pickUpRing(int ringColor, boolean topLevel) {
		ringPickUpMotor.setSpeed(80);
		ringPickUpMotor.rotate(-1,false);
		ringPickUpMotor.rotate(92);
		ringPickUpMotor.stop();
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(250, true);
		rightMotor.rotate(250, false);
		ringPickUpMotor.rotate(-10,false);
		leftMotor.rotate(-200, true);
		rightMotor.rotate(-200, false);
		ringPickUpMotor.rotate(-80);
		ringPickUpMotor.stop();
	}
	
	public static void dropRings() {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		ringPickUpMotor.rotate(85);
		leftMotor.rotate(-400, true);
		rightMotor.rotate(-400, true);
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
			Display.lcd.drawString(colorDetectedString, 0, 0);
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
//		dropRings();
//		ringSetMain();
	}
	

}