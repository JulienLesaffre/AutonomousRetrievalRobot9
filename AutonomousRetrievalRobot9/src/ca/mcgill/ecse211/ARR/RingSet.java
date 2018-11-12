package ca.mcgill.ecse211.ARR;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;

/**
 * 
 * This class handles the travel to the ring set and the ring pick up.
 *
 */
public class RingSet {

	private static Odometer odometer;
	private static SampleProvider leftSampleProvider;
	private static SampleProvider rightSampleProvider;

	private static final NXTRegulatedMotor ringPickUpMotor = new NXTRegulatedMotor(
			LocalEV3.get().getPort("C"));
	private static final EV3MediumRegulatedMotor lightSensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final int[] RS_LL = {0,0}; //RingSet Lower Left coordinates {x,y}
	private static final int[] RS_UR = {0,0}; //RingSet Upper Right coordinates {x,y}
	
	public RingSet(Odometer odo,SampleProvider leftLight, SampleProvider rightLight) throws OdometerExceptions {
		odometer = odo;
		leftSampleProvider = leftLight;
		rightSampleProvider = rightLight;

	}


	/**
	 * 
	 * @param ringColor
	 * @param topLevel
	 */
	public static void pickUpRing(int ringColor, boolean topLevel) {
		ringPickUpMotor.setSpeed(80);
		ringPickUpMotor.rotate(-3,false);
		ringPickUpMotor.rotate(60);
		ringPickUpMotor.stop();
		int distance = 22;
		Navigation.leftMotor.setSpeed(80);
		Navigation.rightMotor.setSpeed(80);
		Navigation.leftMotor.rotate(Navigation.convertDistance(distance), true);
		Navigation.rightMotor.rotate(Navigation.convertDistance(distance), false);
		ringPickUpMotor.rotate(-50,false);
		ringPickUpMotor.setSpeed(60);
		Navigation.leftMotor.rotate(-Navigation.convertDistance(10), true);
		Navigation.rightMotor.rotate(-Navigation.convertDistance(10), false);
		ringPickUpMotor.rotate(50,false);
		ringPickUpMotor.rotate(-50,false);
		Navigation.leftMotor.rotate(-Navigation.convertDistance(distance-10), true);
		Navigation.rightMotor.rotate(-Navigation.convertDistance(distance-10), false);
		ringPickUpMotor.rotate(-40,false);

	}
	
	/**
	 * drop the rings stored on the claw
	 */
	public static void dropRings() {
		Navigation.leftMotor.setSpeed(60);
		Navigation.rightMotor.setSpeed(60);
		ringPickUpMotor.setSpeed(100);
		ringPickUpMotor.rotate(40);
		Navigation.leftMotor.rotate(-400, true);
		Navigation.rightMotor.rotate(-400, true);
		ringPickUpMotor.setSpeed(200);
		for (int i=0; i<= 6; i++) {
			ringPickUpMotor.rotate(10);
			ringPickUpMotor.rotate(-10);		
		}
	}
	
	

	/**
	 * Detect the rings and their color
	 */
	private static void detectRings() {
		int distanceToTravel = 20;
		Navigation.moveStraight(distanceToTravel, true, true);
		int colorDetected;
		boolean lineDetected = false;
		boolean ringDetected = false;
		while (Navigation.leftMotor.isMoving()) {
			colorDetected = RingDetection.colorDetection();
			switch (colorDetected) {
				case 1:
					Sound.beep();
					break;
				case 2:
					Sound.beep();
					break;
				case 3:
					Sound.beep();
					break;
				case 4:
					Sound.beep();
					break;
				default:
					break;
			}
			
			String colorDetectedString = Integer.toString(colorDetected);
			AutonomousRetrievalRobot.lcd.drawString("Color: " + colorDetectedString, 0, 0);
		}
		

	}
	
	public static void findLineAhead() throws OdometerExceptions {
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;
		float[] newColorLeft = {0};
		float oldSampleLeft = 0;
		float[] newColorRight = {0};
		float oldSampleRight = 0;
		int colorDetected = RingDetection.colorDetection();
		Navigation.setSpeedAcceleration(60, 500);
		Navigation.leftMotor.forward();
		Navigation.rightMotor.forward();
		boolean ringDetected = false;
		while(!ringDetected || Navigation.leftMotor.isMoving() || Navigation.rightMotor.isMoving()) {
			ringDetected = RingDetection.startOfRing();
			if(ringDetected)
				break;

					
			// Get color sensor readings
			leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
			rightSampleProvider.fetchSample(newColorRight, 0); 

			// If line detected for left sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft[0]) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				Navigation.leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight[0]) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				Navigation.rightMotor.stop(true);
				foundRight++;
			}

			// Store last color readings
			oldSampleLeft = newColorLeft[0];
			oldSampleRight = newColorRight[0];

			// If line found for both sensors, exit
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}
		Navigation.moveStraight(1.5, true, false);
		colorDetected = RingDetection.colorDetection();
		
		String colorDetectedString = Integer.toString(colorDetected);
		AutonomousRetrievalRobot.lcd.drawString("Color: " + colorDetectedString, 0, 0);
		
		
	}
	


	public static void ringSetMain() {
		for (int i = 0; i <= 3; i++) {
			detectRings();
			Navigation.turnRobot(90,true,false);
		}

	}
	

	public static void testRingSet() throws OdometerExceptions {
		findLineAhead();

	}
	

}
