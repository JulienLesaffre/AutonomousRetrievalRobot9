package ca.mcgill.ecse211.ARR;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.ARR.Ring.Side;
import ca.mcgill.ecse211.odometer.*;

/**
 * 
 * This class handles the travel to the ring set and the ring pick up.
 *
 */
public class RingController {

	//class variables
	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static EV3LargeRegulatedMotor poleMotor;
	private static EV3MediumRegulatedMotor clawMotor;
	private static SampleProvider sampleProvider;
	
	//store ring information
	private static Ring ringNorth;
	private static Ring ringEast;
	private static Ring ringSouth;
	private static Ring ringWest;
	
	//speed and acceleration
	private static final int COLOR_DETECTION_SPEED = 100;
	private static final int COLOR_DETECTION_ACCEL = 1000;
	
	//distance and angles
	private static final int LOWER_RING_ANGLE_OFFSET = 60;
	private static final int CLAW_GRAB_ANGLE_FULL = 110;
	private static final int CLAW_GRAB_ANGLE_HALF = 50;
	private static final int POLE_TOPRING_ANGLE = 44;
	private static final int POLE_BOTTOMRING_ANGLE = 60;
	private static final double POLE_JAB_DISTANCE = 7;
	
	
	
	public RingController(Odometer odo, EV3LargeRegulatedMotor lMotor, EV3LargeRegulatedMotor rMotor, 
			EV3LargeRegulatedMotor pMotor, EV3MediumRegulatedMotor cMotor, SampleProvider sp) throws OdometerExceptions {
		odometer = odo;
		leftMotor = lMotor;
		rightMotor = rMotor;
		poleMotor = pMotor;
		clawMotor = cMotor;
		sampleProvider = sp;
	}



	public static void pickUpTwoRings() throws OdometerExceptions {
		raisePole();
		
		for(int i = 2; i > 0; i--) {
			
			Navigation.moveStraight(Navigation.SENSOR_OFFSET, true, false);
			Navigation.findLineStraight(true);
			Navigation.moveStraight(Navigation.SENSOR_OFFSET + 3 , true, false); //move by + 3 so pole doesnt hit the ring on the way down
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
			Navigation.findLineStraight(true);
			Navigation.moveStraight(Navigation.SENSOR_OFFSET + Navigation.POLE_CENTER_OFFSET, true, false);
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
			
			//now retrieve which ring is here, and act accordingly
			Ring ringAhead = getRingAhead();
			
			
			
			clawMotor.rotate(-CLAW_GRAB_ANGLE_FULL);		//extend claw
			
			if(ringAhead.isUp) 								//bring down pole
				poleMotor.rotate(POLE_TOPRING_ANGLE);			
			else 
				poleMotor.rotate(POLE_BOTTOMRING_ANGLE);
			
			poleMotor.stop(true);							//hold pole in place
			
			Navigation.moveStraight(POLE_JAB_DISTANCE, true, false);
			
			if(ringAhead.ringColor == 4) 					//if its orange only do half a grab
				clawMotor.rotate(CLAW_GRAB_ANGLE_HALF);
			else 
				clawMotor.rotate(CLAW_GRAB_ANGLE_FULL);
			
			
			//move back, raise pole and hook claw
			Navigation.moveStraight(POLE_JAB_DISTANCE - 3, false, true);
			poleMotor.setAcceleration(500);
			poleMotor.setSpeed(40);
			poleMotor.setStallThreshold(10, 2);
			poleMotor.rotate(-180, true);
			if(ringAhead.ringColor == 4) 					//extend claw back to push ring in pole
				clawMotor.rotate(-CLAW_GRAB_ANGLE_HALF, true);
			else 
				clawMotor.rotate(-CLAW_GRAB_ANGLE_FULL, true);
			while(true) {
				if(poleMotor.isStalled()) {
					poleMotor.stop(true);
					break;
				}
			}
			
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, true, false);
		}
		
	}
	
	
	
	
	
	public static Ring getRingAhead() {
		int ringSetX, ringSetY;
		if(Navigation.RedTeam == 9) {
			ringSetX = Navigation.TR_x;
			ringSetY = Navigation.TR_y;
		} else {
			ringSetX = Navigation.TG_x;
			ringSetY = Navigation.TG_y;
		}
		
		double myX = odometer.getXYT()[0] / Navigation.SQUARE_SIZE;
		double myY = odometer.getXYT()[1] / Navigation.SQUARE_SIZE;
		int x = (int) Math.round(myX);
		int y = (int) Math.round(myY);

		if(ringSetX > x) 
			return ringWest;
		else if(ringSetX < x)
			return ringEast;
		else if(ringSetY > y)
			return ringSouth;
		else
			return ringNorth;
	}

	
	
	public static void detectAllRings() throws OdometerExceptions {
		
		raisePole();

		Navigation.moveStraight(Navigation.RING_DETECTION_OFFSET, true, false);
		Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
		

		float[] rgbValues = new float[sampleProvider.sampleSize()];
		int colorDetected = 0;
		
		Navigation.setSpeedAcceleration(COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL);
		Navigation.moveStraight(Navigation.SQUARE_SIZE, true, true);
		
		while(leftMotor.isMoving()) {
			sampleProvider.fetchSample(rgbValues, 0);
			colorDetected = RingDetection.detectColor(rgbValues);
			if(colorDetected != 0) {
				Navigation.stopMotors();
				break;
			}

		}
		
		Navigation.moveStraight(1, true, false);
		
		sampleProvider.fetchSample(rgbValues, 0);
		colorDetected = RingDetection.detectColor(rgbValues);
		Display.displayRingColor(colorDetected);
		
		switch (colorDetected) {
		case 1:
			Sound.beep();
			break;
		case 2:
			Sound.beep();
			Sound.beep();
			break;
		case 3:
			Sound.beep();
			Sound.beep();
			Sound.beep();
			break;
		case 4:
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			break;
		}
		
		//if color detected save value
		if(colorDetected != 0) saveRingDetection(true, colorDetected);
		else {
			//else search bottom ring half
			Navigation.findLineStraight(false);				//move to line
			poleMotor.rotate(LOWER_RING_ANGLE_OFFSET);
			
			sampleProvider.fetchSample(rgbValues, 0);
			colorDetected = RingDetection.detectColor(rgbValues);
			Display.displayRingColor(colorDetected);
			
			switch (colorDetected) {
			case 1:
				Sound.beep();
			case 2:
				Sound.beep();
				Sound.beep();
			case 3:
				Sound.beep();
				Sound.beep();
				Sound.beep();
			case 4:
				Sound.beep();
				Sound.beep();
				Sound.beep();
				Sound.beep();
			}
			
			if(colorDetected != 0) saveRingDetection(false, colorDetected);
			
		}
	}
	
	
	private static void saveRingDetection(boolean ringIsOnTop, int colorCode) {
		//if robot is facing north, ringset is to west, so its east side
		Side ringSide = Side.Null;
		String robotHeading = Navigation.getCurrentHeading();
		if(robotHeading.equalsIgnoreCase("north")) { 						//heading north, if ringset x smaller than tunnel x it's left
			ringSide = Side.East;
			ringEast = new Ring(ringIsOnTop, colorCode, ringSide);
		} else if (robotHeading.equalsIgnoreCase("east")) {					//heading east, if ringset y is larger than tunnel y it's left
			ringSide = Side.South;
			ringSouth = new Ring(ringIsOnTop, colorCode, ringSide);
		} else if (robotHeading.equalsIgnoreCase("south")) { 				//heading south, if ringset x is larger than tunnel x it's left
			ringSide = Side.West;
			ringWest = new Ring(ringIsOnTop, colorCode, ringSide);
		} else if (robotHeading.equalsIgnoreCase("west")) { 				//heading west, if ringset y is smaller than tunnel y it's left
			ringSide = Side.North;
			ringNorth = new Ring(ringIsOnTop, colorCode, ringSide);
		}
	}
	
	
	public static void raisePole() {
		//make sure color sensor is at its highest
		poleMotor.setAcceleration(500);
		poleMotor.setSpeed(40);
		poleMotor.setStallThreshold(10, 2);
		poleMotor.rotate(-180, true);
		while(true) {
			if(poleMotor.isStalled()) {
				poleMotor.stop(true);
				break;
			}
		}
	}
	

	
	/**
	 * drop the rings stored on the claw
	 */
	public static void dropRings() {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		poleMotor.rotate(85);
		leftMotor.rotate(-400, true);
		rightMotor.rotate(-400, true);
		for (int i=0; i<= 5; i++) {
			poleMotor.rotate(10);
			poleMotor.rotate(-10);		
		}
	}
	
	/*
	poleMotor.setAcceleration(500);
	poleMotor.setSpeed(70);
	poleMotor.rotate(-45);
	poleMotor.stop();
	ringClawMotor.setAcceleration(500);
	ringClawMotor.setSpeed(50);
	ringClawMotor.rotate(110);
	ringClawMotor.stop();
	*/
	
	/*
	poleMotor.setAcceleration(500);
	poleMotor.setSpeed(40);
	poleMotor.setStallThreshold(10, 2);
	poleMotor.rotate(-180, true);
	while(true) {
		if(poleMotor.isStalled()) {
			poleMotor.stop(true);
			break;
		}
	}
	
	Navigation.setSpeedAcceleration(80, 500);
	LightLocalizer.testClaw();
	
	
	
	
	
	ringClawMotor.rotate(-110);
	poleMotor.rotate(44);
	poleMotor.stop(true);
	Navigation.moveStraight(7, true, false);
	ringClawMotor.rotate(50);
	Navigation.moveStraight(7, false, false);
	*/
	

}