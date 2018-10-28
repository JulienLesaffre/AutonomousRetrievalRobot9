package ca.mcgill.ecse211.ARR;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;

public class Navigation {

	private static final int ROTATE_SPEED = 150;
	private static final double WHEEL_RAD = 0;
	private static final double TRACK = 0;


	private static Odometer odometer;
	private static boolean isNavigating;
	
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));


	public Navigation(double coordinates[][]) throws OdometerExceptions {
		Navigation.odometer = Odometer.getOdometer();

	}
	


	
	// This method causes the robot to travel to the absolute field location (x, y),
	// specified in tile points. This method should continuously call turnTo(double
	// theta) and then set to motor speed to forward(straight). This will make sure
	// that your heading is updated until you reach your exact goal. This method
	// will poll the odometer for information.
	public static void travelTo(double x, double y) {
		double x0 = odometer.getXYT()[0];
		double y0 = odometer.getXYT()[1];
		double theta0 = odometer.getXYT()[2];
		double xVector = x - x0;
		double yVector = y - y0;
		double vectorDistance = Math.sqrt(Math.pow(xVector, 2) + Math.pow(yVector, 2));
		double thetaVector = Math.toDegrees(Math.atan2(xVector, yVector));
		turnTo(thetaVector - theta0);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, vectorDistance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, vectorDistance), true);
	}


	// This method causes the robot to turn (on point) to the absolute heading
	// theta. This method should turn a MINIMAL angle to its target.
	private static void turnTo(double theta) {

		if (theta >= 180) {
			theta = -(360 - theta);
		} else if (theta < -180) {
			theta = (360 + theta);
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD,TRACK, theta), false);

	}

	// This method returns true if another thread has called travelTo() or turnTo()
	// and the method has yet to return; false otherwise.
	public boolean isNavigating() {
		return isNavigating;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}



}
