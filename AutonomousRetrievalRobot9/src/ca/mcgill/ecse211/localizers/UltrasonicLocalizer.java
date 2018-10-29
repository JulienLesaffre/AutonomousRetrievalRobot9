package ca.mcgill.ecse211.localizers;

import ca.mcgill.ecse211.ARR.Navigation;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.sensors.DataController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to localize the angle of the robot using the Ultrasonic sensor
 *
 */
public class UltrasonicLocalizer extends Thread {
	
	private static final int THRESHOLD = 39;
	private static final int ERROR_MARGIN = 6;
  	private static final int ROTATE_SPEED = 40;
  	private static final int TURN_ANGLE = 360;
	private Odometer odo;
	private Navigation navigator;
	private DataController dataCont;

	
	/**
	 * Constructor
	 * @param leftMotor
	 * @param rightMotor
	 */
	public UltrasonicLocalizer(Navigation navigator) throws OdometerExceptions {
		this.odo = Odometer.getOdometer();
		this.dataCont = DataController.getDataController();
		this.navigator = navigator;
	}
	
	
	/**
	 * This thread localizes the robot using the falling edge procedure
	 */
	public void run() {
		
	    // reset motors
	    navigator.stopMotors();

	    // sleep 2 seconds
	    try {
	      Thread.sleep(2000);
	    } catch (InterruptedException e) {
	    }
		
		double x1 = 1.0, x2 = 1.0, y1 = 1.0, y2 = 1.0, d;
		double backWall = 360.0, leftWall = 360.0, dTheta;
		

		navigator.turnRobot(TURN_ANGLE, ROTATE_SPEED, true,  true);
		while(true) {
			d = dataCont.getD();
			if (d < THRESHOLD + ERROR_MARGIN) {
				x1 = odo.getXYT()[2];
				while(true) {
					d = dataCont.getD();
					if (d < THRESHOLD - ERROR_MARGIN) {
						x2 = odo.getXYT()[2];
						break;
					}
				}
				break;
			}
		}
	
		navigator.turnRobot(TURN_ANGLE, ROTATE_SPEED, false, true);
		
	    // sleep 2 seconds
	    try {
	      Thread.sleep(2000);
	    } catch (InterruptedException e) {
	    }
		
		while(true) {
			d = dataCont.getD();
			if (d < THRESHOLD + ERROR_MARGIN) {
				y1 = odo.getXYT()[2];
				while(true) {
					d = dataCont.getD();
					if (d < THRESHOLD - ERROR_MARGIN) {
						y2 = odo.getXYT()[2];
						break;
					}
				}
				break;
			}
		}
		
		navigator.stopMotors();	//reset motors
		backWall = (x1+x2)/2.0;
		leftWall = (y1+y2)/2.0;
		dTheta = dThetaFallingEdge(backWall, leftWall);
		correctAngle(dTheta);
	}

	
	public double dThetaFallingEdge(double backWall, double leftWall) {
		return 225.0 - (backWall+leftWall)/2.0;
	}
	
	public void correctAngle(double dTheta) {
		double newTheta = (odo.getXYT()[2] + dTheta) % 360;
		odo.setTheta(newTheta);
		int turnAngle = (int) (360.0 - (newTheta));
		navigator.turnRobot(turnAngle,ROTATE_SPEED, true, true);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
