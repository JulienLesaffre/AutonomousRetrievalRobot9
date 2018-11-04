package ca.mcgill.ecse211.ARR;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.sensors.LightController;



/**
 * This class handles the movement of the robot
 * it initializes the motors and all motor access should be through this class.
 * Methods are defined as static and any other class can refer statically
 * to control the movement of robot.
 *
 */
public class Navigation {

	// Game parameters (will be provided with Wifi Class)
	// - TR: team red, TG: team green, BR: bridge
	public static final int RedTeam = 0; 		//this theyll probs give i.e. redTeam = 9 for our group number
	public static final int GreenTeam = 0; 
	public static final int RedCorner = -1; 	//i think this is a number from 0-3
	public static final int GreenCorner = -1;
	public static final int[] Red_LL = {0,0}; 
	public static final int[] Red_UR = {0,0}; 
	public static final int[] Green_LL = {0,0}; 
	public static final int[] Green_UR = {0,0}; 
	public static final int[] BRR_LL = {0,0}; 
	public static final int[] BRR_UR = {0,0};
	public static final int[] BRG_LL = {0,0}; 
	public static final int[] BRG_UR = {0,0}; 
	public static final int[] TR_LL = {0,0}; 
	public static final int[] TR_UR = {0,0}; 
	public static final int[] TG_LL = {0,0}; 
	public static final int[] TG_UR = {0,0}; 
	
	
	private static final int ROTATE_SPEED = 100;
	private static final int NAV_WITH_CORR_SPEED = 130;
	private static final int NAV_WITH_CORR_ACCEL = 1000;
	public static final double WHEEL_RAD = 0;
	public static final double TRACK = 0;
	public static final double SQUARE_SIZE = 30.48;
	public static final double SENSOR_OFFSET = 6.4;

	private static Odometer odometer;
	private static boolean isNavigating;
	private static boolean isTunnelVertical;

	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	
	public Navigation() throws OdometerExceptions {
		Navigation.odometer = Odometer.getOdometer();
	}
	
	
	
	/**
	 * This method asssumes the robot has finished localizing, it takes its coordinates
	 * and calculates the trajectory to the tunnel. The trajectory is based on horizontal
	 * and vertical movements only. The robot travels x and y to reach the tile before
	 * the start of the tunnel where it is able to localize all odometer data accurately.
	 * This way the robot will be accurately within one tile of the tunnel in order to travel
	 * through without problems.
	 * 
	 * Once x and y travel is calculated, it repeatedly calls on travelToWithCorrection to 
	 * move while constantly correcting using the sensors.
	 */
	public static void travel_Start_To_Tunnel() {
		
		//This method assumes the robot has just localized and is starting, or has dropped off ring
		// and is going to ring holder again.
		//find the best trajectory based on 
		// 1) location and heading of robot
		// 2) the location of the tunnel 
		// 3) size and type of region (i.e. green means its tunnel is in region)
		
		//robot starts at outer edge of corner tile, if tunnel is on the +-1 tile on either x or y axis, go up or down by half tile then travel
		//if tunnel is more than +1 difference, then travel along the lines (3 and larger)
		// - for both cases, the final movement should be towards direction parallel to tunnel
		// - tunnel either horizontal or vertical
		// - first movement will be to within one full tile from center of tunnel
		
		//take the tunnel coordinates and calculate the midpoint and whether tunnel is vertically or horizontally laid
		findTunnelHeading();
		double[] tunnelMidpoint = findTunnelMidpoint();
		
		//if vertically laid, move by x first to within one tile of middle of tunnel
		if(isTunnelVertical) {
			double myX = odometer.getXYT()[0];
			double midpointToRobot = Math.abs(tunnelMidpoint[0] - myX);
			//if midpoint x coordinates are smaller, turn left (270)
			if(tunnelMidpoint[0] < myX) {
				turnTo(270); //turn to 270
			} else {
				turnTo(90);
			}
			//move and while moving make sure to correct and update odometer: if distance tunnel center to robot 3.5 you will cross 2 lines, 2.5 cross 1
			//	move to coordinate of midpointToRobot - 1(tile) in direction you are coming from
			//	once you cross line stop and ask if you have reached the coordinate, if not start travel again.
		}
		
		
		//then turn 90 degrees, if y coordinate of tunnell is smaller than robots and heading is west turn left, if value smaller and heading east turn right
		//		if tunnel larger y and moving east, turn left, if tunnel larger y and moving west turn right
		//then travel y axis to the tile before the tunnel
		//stop, move forward by offset, and turn and localize again according to that position, update odometer values
		
		//if horizontally laid out, move by y first
		
		
		
		
		
	}
	
	
	/**
	 * This method makes the robot move in the direction of the
	 * waypoint whose coordinates are passed as arguments. The robot
	 * moves until a perpendicular line is crossed where the first sensor
	 * to cross the line that motor will stop while the other wheel moves until
	 * that side's sensor picks up the same line, correcting the robots heading.
	 * The method expects the robot to initially be traveling horizontally or vertically.
	 * 
	 * Assumption: the two front light sensors must be polling
	 * 
	 * @param x : x coordinate
	 * @param y : y coordinate
	 * @throws OdometerExceptions
	 */ 
	public static void travelToWithCorrection(double x, double y) throws OdometerExceptions {
		// Define variables
		double odometer[] = { 0, 0, 0 }, absAngle = 0, dist = 0, deltaX = 0, deltaY = 0;
		
		// Get odometer readings
		odometer = Odometer.getOdometer().getXYT();
		
		x = x * SQUARE_SIZE;
		y = y * SQUARE_SIZE;
		
		// Get displacement to travel on X and Y axis
		deltaX = x - odometer[0];
		deltaY = y - odometer[1];

		// Displacement to point (hypothenuse)
		dist = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));
		
		setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);
		

		// Get absolute angle the robot must be facing
		absAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

		turnTo(absAngle);
		
		//if displacement is less than a tile then we will not cross line, so travel normally to coordinate
		if(dist <= SQUARE_SIZE) {
			moveStraight(dist, true, false);
		} else {
			double oldSampleRight = 0;
			double oldSampleLeft = 0;
			double newColorLeft;
			double newColorRight;


			leftMotor.forward();
			rightMotor.forward();

			int foundLeft = 0; int foundRight = 0;
			
			while(true) {
				// Get color sensor readings
				newColorLeft = LightController.colorLeft;
				newColorRight = LightController.colorRight;

				// If line detected for left sensor (intensity less than 0.4), only count once by keeping track of last value
				if((newColorLeft) < 0.4 && oldSampleLeft > 0.4 && foundLeft == 0) {
					leftMotor.stop(true);
					foundLeft++;
				}
				// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
				if((newColorRight) < 0.4 && oldSampleRight > 0.4 && foundRight == 0) {
					rightMotor.stop(true);
					foundRight++;
				}
				// Store last color samples
				oldSampleLeft = newColorLeft;
				oldSampleRight = newColorRight;

				// If line found for both sensors, exit
				if(foundLeft == 1 && foundRight == 1) {
					break;
				}
			}
			//perform correction of odometer here
			correctOdometer();
			
			
		}	
	}
	
	//if moving along x axis only allowed to correct x, if along y axis then correct y
	public static void correctOdometer() {
		
	}
	
	
	/**
	 * This method finds out if the tunnel is vertical or horizontal and sets the static variable.
	 * Used by travel_Start_To_Tunnel method to calculate ideal trajectory.
	 */
	public static void findTunnelHeading() {
		int differenceInXCoord = BRR_LL[0] - BRR_UR[0];
		if(differenceInXCoord == 2) isTunnelVertical = false;
		else isTunnelVertical = true;
	}
	
	/**
	 * This method finds and returns the midpoint of the start of the tunnel relative 
	 * to the starting zone and team our robot is on. It assumes the find tunnel heading
	 * is already called and heading is calculated.
	 * 
	 * @return midpoint: double[] coordinates in actual distance measurement
	 */
	public static double[] findTunnelMidpoint() {
		double[] midpoint = {0,0};
		if(isTunnelVertical) {
			midpoint[1] = (BRR_LL[1]) * SQUARE_SIZE;
			midpoint[0] = (BRR_LL[0] + 0.5) * SQUARE_SIZE;
		} else {
			midpoint[0] = (BRR_LL[0]) * SQUARE_SIZE;
			midpoint[1] = (BRR_LL[1] + 0.5) * SQUARE_SIZE;
		}
		return midpoint;
	}
	

	/**
	 * This method causes the robot to travel to the absolute field location (x,
	 * y),specified in tile points.
	 * @param x: x coordinate
	 * @param y: y coordinate
	 */
	public static void travelTo(double x, double y) {
		double x0 = odometer.getXYT()[0];
		double y0 = odometer.getXYT()[1];
		double theta0 = odometer.getXYT()[2];
		double xVector = x - x0;
		double yVector = y - y0;
		double vectorDistance = Math.sqrt(Math.pow(xVector, 2) + Math.pow(yVector, 2));
		double thetaVector = Math.toDegrees(Math.atan2(xVector, yVector));
		turnTo(thetaVector - theta0);
		leftMotor.rotate(convertDistance(vectorDistance), true);
		rightMotor.rotate(convertDistance(vectorDistance), true);
	}

	/**
	 * This method causes the robot to turn (on point) to the absolute heading
	 * theta. This method should turn a MINIMAL angle to its target.
	 * 
	 * @param theta:theta angle
	 */
	public static void turnTo(double theta) {
		if (theta >= 180) {
			theta = -(360 - theta);
		} else if (theta < -180) {
			theta = (360 + theta);
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(theta), true);
		rightMotor.rotate(-convertAngle(theta), false);

	}
	

	/**
	 * 
	 * @param distance: distance to travel
	 * @param forwards: if true then it goes forward direction
	 * @param continueRunning: if true then program does not wait for wheels to stop, false
	 *            program waits
	 */
	public static void moveStraight(double distance, boolean forwards, boolean continueRunning) {
		int i = 1;
		if (!forwards) i = -1;
		leftMotor.rotate(convertDistance(i * distance), true);
		rightMotor.rotate(convertDistance(i * distance), continueRunning);
	}

	/**
	 * This method turns the robot to the right or left depending on direction
	 * boolean and turns the robot by the specified degrees amount.
	 * 
	 * @param degrees: degrees to turn by
	 * @param direction: true means turn right, left otherwise
	 */
	public static void turnRobot(int degrees, boolean direction, boolean continueRunning) {
		int i = 1;
		if (!direction) i = -1;
		leftMotor.rotate(i * convertAngle(degrees), true);
		rightMotor.rotate(i * -convertAngle(degrees), continueRunning);
	}
	
	public void stopMotors() {
		leftMotor.stop(true);
		rightMotor.stop(true);
	}
	
	/**
	 * This method sets the speed and acceleration of the robot
	 * @param speed
	 * @param acceleration
	 */
	public static void setSpeedAcceleration(int speed, int acceleration) {
		Navigation.leftMotor.setAcceleration(acceleration);
		Navigation.rightMotor.setAcceleration(acceleration);
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
	}

	public static int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

	public static int convertAngle(double angle) {
		return convertDistance(Math.PI * TRACK * angle / 360.0);
	}
	
	/**
	 * @return true if another thread has called travelTo() or turnTo() and the
	 * method has yet to return; false otherwise.
	 */
	public boolean isNavigating() {
		return isNavigating;
	}

}
