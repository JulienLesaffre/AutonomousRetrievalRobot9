package ca.mcgill.ecse211.ARR;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;


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
	public static final int RedCorner = 0; 	//i think this is a number from 0-3
	public static final int GreenCorner = -1;
	public static final int[] Red_LL = {0,0}; 
	public static final int[] Red_UR = {0,0}; 
	public static final int[] Green_LL = {0,0}; 
	public static final int[] Green_UR = {0,0}; 
	public static final int[] BRR_LL = {3,3}; 
	public static final int[] BRR_UR = {4,5};
	public static final int[] BRG_LL = {0,0}; 
	public static final int[] BRG_UR = {0,0}; 
	public static final int[] TR_LL = {0,0}; 
	public static final int[] TR_UR = {0,0}; 
	public static final int[] TG_LL = {0,0}; 
	public static final int[] TG_UR = {0,0}; 
	
	
	private static final int ROTATE_SPEED = 150;
	private static final int ROTATE_ACC = 1500;
	private static final int NAV_WITH_CORR_SPEED = 190;
	private static final int NAV_WITH_CORR_ACCEL = 1500;
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 12.85;
	public static final double SQUARE_SIZE = 30.48;
	public static final double SENSOR_OFFSET = 4.9;

	private static Odometer odometer;
	private static boolean isNavigating;
	private static boolean isTunnelVertical;

	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final int NAV_WITH_CORR_ACC = 0;
	
	
	private static SampleProvider leftSampleProvider;
	private static SampleProvider rightSampleProvider;

	
	public Navigation(SampleProvider leftLight, SampleProvider rightLight, Odometer odo) throws OdometerExceptions {
		Navigation.odometer = odo;
		leftSampleProvider = leftLight;
		rightSampleProvider = rightLight;
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
	public static void travel_Start_To_Tunnel() throws OdometerExceptions{

		//take the tunnel coordinates and calculate the midpoint and whether tunnel is vertically or horizontally laid
		findTunnelHeading();
		double[] tunnelMidpoint = findTunnelMidpoint();
		System.out.println("" + isTunnelVertical);
		double myX = odometer.getXYT()[0];
		double myY = odometer.getXYT()[1];

		
		//if vertically laid, move by x first to within one tile of middle of tunnel
		if(isTunnelVertical) {
			double midpointToRobot = Math.abs(tunnelMidpoint[0] - myX); //this is to check to see if its within 1 tile (case 1 or 2)
			if(midpointToRobot < 35.0) {
				//if midpoint x coordinates are smaller, turn to 270
				if(tunnelMidpoint[0] < myX) {
					turnTo(90); //turn to 90 the other way
				} else {
					turnTo(270);
				}
				//move forward by half a tile
				moveStraight(SQUARE_SIZE/2, true, false);
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				if(tunnelMidpoint[1] > myY) {
					turnTo(0); //turn to 0 
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				} else {
					turnTo(180);
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				}
				
				moveStraight(SENSOR_OFFSET, true, false);
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn to either 270 or 90 depending on tunnel and myX
				if(tunnelMidpoint[0] < myX) { //if midpoint x smaller than myX turn to 270
					turnTo(270);
				} else { //if midpoint x larger than myX turn to 90
					turnTo(90);
				}

				//now do correction with other line
				localizeLine();
				moveStraight(SENSOR_OFFSET, true, false);
				
			} else {
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				//if midpoint x coordinates are smaller, turn to 270
				//travel to the midpoint of the tunnel tile minue one tile and while moving make sure to correct and update odometer
				//watch out cause this method also does a turn to
				if(tunnelMidpoint[0] < myX) {
					turnTo(270); //turn to 270
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					turnTo(90);
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				//turn and move to the y of the tunnel tile + or minus 1
				//so keep the same x but travel to the y
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				if(tunnelMidpoint[1] > myY) {
					turnTo(0); //turn to 0 
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				} else {
					turnTo(180);
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				}
				
				moveStraight(SENSOR_OFFSET, true, false);
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn to either 270 or 90 depending on tunnel and myX
				if(tunnelMidpoint[0] < myX) { //if midpoint x smaller than myX turn to 270
					turnTo(270);
				} else { //if midpoint x larger than myX turn to 90
					turnTo(90);
				}

				//now do correction with other line
				localizeLine();
				moveStraight(SENSOR_OFFSET, true, false);
			}
		} 
		//if horizontally laid out, move by y first
		else { 
			double midpointToRobot = Math.abs(tunnelMidpoint[1] - myY); //this is to check to see if its within 1 tile (case 1 or 2)
			if(midpointToRobot < 30.0) {
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				//if midpoint y coordinates are smaller, turn to 0
				if(tunnelMidpoint[1] < myY) {
					turnTo(0); //turn to 0
				} else {
					turnTo(180);
				}

				//move forward by half a tile
				moveStraight(SQUARE_SIZE/2, true, false);
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				if(tunnelMidpoint[0] < myX) {
					turnTo(270); //turn to 270
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					turnTo(90);
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				moveStraight(SENSOR_OFFSET, true, false);
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn to either 0 or 180 depending on tunnel and myY
				if(tunnelMidpoint[1] < myY) { 
					turnTo(180);
				} else { 
					turnTo(0);
				}

				//now do correction with other line
				localizeLine();
				moveStraight(SENSOR_OFFSET, true, false);
				
			} else {
				System.out.println("entered this");
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				//if midpoint x coordinates are smaller, turn to 180
				//travel to the midpoint of the tunnel tile minue one tile and while moving make sure to correct and update odometer
				//watch out cause this method also does a turn to
				if(tunnelMidpoint[1] < myY) {
					turnTo(180); //turn to 180
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				} else {
					turnTo(0);
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				}

				//turn and move to the x of the tunnel tile + or minus 1
				//so keep the same x but travel to the y
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				if(tunnelMidpoint[0] < myX) {
					turnTo(270); //turn to 0 
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					turnTo(90);
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				moveStraight(SENSOR_OFFSET, true, false);
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn to either 180 or 0 depending on tunnel and myY
				if(tunnelMidpoint[1] < myY) { //if midpoint y smaller than myY turn to 180
					turnTo(180);
				} else { //if midpoint y larger than myY turn to 0
					turnTo(0);
				}

				//now do correction with other line
				localizeLine();
				moveStraight(SENSOR_OFFSET, true, false);
			}
		}
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
		
		setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACC);
		// Define variables
		double odo[] = { 0, 0, 0 }, absAngle = 0, dist = 0, deltaX = 0, deltaY = 0;
		
		// Get odometer readings
		odo = odometer.getXYT();
		
		
		// Get displacement to travel on X and Y axis
		deltaX = x - odo[0];
		deltaY = y - odo[1];

		// Displacement to point (hypothenuse)
		dist = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));
		

		

		// Get absolute angle the robot must be facing
		absAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

		turnTo(absAngle); 
		
		//need a part here that says if its going to end on a line, make it end with the offset already done
		
		setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);
		//if displacement is less than a tile then we will not cross line, so travel normally to coordinate
		if(dist < SQUARE_SIZE - 1 ) {
			System.out.print("2");
			moveStraight(dist, true, false);
		} else  {
			System.out.print("1");
			double oldSampleRight = 0;
			double oldSampleLeft = 0;
			float[] newColorLeft = new float[leftSampleProvider.sampleSize()];
			float[] newColorRight = new float[rightSampleProvider.sampleSize()];

			leftMotor.forward();
			rightMotor.forward();

			int foundLeft = 0; int foundRight = 0;
			
			while(true) {
				// Get color sensor readings
				leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
				rightSampleProvider.fetchSample(newColorRight, 0); 

				// If line detected for left sensor (intensity less than 0.4), only count once by keeping track of last value
				if((newColorLeft[0]) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
					leftMotor.stop(true);
					foundLeft++;
				}
				// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
				if((newColorRight[0]) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
					rightMotor.stop(true);
					foundRight++;
				}
				// Store last color samples
				oldSampleLeft = newColorLeft[0];
				oldSampleRight = newColorRight[0];

				// If line found for both sensors, exit
				if(foundLeft == 1 && foundRight == 1) {
					break;
				}
			}
			//perform correction of odometer here when sensors are on top of lines
			correctOdometer();
			
			odo = odometer.getXYT();
			Display.displayNavigation(odo[0], odo[1], odo[2]);
			
		}	
		
		//if it has not arrived yet to destination, recursively call
		if(!hasArrived(x, y)) {
			travelToWithCorrection(x, y);
		}
	}
	
	
	private static void localizeLine() throws OdometerExceptions {
		System.out.println("\n now doing localizeline");
		
		int foundLeft = 0;
		int foundRight = 0;

		double oldSampleRight = 0;
		double oldSampleLeft = 0;
		float[] newColorLeft = new float[leftSampleProvider.sampleSize()];
		float[] newColorRight = new float[rightSampleProvider.sampleSize()];

		setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);
		
		leftMotor.forward();
		rightMotor.forward();
		
		while(true) {
			// Get color sensor readings
			leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
			rightSampleProvider.fetchSample(newColorRight, 0); 

			// If line detected for left sensor (intensity less than 0.4), only count once by keeping track of last value
			if((newColorLeft[0]) < 0.28 && oldSampleLeft > 0.28 && foundLeft == 0) {
				leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight[0]) < 0.28 && oldSampleRight > 0.28 && foundRight == 0) {
				rightMotor.stop(true);
				foundRight++;
			}
			// Store last color samples
			oldSampleLeft = newColorLeft[0];
			oldSampleRight = newColorRight[0];

			// If line found for both sensors, exit
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}
		
		correctOdometer();
	}
	
    /**
     * This checks if the robot is within a certain distance from a destination
     */
	private static boolean hasArrived(double xd, double yd) throws OdometerExceptions {
		double odometer[] = { 0, 0, 0 };
		odometer = Odometer.getOdometer().getXYT();
		double xf = odometer[0];
		double yf = odometer[1];
		double cErr = Math.hypot(xf - xd, yf - yd);
		return cErr < 6;
	}
	
	//if moving along x axis only allowed to correct x, if along y axis then correct y
	public static void correctOdometer() throws OdometerExceptions {
		Odometer odo = Odometer.getOdometer();
		int theta = (int)(odo.getXYT()[2]);
		double y, x;
		if(theta < 20 && theta >= 0 || theta > 340 && theta <=360) { //heading north correct y
			//robot just stopped at line, get y value and add the offset, then round to nearest coordinate
			y = odo.getXYT()[1];
			y += SENSOR_OFFSET;
			y = y/SQUARE_SIZE;
			int yLine = (int) Math.round(y);
			y = (yLine * SQUARE_SIZE) - SENSOR_OFFSET;
			odo.setY(y);
			odo.setTheta(0.0);
		} else if (theta > 70 && theta < 110) {	//heading east correct x
			// get x value and add offset
			x = odo.getXYT()[0];
			x += SENSOR_OFFSET;
			x = x/SQUARE_SIZE;
			int xLine = (int) Math.round(x);
			x = (xLine * SQUARE_SIZE) - SENSOR_OFFSET;
			odo.setX(x);
			odo.setTheta(90.0);
		} else if (theta > 160 && theta < 200) { //heading south correct y
			y = odo.getXYT()[1];
			y -= SENSOR_OFFSET;
			y = y/SQUARE_SIZE;
			int yLine = (int) Math.round(y);
			y = (yLine * SQUARE_SIZE) + SENSOR_OFFSET;
			odo.setY(y);
			odo.setTheta(180.0);
		} else if (theta > 250 && theta < 290) { //heading is west correct x
			x = odo.getXYT()[0];
			x -= SENSOR_OFFSET;
			x = x/SQUARE_SIZE;
			int xLine = (int) Math.round(x);
			x = (xLine * SQUARE_SIZE) + SENSOR_OFFSET;
			odo.setX(x);
			odo.setTheta(270.0);
		}
	}
	
	
	/**
	 * This method finds out if the tunnel is vertical or horizontal and sets the static variable.
	 * Used by travel_Start_To_Tunnel method to calculate ideal trajectory.
	 */
	public static void findTunnelHeading() {
		int differenceInXCoord = Math.abs(BRR_LL[0] - BRR_UR[0]);
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
     * @param theta : theta angle
     */
	public static void turnTo(double theta) {
		double dTheta = theta - odometer.getXYT()[2];
		if(dTheta < 0) dTheta += 360;
		setSpeedAcceleration(ROTATE_SPEED, ROTATE_ACC);

		if (dTheta > 180) { // turn left
			leftMotor.rotate(-convertAngle(360 - dTheta), true);
			rightMotor.rotate(convertAngle(360 - dTheta), false);
		}
		else { // turn right
			leftMotor.rotate(convertAngle( dTheta), true);
			rightMotor.rotate(-convertAngle( dTheta), false);
		}
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
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
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
