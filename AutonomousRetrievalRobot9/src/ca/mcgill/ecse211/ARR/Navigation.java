package ca.mcgill.ecse211.ARR;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import java.util.ArrayList;


/**
 * This class handles all movement of the robot, where motor initialization is private.
 * Other classes statically call the methods to move. 
 */
public class Navigation {

	//Game parameters
	public static int RedTeam = 9; 		
	public static int GreenTeam = -1; 
	public static int RedCorner = -1; 
	public static int GreenCorner = -1;
	public static int Red_LL_x = -1;
	public static int Red_LL_y = -1;
	public static int Red_UR_x = -1;
	public static int Red_UR_y = -1;
	public static int Green_LL_x = -1; 
	public static int Green_LL_y = -1; 
	public static int Green_UR_x = -1;
	public static int Green_UR_y = -1;
	public static int Island_LL_x = -1;
	public static int Island_LL_y = -1;
	public static int Island_UR_x = -1;
	public static int Island_UR_y = -1;
	public static int TNR_LL_x = 3;
	public static int TNR_LL_y = 3;
	public static int TNR_UR_x = 4;
	public static int TNR_UR_y = 5;
	public static int TNG_LL_x = -1;
	public static int TNG_LL_y = -1;
	public static int TNG_UR_x = -1;
	public static int TNG_UR_y = -1;
	public static int TR_x = 6;
	public static int TR_y = 7;
	public static int TG_x = -1;
	public static int TG_y = -1;
	
	//Speed and acceleration
	private static final int ROTATE_SPEED = 150;
	private static final int ROTATE_ACCEL = 1500;
	private static final int NAV_WITH_CORR_SPEED = 190;
	private static final int NAV_WITH_CORR_ACCEL = 1500;
	private static final int TUNNEL_SPPED = 130;
	private static final int TUNNEL_ACCEL = 1000;
	
	//distance measurements
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 12.85;
	public static final double SQUARE_SIZE = 30.48;
	public static final double SENSOR_OFFSET = 4.9;
	public static final double AVOID_RING_SET_DISTANCE = 10.0;

	//Association variables
	private static Odometer odometer;
	private static boolean isNavigating;

	//EV3 objects
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static SampleProvider leftSampleProvider;
	private static SampleProvider rightSampleProvider;
	
	//Class variables
	private static boolean isTunnelVertical;

	
	/**
	 * Class constructor
	 * @param leftLight : takes in SampleProvider for left light sensor
	 * @param rightLight : takes in SampleProvider for right light sensor
	 * @param odo : takes in odometer, instead of calling Odometer to get, this ensures odometer instantiated first
	 * @throws OdometerExceptions
	 */
	public Navigation(SampleProvider leftLight, SampleProvider rightLight, Odometer odo, 
			EV3LargeRegulatedMotor lMotor,EV3LargeRegulatedMotor rMotor) throws OdometerExceptions {
		odometer = odo;
		leftSampleProvider = leftLight;
		rightSampleProvider = rightLight;
		leftMotor = lMotor;
		rightMotor = rMotor;
	}
	
	
	
	/**
	 * Method asssumes robot done localizing, takes coordinates and calculates the trajectory 
	 * to the tunnel based on four cases. If the tunnel is placed vertically, it travels first horizontally
	 * (along the x axis) then vertically (along y axis) so that it may re-localize just before entering
	 * the tunnel, the robot will be accurately within one tile of the tunnel to travel without a hitch. It
	 * takes the robot to and through the tunnel.
	 * 
	 * Cases:
	 * 1) tunnel has vertical orientation and its midpoint x value is within 1 tile of robot x value
	 * 2) tunnel has vertical orientation and its midpoint x value is greater than 1 tile of robot x value
	 * 3) tunnel has horizontal orientation and its midpoint y value is within 1 tile of robot y value
	 * 4) tunnel has horizontal orientation and its midpoint y value is greater than 1 tile of robot y value
	 * 
	 * In order to achieve localization before entering tunnel for cases 1 and 3, we must move away from the 
	 * tunnel midpoint so as to be able to turn to it and catch the second line.
	 */
	public static void travelStartToTunnel() throws OdometerExceptions {

		//calculate start midpoint and tunnel orientation
		findTunnelHeading();
		double[] tunnelMidpoint = findTunnelMidpoint(true);
		
		double myX = odometer.getXYT()[0];
		double myY = odometer.getXYT()[1];

		//if tunnel vertical, move by x first to within one tile of tunnelMidpoint x value
		if(isTunnelVertical) {
			
			//CASE 1
			if(isTunnelWithinOneTile(tunnelMidpoint[0], myX)) {
				
				//turn away from tunnel midpoint
				if(tunnelMidpoint[0] < myX)	turnTo(90); 
				else turnTo(270);
				
				//move forward by half a tile
				moveStraight(SQUARE_SIZE/2, true, false);
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//now travel along y axis to the y coordinate *one before* the tunnel midpoint
				//travel north if tunnel y value is above on grid, else south
				if(tunnelMidpoint[1] > myY) {
					turnTo(0); 
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				} else {
					turnTo(180);
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				}
				
				//move forward by sensor to center offset
//				moveStraight(SENSOR_OFFSET, true, false);
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn to travel remaining x distance to center of tunnel
				if(tunnelMidpoint[0] < myX) turnTo(270);
				else turnTo(90);

				//now correct correct with second line essentially localizing
				localizeLine();
				moveStraight(SENSOR_OFFSET, true, false);
				
			}
			
			//CASE 2
			else {
				
				//turn towards x value of tunnel midpoint and travel to the point before it in one tile
				if(tunnelMidpoint[0] < myX) {
					turnTo(270); //turn to 270
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					turnTo(90);
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//now travel along y axis to the y coordinate *one before* the tunnel midpoint
				//travel north if tunnel y value is above on grid, else south
				if(tunnelMidpoint[1] > myY) {
					turnTo(0); //turn to 0 
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				} else {
					turnTo(180);
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				}
				
				//move by sensor to center offset
//				moveStraight(SENSOR_OFFSET, true, false);
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn towards tunnel x value
				if(tunnelMidpoint[0] < myX) turnTo(270);
				else turnTo(90);

				//now do correction essentially localizing
				localizeLine();
				moveStraight(SENSOR_OFFSET, true, false);
			}
		} 
		//if tunnel horizontal, move by y first to within one tile of tunnelMidpoint y value
		else { 
			
			//CASE 3
			if(isTunnelWithinOneTile(tunnelMidpoint[1], myY)) {
				
				//turn away from tunnel midpoint y value
				if(tunnelMidpoint[1] < myY) turnTo(0); 
				else turnTo(180);

				//move forward by half a tile in y direction
				moveStraight(SQUARE_SIZE/2, true, false);
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//then move in x direction to x value *just before* tunnel midpoint
				if(tunnelMidpoint[0] < myX) {
					turnTo(270);
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					turnTo(90);
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				//move by sensor to center offset
//				moveStraight(SENSOR_OFFSET, true, false);
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn towards the y of the tunnel midpoint
				if(tunnelMidpoint[1] < myY) turnTo(180);
				else turnTo(0);
				
				//now do correction with other line essentially localizing
				localizeLine();
				moveStraight(SENSOR_OFFSET, true, false);
				
				
			} 
			
			//CASE 4
			else {
				
				//turn towards y value of tunnel midpoint and travel to the point before it in one tile
				if(tunnelMidpoint[1] < myY) {
					turnTo(180); 
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				} else {
					turnTo(0);
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				}

				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//move towards x value *just before* tunnel midpoint 
				if(tunnelMidpoint[0] < myX) {
					turnTo(270); 
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					turnTo(90);
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				//move by sensor to center offset
//				moveStraight(SENSOR_OFFSET, true, false);
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn towards tunnel y value
				if(tunnelMidpoint[1] < myY) turnTo(180);
				else  turnTo(0);
				
				//now do correction with other line essentially localizing
				localizeLine();
				moveStraight(SENSOR_OFFSET, true, false);
			}
		}
		
		//for all cases tunnel now is either to left or right
		//turn to midpoint of tunnel and go through it
		myX = odometer.getXYT()[0];
		myY = odometer.getXYT()[1];
		double absAngle = Math.toDegrees(Math.atan2((tunnelMidpoint[0] - myX), (tunnelMidpoint[1] - myY)));
		turnTo(absAngle); 
		
		setSpeedAcceleration(TUNNEL_SPPED, TUNNEL_ACCEL);
		moveStraight(SQUARE_SIZE * 3.6, true, false);
	}
	
	
	/**
	 * This method takes the robot from just having gone outside the tunnel to the ring set
	 * so that it is in position to start circling the set detecting the ring colors. There
	 * are two cases of traversing. Since the robot ends in middle of tunnel tile and ring 
	 * sets are placed on intersections, the set will either be to the left or right of tunnel.
	 * If to the left we move to the closest tile (bottom right of four surrounding tiles relative
	 * to the robot heading and placement of ring set), if to the right we move to bottom right of 
	 * four surrounding tiles. It then aligns the robot to start traversing the ring set in a 
	 * clockwise direction.
	 * @throws OdometerExceptions
	 */
	public static void travelTunnelToRingSet() throws OdometerExceptions {
		
		String heading = getCurrentHeading();
		double[] tunnelEndMidpoint = findTunnelMidpoint(false);
		
		//CASE 1: 
		if(isRingSetToLeft()) {
			//get bottom right and bottom left corner tiles
			double[] bottomRightTile = new double[2];
			double[] bottomLeftTile = new double[2];;
			double[] tiles = getClosestTiles(true);
			bottomRightTile[0] = tiles[0];
			bottomRightTile[1] = tiles[1];
			bottomLeftTile[0] = tiles[2];
			bottomLeftTile[1] = tiles[3];
			
			
			//travel first in direction you are currently traveling in
			//if traveling north/south, move to y of bottomRightTile
			if(heading.equalsIgnoreCase("north") || heading.equalsIgnoreCase("south")) 
				travelToWithCorrection(tunnelEndMidpoint[0], bottomRightTile[1]);
			//if traveling east/west, move to x of bottomRightTile
			else
				travelToWithCorrection(bottomRightTile[0], tunnelEndMidpoint[1]);
			
			//now move other axis to reach bottomRightTile
			travelToWithCorrection(bottomRightTile[0], bottomRightTile[1]);
			
			
			//ring set should be to the north/east of the robot
			//move to line in front, move forward offset amount, turn right, move to next line and ready for detection
			turnToCoord(bottomLeftTile[0], bottomLeftTile[1]);
			localizeLine();
			moveStraight(AVOID_RING_SET_DISTANCE, true, false);
			turnRobot(90, true, false);
			localizeLine();
			
		} 
		
		//CASE 2:
		else {
			//get bottom left and upper left corner tiles
			double[] bottomLeftTile = new double[2];
			double[] upperLeftTile = new double[2];;
			double[] tiles = getClosestTiles(true);
			bottomLeftTile[0] = tiles[0];
			bottomLeftTile[1] = tiles[1];
			upperLeftTile[0] = tiles[2];
			upperLeftTile[1] = tiles[3];
			
			//travel first in direction you are currently traveling in
			//if traveling north/south, move to y of bottomRightTile
			if(heading.equalsIgnoreCase("north") || heading.equalsIgnoreCase("south")) 
				travelToWithCorrection(tunnelEndMidpoint[0], bottomLeftTile[1]);
			//if traveling east/west, move to x of bottomRightTile
			else
				travelToWithCorrection(bottomLeftTile[0], tunnelEndMidpoint[1]);
			
			//now move other axis to reach bottomRightTile
			travelToWithCorrection(bottomLeftTile[0], bottomLeftTile[1]);
			
			//ring set should be to the north/west of the robot
			//move to line connecting bottom left to upper right, move forward offset amount, turn right, move to next line and ready for detection
			turnToCoord(upperLeftTile[0], upperLeftTile[1]);
			localizeLine();
			moveStraight(AVOID_RING_SET_DISTANCE, true, false);
			turnRobot(90, true, false);
			localizeLine();
		}
	}
	
	
	//assumes we end on top of a line and color sensor is to right of robot facing the ring set
	public static void detectRings() {
		//it detects first one, turns around clockwise and does it again
		//saves the information in the mean time i.e. height and color of ring detected
	}
	
	
	
	/** 
	 * This method takes in the side the ring set is on relative to the heading of the robot
	 * upon exiting the tunnel. Calculates the center coordinates of the tiles closest to the
	 * robot in order to traverse properly to a position where it can begin to search and retrieve.
	 * @param ringSetIsLeft : whether the ring set is to the left or right of the robot
	 * @return : a double[] of 4 values where the first two are for the first closest tile coordinates
	 * 			 and the second two values are second closest.
	 */
	public static double[] getClosestTiles(boolean ringSetIsLeft) {
		String heading = getCurrentHeading();
		double[] tileCoord = new double[4];
		
		//if we are red team use red team ring set, else green ring set
		if(RedTeam == 9) { 	
			tileCoord[0] = TR_x * SQUARE_SIZE;
			tileCoord[1] = TR_y * SQUARE_SIZE;
		} else {
			tileCoord[0] = TG_x * SQUARE_SIZE;
			tileCoord[1] = TG_y * SQUARE_SIZE;
		}
		
		if(heading.equalsIgnoreCase("north")) { 						//heading north
			if(ringSetIsLeft) {		// +x -y
				tileCoord[0] +=  (SQUARE_SIZE/2);
				tileCoord[1] -=  (SQUARE_SIZE/2);
				tileCoord[2] -=  (SQUARE_SIZE/2);
				tileCoord[3] -=  (SQUARE_SIZE/2);
			} else {				// -x -y
				tileCoord[0] -=  (SQUARE_SIZE/2);
				tileCoord[1] -=  (SQUARE_SIZE/2);
				tileCoord[2] -=  (SQUARE_SIZE/2);
				tileCoord[3] +=  (SQUARE_SIZE/2);
			}
		} else if (heading.equalsIgnoreCase("east")) {					//heading east
			if(ringSetIsLeft) {		// -x -y
				tileCoord[0] -=  (SQUARE_SIZE/2);
				tileCoord[1] -=  (SQUARE_SIZE/2);
				tileCoord[2] -=  (SQUARE_SIZE/2);
				tileCoord[3] +=  (SQUARE_SIZE/2);
			} else {				// -x +y
				tileCoord[0] -=  (SQUARE_SIZE/2);
				tileCoord[1] +=  (SQUARE_SIZE/2);
				tileCoord[2] +=  (SQUARE_SIZE/2);
				tileCoord[3] +=  (SQUARE_SIZE/2);
			}
		} else if (heading.equalsIgnoreCase("south")) { 				//heading south
			if(ringSetIsLeft) {		// -x +y
				tileCoord[0] -=  (SQUARE_SIZE/2);
				tileCoord[1] +=  (SQUARE_SIZE/2);
				tileCoord[2] +=  (SQUARE_SIZE/2);
				tileCoord[3] +=  (SQUARE_SIZE/2);
			} else {				// +x +y
				tileCoord[0] +=  (SQUARE_SIZE/2);
				tileCoord[1] +=  (SQUARE_SIZE/2);
				tileCoord[2] +=  (SQUARE_SIZE/2);
				tileCoord[3] -=  (SQUARE_SIZE/2);
			}
		} else if (heading.equalsIgnoreCase("west")) { 					//heading west
			if(ringSetIsLeft) {		// +x +y
				tileCoord[0] +=  (SQUARE_SIZE/2);
				tileCoord[1] +=  (SQUARE_SIZE/2);
				tileCoord[2] +=  (SQUARE_SIZE/2);
				tileCoord[3] -=  (SQUARE_SIZE/2);
			} else {				// +x -y
				tileCoord[0] +=  (SQUARE_SIZE/2);
				tileCoord[1] -=  (SQUARE_SIZE/2);
				tileCoord[2] -=  (SQUARE_SIZE/2);
				tileCoord[3] -=  (SQUARE_SIZE/2);
			}
		}
		return tileCoord;
	}
	
	

	/**
	 * This method uses the game parameters and the robots current heading to
	 * calculate whether the ring set is to the left or right of the robot
	 * upon exiting the tunnel
	 * @return : true if ring set is to the left of the robot, false if to the right
	 */
	public static boolean isRingSetToLeft() {
		//get end tunnel end midpoint, current heading of robot, ringset coordinates
		double[] tunnelEndMidpoint = new double[2];
		int ringSetX, ringSetY;
		boolean isToLeft = false;
		
		String heading = getCurrentHeading();
		tunnelEndMidpoint = findTunnelMidpoint(false);
		
		//if we are red team use red team ring set, else green ring set
		if(RedTeam == 9) { 	
			ringSetX = TR_x;
			ringSetY = TR_y;
		} else {
			ringSetX = TG_x;
			ringSetY = TG_y;
		}
		
		//check if ringset is relatively to the left of robot
		if(heading.equalsIgnoreCase("north")) { 						//heading north, if ringset x smaller than tunnel x it's left
			if(ringSetX < ((int)tunnelEndMidpoint[0]/SQUARE_SIZE)) 
				isToLeft = true;
		} else if (heading.equalsIgnoreCase("east")) {					//heading east, if ringset y is larger than tunnel y it's left
			if(ringSetY > ((int)tunnelEndMidpoint[1]/SQUARE_SIZE)) 
				isToLeft = true;
		} else if (heading.equalsIgnoreCase("south")) { 				//heading south, if ringset x is larger than tunnel x it's left
			if(ringSetX > ((int)tunnelEndMidpoint[0]/SQUARE_SIZE)) 
				isToLeft = true;
		} else if (heading.equalsIgnoreCase("west")) { 					//heading west, if ringset y is smaller than tunnel y it's left
			if(ringSetY < ((int)tunnelEndMidpoint[1]/SQUARE_SIZE)) 
				isToLeft = true;
		}
		
		return isToLeft;
	}
	
	
	/**
	 * This method gets the current theta heading from the odometer and returns
	 * the heading of the robot as a string.
	 * @return : returns a String with either "north", "south", "east", "west"
	 */
	public static String getCurrentHeading() {
		double theta = odometer.getXYT()[2];
		String heading = "error";
		if(theta < 45 && theta >= 0 || theta >= 315 && theta <= 360) { 	//heading north
			heading = "north";
		} else if (theta >= 45 && theta < 135) {						//heading east
			heading = "east";
		} else if (theta >= 135 && theta < 225) { 						//heading south
			heading = "south";
		} else if (theta >= 225 && theta < 315) { 						//heading west
			heading = "west";
		}
		return heading;
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
		double odo[] = { 0, 0, 0 }, absAngle = 0, dist = 0, deltaX = 0, deltaY = 0;
		
		//if it has not arrived move again
		while(!hasArrived(x, y)) {
			
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
			
			
			setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);
			
			//we either move to a line or to half a tile, if its half move straight without correction
			if(dist < SQUARE_SIZE - 4 ) {
				moveStraight(dist, true, false);
			} else  {
				
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
				moveStraight(SENSOR_OFFSET, true, false);
				
				
				//display the corrected values 
				odo = odometer.getXYT();
				Display.displayNavigation(odo[0], odo[1], odo[2]);
				
			}	
		}
	}
	
	
	/**
	 * This method assumes the robot is in the middle of a tile and is facing 
	 * one of the sides rather than the corners. It moves the robot forward until
	 * it detects the line in front of it and stops so that both sensors are on 
	 * top of the line. It then calls correctOdometer() and ends.
	 * @throws OdometerExceptions
	 */
	private static void localizeLine() throws OdometerExceptions {
		
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
	 * Checks if the robot is within a certain distance from a destination.
	 * @param xd : x coordinate desitination
	 * @param yd : y coordinate destination
	 * @return : true if its within the distance
	 * @throws OdometerExceptions
	 */
	private static boolean hasArrived(double xd, double yd) throws OdometerExceptions {
		double odometer[] = { 0, 0, 0 };
		odometer = Odometer.getOdometer().getXYT();
		double xf = odometer[0];
		double yf = odometer[1];
		double cErr = Math.hypot(xf - xd, yf - yd);
		return cErr < 7;
	}
	
	
	/**
	 * This method is used when the robot is traveling either horizontally or vertically
	 * and its light sensors have stopped on top of the lines. It retrieved the direction 
	 * and location of robot and corrects odometer values. If traveling x then only corrects x,
	 * always corrects theta. Estimates heading and position based on odometer readings so assumes
	 * readings are not off by a certain amount.
	 * @throws OdometerExceptions
	 */
	public static void correctOdometer() throws OdometerExceptions {
		double[] data = new double[3];
		data = odometer.getXYT();
		double x = data[0], y = data[1];
		String heading = getCurrentHeading();
		//robot stopped with sensors on line, get x/y and adjust for offset, then round to nearest coordinate
		if(heading.equalsIgnoreCase("north")) { 						//heading north correct y
			y += SENSOR_OFFSET;
			y = y/SQUARE_SIZE;
			int yLine = (int) Math.round(y);
			y = (yLine * SQUARE_SIZE) - SENSOR_OFFSET;
			odometer.setY(y);
			odometer.setTheta(0.0);
		} else if (heading.equalsIgnoreCase("east")) {					//heading east correct x
			x += SENSOR_OFFSET;
			x = x/SQUARE_SIZE;
			int xLine = (int) Math.round(x);
			x = (xLine * SQUARE_SIZE) - SENSOR_OFFSET;
			odometer.setX(x);
			odometer.setTheta(90.0);
		} else if (heading.equalsIgnoreCase("south")) { 				//heading south correct y
			y -= SENSOR_OFFSET;
			y = y/SQUARE_SIZE;
			int yLine = (int) Math.round(y);
			y = (yLine * SQUARE_SIZE) + SENSOR_OFFSET;
			odometer.setY(y);
			odometer.setTheta(180.0);
		} else if (heading.equalsIgnoreCase("west")) { 					//heading is west correct x
			x -= SENSOR_OFFSET;
			x = x/SQUARE_SIZE;
			int xLine = (int) Math.round(x);
			x = (xLine * SQUARE_SIZE) + SENSOR_OFFSET;
			odometer.setX(x);
			odometer.setTheta(270.0);
		}
	}
	
	
	/**
	 * This method finds out if the tunnel is vertical or horizontal by observing
	 * the absolute difference between the tunnels lower left and upper right x values 
	 * and sets the class variable isTunnelVertical accordingly.
	 */
	public static void findTunnelHeading() {
		isTunnelVertical = (Math.abs(TNR_LL_x - TNR_UR_x) == 2) ? false : true;
	}
	
	/**
	 * This method is passed in either x values or y values (only) of both the tunnel and robot values;
	 * must pass in either both x values or both y values. Checks to see if the absolute distance is within 
	 * one tile.
	 * @param tunnelValue : x or y value of tunnel midpoint
	 * @param robotValue : x or y value of robot current position
	 * @return : true if within one tile
	 */
	public static boolean isTunnelWithinOneTile(double tunnelValue, double robotValue) {
		if (Math.abs(tunnelValue - robotValue) < SQUARE_SIZE + 4) // 4 added incase odometry error
			return true;
		else 
			return false;
	}
	
	
	/**
	 * This method finds and returns the midpoint of the start or end of the robot's team's tunnel.
	 * Assumes the tunnel coordinates given i.e. BRR_LL and BRR_UR for red team, are not in actual 
	 * distance values but coordinate values.
	 * @param start : boolean check, if true then method returns exit midpoint of tunnel relative to robot start and team
	 * @return midpoint: double[] coordinates of tunnel midpoint in actual distance measurement
	 */
	public static double[] findTunnelMidpoint(boolean start) {
		findTunnelHeading(); //incase not called
		
		//check if we are red team, then use tunnel coordinates of red team
		int[] tunnelLL = {0, 0}, tunnelUR = {0, 0};
		if(RedTeam == 9) { 	//if we are red team set the arrays to BRR_LL and BRR_UR 
			tunnelLL[0] = TNR_LL_x;
			tunnelLL[1] = TNR_LL_y;
			tunnelUR[0] = TNR_UR_x;
			tunnelUR[1] = TNR_UR_y;
		} else {
			tunnelLL[0] = TNG_LL_x;
			tunnelLL[1] = TNG_LL_y;
			tunnelUR[0] = TNG_UR_x;
			tunnelUR[1] = TNG_UR_y;
		}

		double[] midpoint1 = {0,0}, midpoint2 = {0,0};
		
		//calculate midpoints of both ends of tunnel
		if(isTunnelVertical) {
			midpoint1[1] = (tunnelLL[1]) * SQUARE_SIZE;
			midpoint1[0] = (tunnelLL[0] + 0.5) * SQUARE_SIZE;
			midpoint2[1] = (tunnelUR[1]) * SQUARE_SIZE;
			midpoint2[0] = (tunnelUR[0] - 0.5) * SQUARE_SIZE;
		} else {
			midpoint1[0] = (tunnelLL[0]) * SQUARE_SIZE;
			midpoint1[1] = (tunnelLL[1] + 0.5) * SQUARE_SIZE;
			midpoint2[0] = (tunnelUR[0]) * SQUARE_SIZE;
			midpoint2[1] = (tunnelUR[1] - 0.5) * SQUARE_SIZE;
		}
		
		//whichever is closer is the midpoint we want
		double myX = odometer.getXYT()[0], myY = odometer.getXYT()[1];
		double dX1 = midpoint1[0] - myX; 
		double dY1 = midpoint1[1] - myY; 
		double dX2 = midpoint2[0] - myX; 
		double dY2 = midpoint2[1] - myY; 

		// displacement to points
		double dist1 = Math.hypot(Math.abs(dX1), Math.abs(dY1));
		double dist2 = Math.hypot(Math.abs(dX2), Math.abs(dY2));
		
		//return the one with smaller distance to
		return (dist1 > dist2) ? midpoint2 : midpoint1;
	}
	
	
	public static void testFiniteDifferencesLightSensor() {
		int window = 3;
		ArrayList<Float> currentSamplesLeft = new ArrayList<Float>();
		ArrayList<Float> currentSamplesRight = new ArrayList<Float>();
		float[] newColorLeft = new float[leftSampleProvider.sampleSize()];
		float[] newColorRight = new float[rightSampleProvider.sampleSize()];
		leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
		rightSampleProvider.fetchSample(newColorRight, 0); 
		currentSamplesLeft.add(newColorLeft[0]);
		currentSamplesRight.add(newColorRight[0]);
		leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
		rightSampleProvider.fetchSample(newColorRight, 0); 
		currentSamplesLeft.add(newColorLeft[0]);
		currentSamplesRight.add(newColorRight[0]);
		leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
		rightSampleProvider.fetchSample(newColorRight, 0); 
		currentSamplesLeft.add(newColorLeft[0]);
		currentSamplesRight.add(newColorRight[0]);
		

		float oldAverageLeft = findInitialAverage(currentSamplesLeft);
		float sample_k_n_left = currentSamplesLeft.remove(0);
		float oldAverageRight = findInitialAverage(currentSamplesRight); 
		float sample_k_n_right = currentSamplesRight.remove(0);
		
		leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
		rightSampleProvider.fetchSample(newColorRight, 0); 
		currentSamplesLeft.add(newColorLeft[0]);
		currentSamplesRight.add(newColorRight[0]);
		
		float newAverageLeft = oldAverageLeft + (1/window)*(newColorLeft[0]-sample_k_n_left);
		float newAverageRight = oldAverageRight + (1/window)*(newColorLeft[0]-sample_k_n_right);
		


		leftMotor.forward();
		rightMotor.forward();
		
		int foundLeft = 0; int foundRight = 0;
		
		while(true) {
			// Get color sensor readings
			leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
			rightSampleProvider.fetchSample(newColorRight, 0); 
			sample_k_n_left = currentSamplesLeft.remove(0);
			sample_k_n_right = currentSamplesRight.remove(0);
			currentSamplesLeft.add(newColorLeft[0]);
			currentSamplesRight.add(newColorRight[0]);
			newAverageLeft = oldAverageLeft + (1/window)*(newColorLeft[0]-sample_k_n_left);
			newAverageRight = oldAverageRight + (1/window)*(newColorLeft[0]-sample_k_n_right);
			
			//if f(xi+1) - f(x) <= 0 reached peak of line
			//	YOU CAN TRY TO CHANGE 0 to -0.05 or some shit to say the change must be strong
			if(((newAverageLeft - oldAverageLeft) <= 0) && foundLeft == 0) {
				leftMotor.stop(true);
				foundLeft++;
			}
			if(((newAverageRight - oldAverageRight) <= 0)  && foundRight == 0) {
				rightMotor.stop(true);
				foundRight++;
			}
			
			// Store last averages
			oldAverageLeft = newAverageLeft;
			oldAverageRight = newAverageRight;

			// If line found for both sensors, exit
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}
		
	}
	
	public static float findInitialAverage(ArrayList<Float> samples) {
		float sum = 0;
		for(Float sample : samples)
		    sum += sample;
		return sum/samples.size();
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
		setSpeedAcceleration(ROTATE_SPEED, ROTATE_ACCEL);

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
	 * This method takes two coordinates and turns the robot to face those coordinates based
	 * on where the robot currently is.
	 * @param x
	 * @param y
	 */
	public static void turnToCoord(double x, double y) {
		double x0 = odometer.getXYT()[0];
		double y0 = odometer.getXYT()[1];
		double theta0 = odometer.getXYT()[2];
		double xVector = x - x0;
		double yVector = y - y0;
		double thetaVector = Math.toDegrees(Math.atan2(xVector, yVector));
		turnTo(thetaVector - theta0);
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
	
	/**
	 * This methods stops both of the motors and returns immediately
	 */
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

	/**
	 * This method converts a distance the robot would like to travel to the amount
	 * of rotations in degrees the motor has to turn.
	 * @param distance : distance to travel
	 * @return : rotations in degrees
	 */
	public static int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

	/**
	 * Converts the angle the robot would like to turn, to the rotations in degrees
	 * the motor has to turn.
	 * @param angle : angle to turn
	 * @return : rotations in degrees
	 */
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
