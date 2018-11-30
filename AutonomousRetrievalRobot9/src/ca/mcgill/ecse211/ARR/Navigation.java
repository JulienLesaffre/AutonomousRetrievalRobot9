package ca.mcgill.ecse211.ARR;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


/**
 * This class handles all movement of the robot, where motor initialization is private.
 * Other classes statically call the methods to move. 
 */
public class Navigation {

	//Game parameters
	public static int RedTeam = -1; 		
	public static int GreenTeam = 9; 
	public static int RedCorner = -1; 
	public static int GreenCorner = 0;
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
	public static int TNG_LL_x = 3;
	public static int TNG_LL_y = 0;
	public static int TNG_UR_x = 5;
	public static int TNG_UR_y = 1;
	public static int TR_x = 6;
	public static int TR_y = 6;
	public static int TG_x = -1;
	public static int TG_y = -1;
	public static int field_X_Max = 15;
	public static int field_X_Min = 0;
	public static int field_Y_Max = 9;
	public static int field_Y_Min = 0;
	
	//Speed and acceleration
	public static final int ROTATE_SPEED_SLOW = 170;
	public static final int ROTATE_ACCEL_SLOW = 1300;
	public static final int ROTATE_SPEED_FAST = 230;
	public static final int ROTATE_ACCEL_FAST = 1400;
	private static final int NAV_WITH_CORR_SPEED = 265;
	private static final int NAV_WITH_CORR_ACCEL = 1700;
	private static final int LEFT_MOTOR_SPEED_OFFSET = 6;
	
	
	//distance and angle measurements
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 12.80;
	public static final double SQUARE_SIZE = 30.48;
	public static final double SENSOR_OFFSET = 6.1;
	private static final double TUNNEL_DISTANCE_PASS = SQUARE_SIZE * 2.75;
	private static final double TUNNEL_DISTANCE_PASS_ONE_TILE = SQUARE_SIZE * 1.75;
	
	
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
	private static boolean isOneTile = false;
	public static double[] targetCoordinate = new double[2];
	
	//sensor values
	public static final double LIGHT_THRESHOLD = 0.26;



	
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
	public static void travelToTunnel(boolean toStartOfTunnel) throws OdometerExceptions {

		//calculate start midpoint and tunnel orientation
		findTunnelHeading();
		double[] tunnelMidpoint = findTunnelMidpoint(toStartOfTunnel);
		
		
		double myX = odometer.getXYT()[0];
		double myY = odometer.getXYT()[1];

		setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);

		//if tunnel vertical, move by x first to within one tile of tunnelMidpoint x value
		if(isTunnelVertical) {
			
			//CASE 1
			if(isTunnelWithinOneTile(tunnelMidpoint[0], myX)) {
				
				//turn away from tunnel midpoint
				if(tunnelMidpoint[0] < myX)	turnTo(90, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false); 
				else turnTo(270, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
				
				//move forward by half a tile
				setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);
				moveStraight(SQUARE_SIZE/2, true, false);
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//now travel along y axis to the y coordinate *one before* the tunnel midpoint
				//travel north if tunnel y value is above on grid, else south
				if(tunnelMidpoint[1] > myY) {
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				} else {
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				}
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn to travel remaining x distance to center of tunnel
				if(tunnelMidpoint[0] < myX) turnTo(270, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
				else turnTo(90, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
			}
			
			//CASE 2
			else {
				
				System.out.println("good");
				
				//turn towards x value of tunnel midpoint and travel to the point before it in one tile
				if(tunnelMidpoint[0] < myX) {
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//now travel along y axis to the y coordinate *one before* the tunnel midpoint
				//travel north if tunnel y value is above on grid, else south
				if(tunnelMidpoint[1] > myY) {
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				} else {
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				}
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn towards tunnel x value
				if(tunnelMidpoint[0] < myX) turnTo(270, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
				else turnTo(90, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);

			}
		} 
		//if tunnel horizontal, move by y first to within one tile of tunnelMidpoint y value
		else { 
			
			//CASE 3
			if(isTunnelWithinOneTile(tunnelMidpoint[1], myY)) {
				
				//turn away from tunnel midpoint y value
				if(tunnelMidpoint[1] < myY) turnTo(0, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false); 
				else turnTo(180, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);

				//move forward by half a tile in y direction
				moveStraight(SQUARE_SIZE/2, true, false);
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//then move in x direction to x value *just before* tunnel midpoint
				if(tunnelMidpoint[0] < myX) {
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn towards the y of the tunnel midpoint
				if(tunnelMidpoint[1] < myY) turnTo(180, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
				else turnTo(0, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
				
			} 
			//CASE 4
			else {
				
				//turn towards y value of tunnel midpoint and travel to the point before it in one tile
				if(tunnelMidpoint[1] < myY) {
					travelToWithCorrection(myX, tunnelMidpoint[1]+SQUARE_SIZE);
				} else {
					travelToWithCorrection(myX, tunnelMidpoint[1]-SQUARE_SIZE);
				}

				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//move towards x value *just before* tunnel midpoint 
				if(tunnelMidpoint[0] < myX) {
					travelToWithCorrection(tunnelMidpoint[0]+SQUARE_SIZE, myY);
				} else {
					travelToWithCorrection(tunnelMidpoint[0]-SQUARE_SIZE, myY);
				}
				
				
				myX = odometer.getXYT()[0];
				myY = odometer.getXYT()[1];
				
				//turn towards tunnel y value
				if(tunnelMidpoint[1] < myY) turnTo(180, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
				else  turnTo(0, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
			}
		}
		
		//now correct with second line essentially localizing
		findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
		
		//move to tunnel center
		moveStraight(SQUARE_SIZE/2, true, false);
		
		//for all cases tunnel now is either to left or right
		//turn to midpoint of tunnel and go through it
		myX = odometer.getXYT()[0];
		myY = odometer.getXYT()[1];
		double absAngle = Math.toDegrees(Math.atan2((tunnelMidpoint[0] - myX), (tunnelMidpoint[1] - myY)));
		turnTo(absAngle, ROTATE_SPEED_SLOW, ROTATE_ACCEL_SLOW, true); 
		
		//first correct on tunnel entrance lines
		findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);

		//with no correction the robot steers towards left, increase speed for duration of tunnel
		//in order to avoid collision
		leftMotor.setSpeed(NAV_WITH_CORR_SPEED + LEFT_MOTOR_SPEED_OFFSET);		
		rightMotor.setSpeed(NAV_WITH_CORR_SPEED);
		leftMotor.setAcceleration(3000);
		rightMotor.setAcceleration(3000);
		
		//now move through tunnel depending on tunnel size
		if(isOneTile) {
			moveStraight(TUNNEL_DISTANCE_PASS_ONE_TILE, true, false);
		} else {
			moveStraight(TUNNEL_DISTANCE_PASS, true, false);
		}

		//correct for future movement
		setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);
	}
	
	


	/**
	 * This method takes the robot from just having gone outside the tunnel to the ring set.
	 * If the ringset is to the left of the robot upon exiting tunnel, it travels to the tile intersection
	 * to the right of it, else if the ringset is to the right it travels to the tile intersection to the 
	 * left of it.
	 * @throws OdometerExceptions
	 */
	public static void travelTunnelToRingSet() throws OdometerExceptions {
		
		String heading = getCurrentHeading();

		//if ring is to left, go to tile intersection to the right of it relative to robot position
		//else if ring is right, go to tile intersection below it relative to robot position
		boolean isLeft = isRingSetToLeft();
		
		//retrieve the coordinates the robot should travel to
		if(isLeft)
			targetCoordinate = getTargetCoordinate(true);
		else 
			targetCoordinate = getTargetCoordinate(false);
		

		//if the ringset is to the left, there can only be a wall on the right side of robot,
		//check if there is a wall, if so move to left upon exiting tunnel
		if(isLeft && !isThereAWall(false)) {
			turnRobot(80, true, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
			turnRobot(80, false, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
		} 
		//else if ringset is to left, turn robot to right to account for case where rinset is directly on line to the left upon exiting tunnel
		else if (isLeft) {
			turnRobot(80, false, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
			turnRobot(80, true, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
		} 
		//if the ringset is to the right, there can only be a wall on the left side of robot,
		//check if there is a wall, if so move to the right upon exiting tunnel
		else if(!isLeft && !isThereAWall(true)) {
			turnRobot(80, false, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
			turnRobot(80, true, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
		} 
		//else if ringset is to right, turn robot to left to account for case where rinset is directly on line to the right upon exiting tunnel
		else if(!isLeft) {
			turnRobot(80, true, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
			turnRobot(80, false, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
		}
			
		double myX = odometer.getXYT()[0];
		double myY = odometer.getXYT()[1];
		
		//travel first in direction you are currently traveling in
		//if traveling north/south, move to y of target coordinate
		if(heading.equalsIgnoreCase("north") || heading.equalsIgnoreCase("south")) 
			travelToWithCorrection(myX, targetCoordinate[1]);
		//if traveling east/west, move to x of target coordinate
		else
			travelToWithCorrection(targetCoordinate[0], myY);

		//now move other axis to reach target coordinate
		travelToWithCorrection(targetCoordinate[0], targetCoordinate[1]);

		//announce arrival
		RingController.makeSound(3);
	}
	
	
	/**
	 * This method takes in the current position of the robot, and checks to see if there is a wall within 1 tile
	 * to the left or right of the robot. It is used for cases where the tunnel is against the wall, upon exiting the tunnel,
	 * so that it knows to turn the other direction and not hit the wall.
	 * @param toTheLeft Whether the method should check if there is a wall to the left or right of the robot.
	 * @return
	 */
	public static boolean isThereAWall(boolean toTheLeft) {
		double currentX = odometer.getXYT()[0];
		double currentY = odometer.getXYT()[1];
		String heading = getCurrentHeading();
		
		//assumes robot is in middle of tunnel tile, subtracting or adding 1 tile length
		//depending on heading and toTheLeft boolean should allow check for whether wall exists
		if(toTheLeft) {
			if(heading.equalsIgnoreCase("north")) { 						
				if(currentX - SQUARE_SIZE < 0)
					return true;
			} else if (heading.equalsIgnoreCase("east")) {					
				if(currentY + SQUARE_SIZE > field_Y_Max)
					return true;
			} else if (heading.equalsIgnoreCase("south")) { 			
				if(currentX + SQUARE_SIZE > field_X_Max)
					return true;
			} else if (heading.equalsIgnoreCase("west")) { 				
				if(currentY - SQUARE_SIZE < 0)
					return true;
			}
		} else {
			if(heading.equalsIgnoreCase("north")) { 
				if(currentX + SQUARE_SIZE > field_X_Max)
					return true;
			} else if (heading.equalsIgnoreCase("east")) {		
				if(currentY - SQUARE_SIZE < 0)
					return true;
			} else if (heading.equalsIgnoreCase("south")) { 		
				if(currentX - SQUARE_SIZE < 0)
					return true;
			} else if (heading.equalsIgnoreCase("west")) { 				
				if(currentY + SQUARE_SIZE > field_Y_Max)
					return true;
			}
		}
		return false;
	}
	
	
	/**
	 * This method assumes the robot has finished picking up the rings and takes the robot to and through the tunnel.
	 * The robot picks up the rings by moving anti clockwise. Since it starts off at the target coordinate it first moved
	 * to to reach the ringset, we move the robot in anti clockwise until it reaches the coordinate again to easily avoid
	 * hitting the ringset.
	 * @throws OdometerExceptions
	 */
	public static void ringSetToTunnel() throws OdometerExceptions {
		Navigation.setupHeadingForDetection(false);	
		
		//until we arrive to the target coordinate, move anticlockwise correcting on the lines
		while(!hasArrived(targetCoordinate[0], targetCoordinate[1], 10)) {
			//if there is an edge of the tunnel ahead, move by 60 degrees instead to avoid the edge
			if(RingController.shouldWeTurn) {
				Navigation.turnRobot(60, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
				Navigation.moveStraight(12, true, false);
			}
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
			turnRobot(90, false, false, ROTATE_SPEED_FAST, ROTATE_ACCEL_SLOW);
			findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
		}
		
		
		double myX = odometer.getXYT()[0];
		double myY = odometer.getXYT()[1];
		myX = odometer.getXYT()[0];
		myY = odometer.getXYT()[1];
		
		//now it should be where it first started, now go to closest upper tile to midpoint point
		double[] destination = findClosestTunnelSideCoordinates();
		double[] tunnelExitMidpoint = findTunnelMidpoint(false);
		double[] tunnelExitMidpointPlusOne = findTunnelExitMidpointPlusTile();

		
		//if distance to the tunnel midpoint is smaller than 45, it is already at the desired intersection and can just move to the tunnel
		//exit midpoint, else move first perpendicular to tunnel orientation
		double distanceToTunnel = Math.hypot(tunnelExitMidpointPlusOne[0] - myX, tunnelExitMidpointPlusOne[1] - myY);
		if(distanceToTunnel > 45) {
			if(!isTunnelVertical) {
				travelToWithCorrection(destination[0], myY);
				travelToWithCorrection(destination[0], destination[1]);
			} else {
				travelToWithCorrection(myX, destination[1]);
				travelToWithCorrection(destination[0], destination[1]);
			}
		}

		myX = odometer.getXYT()[0];
		myY = odometer.getXYT()[1];
		
		//travel to tunnel exit midpoint plue one tile
		if(isTunnelVertical) {
			travelToWithCorrection(tunnelExitMidpointPlusOne[0], myY);
			travelToWithCorrection(tunnelExitMidpointPlusOne[0], tunnelExitMidpointPlusOne[1]);
		} else {
			travelToWithCorrection(myX, tunnelExitMidpointPlusOne[1]);
			travelToWithCorrection(tunnelExitMidpointPlusOne[0], tunnelExitMidpointPlusOne[1]);
		}

		
		turnToCoord(tunnelExitMidpoint[0], tunnelExitMidpoint[1], ROTATE_SPEED_SLOW, ROTATE_ACCEL_SLOW);

		//now travel through the tunnel
		findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);
		leftMotor.setSpeed(NAV_WITH_CORR_SPEED + LEFT_MOTOR_SPEED_OFFSET);
		rightMotor.setSpeed(NAV_WITH_CORR_SPEED);
		leftMotor.setAcceleration(3000);
		rightMotor.setAcceleration(3000);

		moveStraight(TUNNEL_DISTANCE_PASS, true, false);
		findLineStraight(true, NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL, true);

	}
	
	
	/**
	 * This method finds the coordinate of the midpoint of the tile just after the tunnel exit midpoint.
	 * @return The coordinates of the middle of the tile adjacent to the tunnel exit midpoint
	 */
	public static double[] findTunnelExitMidpointPlusTile() {
		double[] tunnelStartMidpoint = findTunnelMidpoint(true);
		double[] tunnelExitMidpoint = findTunnelMidpoint(false);

		double[] coordinates = new double[2];

		if(isTunnelVertical && tunnelStartMidpoint[1] < tunnelExitMidpoint[1]) { 			//north
			coordinates[0] = tunnelExitMidpoint[0];
			coordinates[1] = tunnelExitMidpoint[1] + SQUARE_SIZE;
		} else if (isTunnelVertical && tunnelStartMidpoint[1] > tunnelExitMidpoint[1]) {	//south
			coordinates[0] = tunnelExitMidpoint[0];
			coordinates[1] = tunnelExitMidpoint[1] - SQUARE_SIZE;
		} else if (!isTunnelVertical && tunnelStartMidpoint[0] < tunnelExitMidpoint[0]) {	//east
			coordinates[0] = tunnelExitMidpoint[0] + SQUARE_SIZE;
			coordinates[1] = tunnelExitMidpoint[1];
		} else if (!isTunnelVertical && tunnelStartMidpoint[0] > tunnelExitMidpoint[0]) {	//west
			coordinates[0] = tunnelExitMidpoint[0] - SQUARE_SIZE;
			coordinates[1] = tunnelExitMidpoint[1];
		}
		return coordinates;
	}
	
	
	/**
	 * This method finds the coordinates of the middle of the tiles to the left and right of the exit tunnel
	 * tile. It does so in order for the robot to move to it and correct both axis' to move through the tunnel smoothly.
	 * It returns the closest one of the two coordinates hence should be called when about to return. 
	 * @return	The coordinates the robot will move in to reach the tunnel exit point
	 */
	public static double[] findClosestTunnelSideCoordinates() {
		
		//find 
		double[] tunnelStartMidpoint = findTunnelMidpoint(true);
		double[] tunnelExitMidpoint = findTunnelMidpoint(false);

		double[] coordinatesRight = new double[2];
		double[] coordinatesLeft = new double[2];
		
		//then find the midpoint of the tiles to the right and left of the tunnel exit
		//example:
		// if tunnel is vertical and the robot enters the tunnel facing north
		//  coordinate 1 (right tile) - increase x and y values by 1 tile
		//  coordinate 2 (left tile) - decrease x and increase y values by 1 tile
		if(isTunnelVertical && tunnelStartMidpoint[1] < tunnelExitMidpoint[1]) { 			//north
			coordinatesRight[0] = tunnelExitMidpoint[0] + SQUARE_SIZE;
			coordinatesRight[1] = tunnelExitMidpoint[1] + SQUARE_SIZE;
			coordinatesLeft[0] = tunnelExitMidpoint[0] - SQUARE_SIZE;
			coordinatesLeft[1] = tunnelExitMidpoint[1] + SQUARE_SIZE;
		} else if (isTunnelVertical && tunnelStartMidpoint[1] > tunnelExitMidpoint[1]) {	//south
			coordinatesRight[0] = tunnelExitMidpoint[0] - SQUARE_SIZE;
			coordinatesRight[1] = tunnelExitMidpoint[1] - SQUARE_SIZE;
			coordinatesLeft[0] = tunnelExitMidpoint[0] + SQUARE_SIZE;
			coordinatesLeft[1] = tunnelExitMidpoint[1] - SQUARE_SIZE;
		} else if (!isTunnelVertical && tunnelStartMidpoint[0] < tunnelExitMidpoint[0]) {	//east
			coordinatesRight[0] = tunnelExitMidpoint[0] + SQUARE_SIZE;
			coordinatesRight[1] = tunnelExitMidpoint[1] - SQUARE_SIZE;
			coordinatesLeft[0] = tunnelExitMidpoint[0] + SQUARE_SIZE;
			coordinatesLeft[1] = tunnelExitMidpoint[1] + SQUARE_SIZE;
		} else if (!isTunnelVertical && tunnelStartMidpoint[0] > tunnelExitMidpoint[0]) {	//west
			coordinatesRight[0] = tunnelExitMidpoint[0] - SQUARE_SIZE;
			coordinatesRight[1] = tunnelExitMidpoint[1] + SQUARE_SIZE;
			coordinatesLeft[0] = tunnelExitMidpoint[0] - SQUARE_SIZE;
			coordinatesLeft[1] = tunnelExitMidpoint[1] - SQUARE_SIZE;
		}

		double myX = odometer.getXYT()[0];
		double myY = odometer.getXYT()[1];

		//return whichever is closer to move to depedning on current position
		double[] tileCoordinates = 
				Math.hypot(coordinatesRight[0] - myX, coordinatesRight[1] - myY) < Math.hypot(coordinatesLeft[0] - myX, coordinatesLeft[1] - myY) 
				? coordinatesRight : coordinatesLeft;
		return tileCoordinates;
	}
	
	
	/**
	 * This method travels from the tunnel to the starting corner, expects the robot so have exited the tunnel
	 * successfuly, turns towards the starting corner and moves to within it by a tile without correction
	 * @throws OdometerExceptions
	 */
	public static void travelTunnelToStart() throws OdometerExceptions {
		double startCornerCoordX = -1, startCornerCoordY = 1;
		int startingCorner = RedTeam == 9 ? RedCorner : GreenCorner;
		
		//retrieve starting corner coordinates
		switch(startingCorner) {
		case 0:
			startCornerCoordX = field_X_Min*SQUARE_SIZE;
			startCornerCoordY = field_Y_Min*SQUARE_SIZE;
			break;
		case 1:
			startCornerCoordX = field_X_Max*SQUARE_SIZE;
			startCornerCoordY = field_Y_Min*SQUARE_SIZE;
			break;
		case 2:
			startCornerCoordX = field_X_Max*SQUARE_SIZE;
			startCornerCoordY = field_Y_Max*SQUARE_SIZE;
			break;
		case 3:
			startCornerCoordX = field_X_Min*SQUARE_SIZE;
			startCornerCoordY = field_Y_Max*SQUARE_SIZE;
			break;
		}
		double myX = odometer.getXYT()[0];
		double myY = odometer.getXYT()[1];
		//turn to the starting corner
		turnToCoord(startCornerCoordX, startCornerCoordY, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST);
		double distance = Math.hypot(startCornerCoordX - myX, startCornerCoordY - myY);
		distance -= SQUARE_SIZE;
		setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);
		//move to the starting corner
		moveStraight(distance, true, false);
	}
	
	
	
	/**
	 * This method assumes the robot is directly on top of one of the 4 intersections
	 * adjacent to the ring set. It turns the robot so that it is either facing the ringset
	 * or facing so that the ringset is to the left of the robot for traversal.
	 */
	public static void setupHeadingForDetection(boolean straightAhead) {
		int ringSetX, ringSetY;
		
		if(RedTeam == 9) {
			ringSetX = TR_x;
			ringSetY = TR_y;
		} else {
			ringSetX = TG_x;
			ringSetY = TG_y;
		}
		
		double myX = odometer.getXYT()[0] / SQUARE_SIZE;
		double myY = odometer.getXYT()[1] / SQUARE_SIZE;
		int x = (int) Math.round(myX);
		int y = (int) Math.round(myY);
		
		int angle1 = 180, angle2 = 0, angle3 = 90, angle4 = 270;
		
		if(straightAhead) {
			angle1 = 90;
			angle2 = 270;
			angle3 = 0;
			angle4 = 180;
		}
		
		
		if(ringSetX > x) 													//if ringsetX is larger than robot x, its to the left of ringset 
			turnTo(angle1, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
		else if(ringSetX < x) 												//if ringsetX is smaller than robot x, its to the right of ringset
			turnTo(angle2, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
		else if(ringSetY > y) 												//if ringsetY is larger than robot y, its below the ringset
			turnTo(angle3, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
		else if(ringSetY < y) 												//if ringsetY is smaller than robot y, its above the ringset
			turnTo(angle4, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false);
		
		
	}
	
	
	/**
	 * This method takes in whether the ringset is to the left or right of the robot upon exiting the tunnel.
	 * If the ring is to left, go to tile intersection to the right of it relative to robot heading,
	 * else if ring is to the right, go to tile intersection to the left of it relative to robot heading.
	 * @param ringSetIsLeft True if ringset is to the left of the robot's current position having just exited the tunnel
	 * @return	The coordinates of the position the robot should navigate to
	 */
	public static double[] getTargetCoordinate(boolean ringSetIsLeft) {
		
		double[] tileCoord = new double[2];
		
		//if we are red team use red team ring set, else green ring set
		//retrieve the location of the ring set
		if(RedTeam == 9) { 	
			tileCoord[0] = TR_x * SQUARE_SIZE;
			tileCoord[1] = TR_y * SQUARE_SIZE;
		} else {
			tileCoord[0] = TG_x * SQUARE_SIZE;
			tileCoord[1] = TG_y * SQUARE_SIZE;
		}
		
		double[] tunnelStartMidpoint = findTunnelMidpoint(true);
		double[] tunnelExitMidpoint = findTunnelMidpoint(false);
		
		//depending on the orientation of the tunnel, the coordinate location of the start and end tunnel midpoints,
		//and whether the ringset is to the left of the robot, retrieve the target coordinate
		//example: if ringset is to the left, robot exited tunnel and is facing north, increment x coordinate of ring set to get target coordinate
		if(isTunnelVertical && tunnelStartMidpoint[1] < tunnelExitMidpoint[1]) { 			//north
			if(ringSetIsLeft) {		// +x -y
				tileCoord[0] +=  (SQUARE_SIZE);
			} else {				// -x -y
				tileCoord[0] -=  (SQUARE_SIZE);
			}
		} else if (isTunnelVertical && tunnelStartMidpoint[1] > tunnelExitMidpoint[1]) {	//south
			if(ringSetIsLeft) {		// -x +y
				tileCoord[0] -=  (SQUARE_SIZE);
			} else {				// +x +y
				tileCoord[0] +=  (SQUARE_SIZE);
			}
		} else if (!isTunnelVertical && tunnelStartMidpoint[0] < tunnelExitMidpoint[0]) {	//east
			if(ringSetIsLeft) {		// -x -y
				tileCoord[1] -=  (SQUARE_SIZE);
			} else {				// -x +y
				tileCoord[1] +=  (SQUARE_SIZE);
			}
		} else if (!isTunnelVertical && tunnelStartMidpoint[0] > tunnelExitMidpoint[0]) {	//west
			if(ringSetIsLeft) {		// +x +y
				tileCoord[1] +=  (SQUARE_SIZE);
			} else {				// +x -y
				tileCoord[1] -=  (SQUARE_SIZE);
			}
		}
		return tileCoord;
	}

	
	

	/**
	 * This method uses the game parameters only to calculate whether the ring set will be to the left
	 * or to the right of the robot upon exiting the tunnel. It does not depend on the current position
	 * of the robot so can be called at any moment and return the same result.
	 * @return : true if ring set is to the left of the robot, false if to the right
	 */
	public static boolean isRingSetToLeft() {

		int ringSetX, ringSetY;
		boolean isToLeft = false;

		//if we are red team use red team ring set, else green ring set
		if(RedTeam == 9) { 	
			ringSetX = TR_x;
			ringSetY = TR_y;
		} else {
			ringSetX = TG_x;
			ringSetY = TG_y;
		}
		
		double[] tunnelStartMidpoint = findTunnelMidpoint(true);
		double[] tunnelExitMidpoint = findTunnelMidpoint(false);
		if(isTunnelVertical && tunnelStartMidpoint[1] < tunnelExitMidpoint[1]) { 			//north - if start midpoint has lower y than exit midpoint
			if(ringSetX < ((int)tunnelExitMidpoint[0]/SQUARE_SIZE)) 
				isToLeft = true;
		} else if (isTunnelVertical && tunnelStartMidpoint[1] > tunnelExitMidpoint[1]) {	//south - if start midpoint has higher y than exit midpoint
			if(ringSetX > ((int)tunnelExitMidpoint[0]/SQUARE_SIZE)) 
				isToLeft = true;
		} else if (!isTunnelVertical && tunnelStartMidpoint[0] < tunnelExitMidpoint[0]) {	//east	- if start midpoint has lower x than exit midpoint
			if(ringSetY > ((int)tunnelExitMidpoint[1]/SQUARE_SIZE)) 
				isToLeft = true;
		} else if (!isTunnelVertical && tunnelStartMidpoint[0] > tunnelExitMidpoint[0]) {	//west - if start midpoint has higher x than exit midpoint
			if(ringSetY < ((int)tunnelExitMidpoint[1]/SQUARE_SIZE)) 
				isToLeft = true;
		}
		return isToLeft;
	}
	
	
	/**
	 * This method gets the current theta heading from the odometer and returns
	 * the heading of the robot as either north, south, east, or west.
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
	 * This method makes the robot move in the direction of the waypoint whose coordinates are passed as arguments. The robot
	 * moves until a perpendicular line is crossed where the first sensor to cross the line that motor will stop while the other 
	 * wheel moves until that side's sensor picks up the same line, correcting the robots heading. The method assumes the coordinates 
	 * given to it will make the robot travel either horizontally or vertically. The method travels with correction until it has arrived ]
	 * to the target coordinates.
	 * @param x : target x coordinate
	 * @param y : target y coordinate
	 * @throws OdometerExceptions
	 */ 
	public static void travelToWithCorrection(double x, double y) throws OdometerExceptions {
		// Define variables
		double odo[] = { 0, 0, 0 }, absAngle = 0, dist = 0, deltaX = 0, deltaY = 0;
		
		//if it has not arrived 
		while(!hasArrived(x, y, 4)) {
			
			// Get odometer readings
			odo = odometer.getXYT();
			
			// Get displacement to travel on X and Y axis
			deltaX = x - odo[0];
			deltaY = y - odo[1];

			// Displacement to point (hypothenuse)
			dist = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));
			
			// Get absolute angle the robot must be facing
			absAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

			//turn towards coordinates
			turnTo(absAngle, ROTATE_SPEED_FAST, ROTATE_ACCEL_FAST, false); 
			
			setSpeedAcceleration(NAV_WITH_CORR_SPEED, NAV_WITH_CORR_ACCEL);
			
			//we either move to a perpendicular line or move without correction
			//if the distance to the waypoint is smaller than a tile size minus an error offset - move without correction
			//else move with correction
			if(dist < SQUARE_SIZE - 4 ) {
				moveStraight(dist, true, false);
			} else  {
				
				//correct for weight offset
				leftMotor.rotate(7, true);
				rightMotor.rotate(-7, false);
				
				double oldSampleRight = 0;
				double oldSampleLeft = 0;
				float[] newColorLeft = new float[leftSampleProvider.sampleSize()];
				float[] newColorRight = new float[rightSampleProvider.sampleSize()];

				//move until sensors detect perpendicular line
				leftMotor.forward();
				rightMotor.forward();

				int foundLeft = 0; int foundRight = 0;
				while(true) {
					// Get color sensor readings
					leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
					rightSampleProvider.fetchSample(newColorRight, 0); 

					// If line detected for left sensor (intensity less than 0.4), only count once by keeping track of last value
					if((newColorLeft[0]) < LIGHT_THRESHOLD && oldSampleLeft > LIGHT_THRESHOLD && foundLeft == 0) {
						leftMotor.stop(true);
						foundLeft++;
					}
					// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
					if((newColorRight[0]) < LIGHT_THRESHOLD && oldSampleRight > LIGHT_THRESHOLD && foundRight == 0) {
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
	 * Checks if the robot is within a certain distance from a destination.
	 * @param xd : x coordinate desitination
	 * @param yd : y coordinate destination
	 * @param errorMargin: accepted distance error to target waypoint
	 * @return : true if its within the distance
	 * @throws OdometerExceptions
	 */
	public static boolean hasArrived(double xd, double yd, int errorMargin) throws OdometerExceptions {
		double odometer[] = { 0, 0, 0 };
		odometer = Odometer.getOdometer().getXYT();
		double xf = odometer[0];
		double yf = odometer[1];
		double cErr = Math.hypot(xf - xd, yf - yd);
		return cErr < errorMargin;
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
	 * and sets the class variable isTunnelVertical accordingly. It will also find the tunnel
	 * heading if the tunnel is of size 1 tile. It sets the static variable isTunnelVertical.
	 */
	public static void findTunnelHeading() {
		int tn_LL_x, tn_UR_x,tn_LL_y, tn_UR_y;
		if(RedTeam == 9) {
			tn_LL_x = TNR_LL_x;
			tn_UR_x = TNR_UR_x;
			tn_LL_y = TNR_LL_y;
			tn_UR_y = TNR_UR_y;
		} else {
			tn_LL_x = TNG_LL_x;
			tn_UR_x = TNG_UR_x;
			tn_LL_y = TNG_LL_y;
			tn_UR_y = TNG_UR_y;
		}
		
		//if the tunnel is size 1 tile, use the team corner points to check if the tunnel is vertical or horizontal
		if((Math.hypot(tn_UR_x - tn_LL_x, tn_UR_y - tn_LL_y)*SQUARE_SIZE) < 45) {
			isOneTile = true;
			int team_LL_y, team_UR_y;
			int startingCorner = -1;
			if(RedTeam == 9) {
				team_LL_y = Red_LL_y; 
				team_UR_y = Red_UR_y; 
				startingCorner = RedCorner;
			} else {
				team_LL_y = Green_LL_y; 
				team_UR_y = Green_UR_y; 
				startingCorner = GreenCorner;
			}
			switch(startingCorner) {
			case 0:
				if(tn_LL_y == team_UR_y) {
					isTunnelVertical = true;
				} 
				break;
			case 1:
				if(tn_LL_y == team_UR_y) 
					isTunnelVertical = true;
				break;
			case 2:
				if(tn_UR_y == team_LL_y) {
					isTunnelVertical = true;
				}
				break;
			case 3:
				if(tn_UR_y == team_LL_y) {
					isTunnelVertical = true;
				}
				break;
			}
		} 
		//otherwise check to see if the tunnel's x have a difference of 2, if so its horizontal, otherwise vertical
		else {
			isTunnelVertical = (Math.abs(tn_LL_x - tn_UR_x) == 2) ? false : true;
		}
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
		//if tunnel is vertical then midpoints are the upper coordinate minus 0.5 for the x value
		// and + 0.5 for the lower coordinate
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
		int startingCorner;
		if(RedTeam == 9) 
			startingCorner = RedCorner;
		else 
			startingCorner = GreenCorner;
			
		double startX, startY;
		

		//calculate starting corner coordinates
		switch(startingCorner) {
		case 0:
			startX = (Navigation.SQUARE_SIZE);
			startY = (Navigation.SQUARE_SIZE);
			break;
		case 1:
			startX = ((field_X_Max-1) * Navigation.SQUARE_SIZE);
			startY = (Navigation.SQUARE_SIZE);
			break;
		case 2:
			startX = ((field_X_Max-1) * Navigation.SQUARE_SIZE);
			startY = ((field_Y_Max-1) * Navigation.SQUARE_SIZE);
			break;
		case 3:
			startX = (Navigation.SQUARE_SIZE);
			startY = ((field_Y_Max-1) * Navigation.SQUARE_SIZE);
			break;
		default:
			startX = (Navigation.SQUARE_SIZE);
			startY = (Navigation.SQUARE_SIZE);
			break;
		}


		double dX1 = midpoint1[0] - startX; 
		double dY1 = midpoint1[1] - startY; 
		double dX2 = midpoint2[0] - startX; 
		double dY2 = midpoint2[1] - startY; 

		// displacement to points
		double dist1 = Math.hypot(Math.abs(dX1), Math.abs(dY1));
		double dist2 = Math.hypot(Math.abs(dX2), Math.abs(dY2));
		
		//start is the midpoint closer to the starting corner
		double[] startMidpoint  = (dist1 > dist2) ? midpoint2 : midpoint1;
		double[] endMidpoint = (dist1 < dist2) ? midpoint2 : midpoint1;
		
		//return start or end
		if(start)
			return startMidpoint;
		else 
			return endMidpoint;
	}
	
	
	/**
	 * This method expects the robot to be heading perpendicular to a grid line and moves until it finds
	 * the grid line and localizes on it. 
	 * @param forwards	True - it localizes on perpendicular line ahead, false - on line behind
	 * @param speed	Motor speed
	 * @param acceleration Motor acceleration
	 * @param shouldWeCorrect Boolean checks to see if we should use correction
	 * @throws OdometerExceptions
	 */
	public static void findLineStraight(boolean forwards, int speed, int acceleration, boolean shouldWeCorrect) throws OdometerExceptions {
		
		float[] newColorLeft = {0};
		float oldSampleLeft = 0;
		float[] newColorRight = {0};
		float oldSampleRight = 0;
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;
		
		setSpeedAcceleration(speed, acceleration);

		if(forwards) {
			leftMotor.forward();
			rightMotor.forward();
		} else {
			leftMotor.backward();
			rightMotor.backward();
		}


		while(true) {
			// Get color sensor readings
			leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
			rightSampleProvider.fetchSample(newColorRight, 0); 

			// If line detected for left sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft[0]) < LIGHT_THRESHOLD && oldSampleLeft > LIGHT_THRESHOLD && foundLeft == 0) {
				leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight[0]) < LIGHT_THRESHOLD && oldSampleRight > LIGHT_THRESHOLD && foundRight == 0) {
				rightMotor.stop(true);
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
		
		if(shouldWeCorrect) 
			correctOdometer();
		
		moveStraight(SENSOR_OFFSET, true, false);
	}
	


	
    /**
	 * This method causes the robot to turn (on point) to the absolute heading
	 * theta. This method should turn a MINIMAL angle to its target.
     * @param theta : double theta angle
     * @param speed : int motor speed
     * @param accel : int motor acceleration
     */
	public static void turnTo(double theta, int speed, int accel, boolean rightCorrection) {
		double dTheta = theta - odometer.getXYT()[2];
		if(dTheta < 0) dTheta += 360;
		setSpeedAcceleration(speed, accel);

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
	 * @param x X coordinate to turn to
	 * @param y Y coordinate to turn to
	 */
	public static void turnToCoord(double x, double y, int speed, int accel) {
		double x0 = odometer.getXYT()[0];
		double y0 = odometer.getXYT()[1];
		double xVector = x - x0;
		double yVector = y - y0;
		double thetaVector = Math.toDegrees(Math.atan2(xVector, yVector));
		turnTo(thetaVector, speed, accel, false);
	}
	

	/**
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
	public static void turnRobot(int degrees, boolean direction, boolean continueRunning, int speed, int accel) {
		setSpeedAcceleration(speed, accel);
		int i = 1;
		if (!direction) i = -1;
		leftMotor.rotate(i * convertAngle(degrees), true);
		rightMotor.rotate(i * -convertAngle(degrees), continueRunning);
	}
	
	/**
	 * This methods stops both of the motors and returns immediately
	 */
	public static void stopMotors() {
		leftMotor.stop(true);
		rightMotor.stop(true);
	}
	
	/**
	 * This method sets the speed and acceleration of the robot
	 * @param speed The speed to set both the motors
	 * @param acceleration The acceleration to set both motors
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
