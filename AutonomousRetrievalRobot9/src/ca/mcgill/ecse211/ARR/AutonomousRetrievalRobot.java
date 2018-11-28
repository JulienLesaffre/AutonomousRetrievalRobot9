/**
 * 
 */
package ca.mcgill.ecse211.ARR;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.localizers.LightLocalizer;
import ca.mcgill.ecse211.localizers.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import java.util.Map;

/*
 * One size tunnel - all the parts too with the if its horizontal/vertical
 * edge case the one next to water, the one next to wall
 * 
 * READ THE WIFI PDF, it might say why the timer is not starting
 * did not take case into consideration start 00, at 22 and 34
 * 
 */

/**
 * This is the main execution class for the robot.
 * @author JulienLesaffre
 * @author FouadBitar
 */
public class AutonomousRetrievalRobot {
	
	//associations
	static Odometer odometer = null;
	static Navigation nav = null;
	static UltrasonicLocalizer usLocalizer;
	static LightLocalizer lightLocalizer;
	static RingDetection ringDetection;
	static RingController ringCont;

	//sensors and motors
	private static SampleProvider leftSampleProvider;
	private static SampleProvider rightSampleProvider;
	private static SampleProvider usSampleProvider;
	private static SensorModes usSensor;
	private static SampleProvider colorSampleProvider;
	private static SensorModes colorSensor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor dumpRingMotor;
	private static EV3MediumRegulatedMotor clawMotor;

	
	//wifi connection parameters
	private static final String SERVER_IP = "192.168.2.13";
	private static final int TEAM_NUMBER = 9;
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;


	/**
	 * Initializes the sensors and motors, starts odometer thread, and initializes class variables
	 * with the proper sensors/motors they require. For example navigation, odometer, and localizers 
	 * are given the motors. We initialize all the sensors and motors here so that all classes use the
	 * same instance.
	 * @throws OdometerExceptions
	 */
	@SuppressWarnings("resource")
	private static void initialize() throws OdometerExceptions{
		
		//motors
		leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
		clawMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
		dumpRingMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
		
		//sensors
		SensorModes myColorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		leftSampleProvider = myColorLeft.getMode("Red");
		SensorModes myColorRight = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
		rightSampleProvider = myColorRight.getMode("Red");
		usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		usSampleProvider = usSensor.getMode("Distance");
		colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
		colorSampleProvider = colorSensor.getMode("RGB");
		


		//start odometer thread
		odometer = Odometer.getOdometer(leftMotor, rightMotor, Navigation.TRACK, Navigation.WHEEL_RAD);
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
       
		//initialize classes with required ev3 sensors and motors
		nav = new Navigation(leftSampleProvider, rightSampleProvider, odometer, leftMotor, rightMotor); 
		usLocalizer = new UltrasonicLocalizer(odometer, usSampleProvider, leftMotor, rightMotor);
		lightLocalizer = new LightLocalizer(leftSampleProvider, rightSampleProvider, odometer, leftMotor, rightMotor);
		ringDetection = new RingDetection(colorSampleProvider);
		ringCont = new RingController(dumpRingMotor, clawMotor);
	}
	

	/**
	 * This method connects to the server specified by the IP address variable and waits for 
	 * the server to pass the game parameters for the round. It extracts the data and places
	 * the data in the correct variable in the Navigation class.
	 */
	private static void retrieveDataFromServer() {
		
		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			
			//waits till start button pushed
			//can kill while waiting by pressing escape button
			@SuppressWarnings("rawtypes")
			Map data = conn.getData();

			//assign all the corresponding variables by retrieving the data
			Navigation.RedTeam = ((Long) data.get("RedTeam")).intValue();
			Navigation.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
			Navigation.RedCorner = ((Long) data.get("RedCorner")).intValue();
			Navigation.GreenCorner = ((Long) data.get("GreenCorner")).intValue();
			Navigation.Red_LL_x = ((Long) data.get("Red_LL_x")).intValue();
			Navigation.Red_LL_y = ((Long) data.get("Red_LL_y")).intValue();
			Navigation.Red_UR_x = ((Long) data.get("Red_UR_x")).intValue();
			Navigation.Red_UR_y = ((Long) data.get("Red_UR_y")).intValue();
			Navigation.Green_LL_x = ((Long) data.get("Green_LL_x")).intValue();
			Navigation.Green_LL_y = ((Long) data.get("Green_LL_y")).intValue();
			Navigation.Green_UR_x = ((Long) data.get("Green_UR_x")).intValue();
			Navigation.Green_UR_y = ((Long) data.get("Green_UR_y")).intValue();
			Navigation.Island_LL_x = ((Long) data.get("Island_LL_x")).intValue();
			Navigation.Island_LL_y = ((Long) data.get("Island_LL_y")).intValue();
			Navigation.Island_UR_x = ((Long) data.get("Island_UR_x")).intValue();
			Navigation.Island_UR_y = ((Long) data.get("Island_UR_y")).intValue();
			Navigation.TNR_LL_x = ((Long) data.get("TNR_LL_x")).intValue();
			Navigation.TNR_LL_y = ((Long) data.get("TNR_LL_y")).intValue();
			Navigation.TNR_UR_x = ((Long) data.get("TNR_UR_x")).intValue();
			Navigation.TNR_UR_y = ((Long) data.get("TNR_UR_y")).intValue();
			Navigation.TNG_LL_x = ((Long) data.get("TNG_LL_x")).intValue();
			Navigation.TNG_LL_y = ((Long) data.get("TNG_LL_y")).intValue();
			Navigation.TNG_UR_x = ((Long) data.get("TNG_UR_x")).intValue();
			Navigation.TNG_UR_y = ((Long) data.get("TNG_UR_y")).intValue();
			Navigation.TR_x = ((Long) data.get("TR_x")).intValue();
			Navigation.TR_y = ((Long) data.get("TR_y")).intValue();
			Navigation.TG_x = ((Long) data.get("TG_x")).intValue();
			Navigation.TG_y = ((Long) data.get("TG_y")).intValue();

		} catch (Exception e) {
			//throws exception when: wrong IP, server not running, not connected to WIFI
			//also throws exception if: recieves bad data, message from server, e.g. make sure TEAM numb correct
			System.err.println("Error: " + e.getMessage());
			RingController.makeSound(6);
		}
	}
	
	
	
	/**
	 * This method expects the pole to be in any position, and the claw medium motor to be
	 * at the max rotated in the negative direction. It raises the pole until it stalls
	 * where the motor stops and resets the tachometer count so that all movement afterwards
	 * is with rotateTo. Expects the motor variables to be initialized and connected to the ports.
	 */
	public static void initializeMotors() {
		dumpRingMotor.resetTachoCount();
		clawMotor.resetTachoCount();
		clawMotor.setAcceleration(4000);
		clawMotor.setSpeed(250);
	}
	
	
	public static void main(String[] args) throws OdometerExceptions {
		
		initialize(); 									//initialize class variables needed

		initializeMotors();
		
		clawMotor.rotateTo(135);
		clawMotor.flt();

		retrieveDataFromServer();						//connect to the server and wait to recieve variables
		
		usLocalizer.fallingEdge();						//us localize
		
		lightLocalizer.localize(); 						//light localize
		
		Navigation.travelToTunnel(true);				//travel to and through tunnel
		
		Navigation.travelTunnelToRingSet();				//travel from tunnel to ring set
		

		RingController.detectAllRings();
		
		RingController.pickUpRings();
		
		Navigation.ringSetToTunnel();
		
		Navigation.travelTunnelToStart();
		
		RingController.dropRings();
		
	}
}
