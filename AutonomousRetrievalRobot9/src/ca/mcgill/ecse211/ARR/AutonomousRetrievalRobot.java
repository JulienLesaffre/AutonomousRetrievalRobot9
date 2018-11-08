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
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import java.util.Map;
import lejos.hardware.Button;

/*
////////////////////////////////////
/////////////// TODO ///////////////
 * check if turnto method works properly, it might do 360 turns because the odometer is already off and it thinks its heading another direction
 * running slowly, increase speed, if can't its because too much on cpu
 * turn off sensors when not needed i.e. ultrasonic can disable/enable work?

 * 
 * Structure:
 * robot is placed in random orientation in corner, need to account for it starting at low edge
 * have to wait once placed in corner for wifi to pass parameters
 * beep 3 times after localizing
 * go to ring set and once you get there issue 3 beeps
 * when you detect the rings you issue beeps based on their number
 * when you are done unloading you issue 5 beeps
 * 
 * 
 * TEST:
 * mean light filter and finite difference
 * able to recieve values from wifi
 * can test out going diagonally to the side of the tunnel and then "localizing"
 * scaling test if the robot is skewed
////////////////////////////////////
*/


/**
 * @author JulienLesaffre
 * @author FouadBitar
 * 
 * This is the main execution class for the robot.
 * 
 * There are no polling classes that create threads so as to minimize the CPU load. The SampleProviders
 * are initialized in the main and are sent to the classes that want to use it. This is so that we use same
 * initialization.
 *
 */
public class AutonomousRetrievalRobot {
	
	//associations
	static Odometer odometer = null;
	static Navigation nav = null;
	static UltrasonicLocalizer usLocalizer;
	static LightLocalizer lightLocalizer;
	static RingSet ringSet;

	//sensors and motors
	private static SampleProvider leftSampleProvider;
	private static SampleProvider rightSampleProvider;
	private static SampleProvider usSampleProvider;
	private static EV3LargeRegulatedMotor rightMotor;
	private static EV3LargeRegulatedMotor leftMotor;
	private static NXTRegulatedMotor ringPickUpMotor;
	private static EV3MediumRegulatedMotor lightSensorMotor;

	//wifi connection parameters
	private static final String SERVER_IP = "192.168.2.11";
	private static final int TEAM_NUMBER = 9;
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;


	/**
	 * Initializes the sensors and motors, starts odometer thread, and initializes class variables
	 * with the proper sensors/motors they require. For example navigation, odometer, and localizers 
	 * are given the motors. We initialize all the sensors and motors here so that all classes use the
	 * same instance.
	 * 
	 * @throws OdometerExceptions
	 */
	@SuppressWarnings("resource")
	private static void initialize() throws OdometerExceptions{
		
		//connect/intialize sensors and motors
		leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
		lightSensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
		ringPickUpMotor = new NXTRegulatedMotor(LocalEV3.get().getPort("C"));
		SensorModes myColorLeft = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		leftSampleProvider = myColorLeft.getMode("Red");
		SensorModes myColorRight = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
		rightSampleProvider = myColorRight.getMode("Red");
		SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		usSampleProvider = usSensor.getMode("Distance");

		
		
		//start odometer
		odometer = Odometer.getOdometer(leftMotor, rightMotor, Navigation.TRACK, Navigation.WHEEL_RAD);
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
       
		//initialize and give class variables required ev3 sensors and motors
		nav = new Navigation(leftSampleProvider, rightSampleProvider, odometer, leftMotor, rightMotor); 
		usLocalizer = new UltrasonicLocalizer(usSampleProvider, leftMotor, rightMotor);
		lightLocalizer = new LightLocalizer(leftSampleProvider, rightSampleProvider, odometer, leftMotor, rightMotor);
		ringSet = new RingSet(leftMotor, rightMotor, odometer, ringPickUpMotor, lightSensorMotor);
		
	}
	
	@SuppressWarnings("rawtypes")
	private static void retrieveDataFromServer() {
		
		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			
			//waits till start button pushed
			//can kill while waiting by pressing escape button
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
		}
		// Wait until user decides to continue
		Button.waitForAnyPress();
	}
	
	
	

	public static void main(String[] args) throws OdometerExceptions {
		
		Display.displayStartScreen(); 
		
		initialize(); 									//initialize class variables needed
		
		retrieveDataFromServer();						//connect to the server and wait to recieve variables
		
		usLocalizer.fallingEdge();						//us localize
		
		lightLocalizer.localize(); 						//light localize
		
		Navigation.travelStartToTunnel();
		

	}

}
