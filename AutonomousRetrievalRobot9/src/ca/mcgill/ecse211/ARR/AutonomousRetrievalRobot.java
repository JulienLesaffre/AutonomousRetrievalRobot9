/**
 * 
 */
package ca.mcgill.ecse211.ARR;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author JulienLesaffre
 *
 */
public class AutonomousRetrievalRobot {
	
	// Game parameters (will be provided with Wifi Class)
	private static final int RedTeam = 0; // Team starting out from red zone
	private static final int GreenTeam = 0; // Team starting out from green zone
	private static final int[] RedCorner = {0,0}; // Starting corner for red team
	private static final int[] GreenCorner = {0,0}; // Starting corner for green team
	private static final int[] Red_LL = {0,0}; // lower left hand corner of Red Zone
	private static final int[] Red_UR = {0,0}; // upper right hand corner of Red Zone
	private static final int[] Green_LL = {0,0}; // lower left hand corner of Green Zone
	private static final int[] Green_UR = {0,0}; // upper right hand corner of Green Zone
	private static final int[] BRR_LL = {0,0}; // lower left hand corner of the red tunnel footprint
	private static final int[] BRR_UR = {0,0}; // upper right hand corner of the red tunnel footprint
	private static final int[] BRG_LL = {0,0}; // lower left hand corner of the green tunnel footprint
	private static final int[] BRG_UR = {0,0}; // upper right hand corner of the green tunnel footprint
	private static final int[] TR_LL = {0,0}; // lower left hand corner of the red player ring set
	private static final int[] TR_UR = {0,0}; // upper right hand corner of the red player ring set
	private static final int[] TG_LL = {0,0}; // lower left hand corner of the green player ring set
	private static final int[] TG_UR = {0,0}; // upper right hand corner of the green player ring set

	
	 // Motor objects and robot parameters

	public static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static void main(String[] args) {
		int buttonChoice;
		do {
			lcd.clear(); // clear the display
			lcd.drawString("      ", 0, 0);
			lcd.drawString("Start ? ", 0, 1);
			lcd.drawString("      ", 0, 2);


			buttonChoice = Button.waitForAnyPress(); 
			
		} while (buttonChoice != Button.ID_ENTER);
		
	}

}
