
import lejos.hardware.Battery;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;
import lejos.utility.Timer;

//add more commments 
//for colour sensor
//add dark light dettector

public class RoboticsProject {
	
	//private fields avoiding magic numbers
	final static float WHEEL_DIAMETER = 51; // The diameter (mm) of the wheels
	final static float AXLE_LENGTH = 44; // The distance (mm) your two driven wheels
	final static float ANGULAR_SPEED = 5; // How fast around corners (degrees/sec)
	final static float LINEAR_SPEED = 70; // How fast in a straight line (mm/sec)
	final static float max_level = (float) 0.15; // max level 
	final static float min_level = (float) 0.04;// min level 
    final static float LIGHT_AVERAGE = (max_level + min_level)/2 ;// average 
    final static float LEFT = 555;//value for left 
    final static float RIGHT = 185;//value for right 
    final static float RETURNPATH = 380;// value for return path 
    
	public static void main(String args[]) {
		//welcome message 
		LCD.drawString("Team 9 BMO", 0, 1 );
		LCD.drawString("by Kieran, Nerea", 0, 2 );
		LCD.drawString("and Isha ", 0, 3 );

		//start obstacle course
		LCD.drawString("Press button ", 0,4);
		LCD.drawString("to start: ", 0,5);
		Button.LEDPattern(1);
		Sound.beepSequenceUp();
		//starts when button is pressed
		Button.ENTER.waitForPressAndRelease();
		Delay.msDelay(1000);
		LCD.clear();
		//holds light level for colour sensor 
		float[] level = new float[1]; 
		
		
		// creates new colour sensor with RGB mode 
        EV3ColorSensor ss = new EV3ColorSensor(SensorPort.S1);
        SampleProvider light = ss.getRGBMode();
        //creates Ultrasonic sensor for port 2 uses distance mode 
        EV3UltrasonicSensor us = new EV3UltrasonicSensor(SensorPort.S2);
		SampleProvider sp = us.getDistanceMode();
		//creates a pilot for uses motor ports 1 and 2 
		MovePilot pilot = getPilot(MotorPort.A, MotorPort.B, 60, 29);
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
		SampleProvider dp = colorSensor.getRGBMode();
		
		// creates instance of all behaviours using the behaviour parent class  + dark behavior
		Behavior Trundle = new trundle(pilot);
		Behavior escape = new Backup( sp, pilot);
		Behavior Battery = new dark(pilot,LIGHT_AVERAGE  , dp, level);
		Behavior Emergency = new emergency_stop(pilot);// if button stopped Emergency stops program
		Behavior Colour = new colour(light,pilot);

		//creates the list of behaviours
		
		Arbitrator behaviour_go = new Arbitrator(new Behavior[] {Battery, Trundle ,  Emergency, escape ,Colour });
		
		//launches the behaviours 
		behaviour_go.go(); 
		}
	
	private static MovePilot getPilot(Port left, Port right, int diam, int offset) {
		BaseRegulatedMotor mL = new EV3LargeRegulatedMotor(left);
		Wheel wL = WheeledChassis.modelWheel(mL, diam).offset(-1 * offset);
		BaseRegulatedMotor mR = new EV3LargeRegulatedMotor(right);
		Wheel wR = WheeledChassis.modelWheel(mR, diam).offset(offset);
		Wheel[] wheels = new Wheel[] {wR, wL};
		Chassis chassis = new WheeledChassis(wheels, WheeledChassis.TYPE_DIFFERENTIAL);
		return new MovePilot(chassis);
		}
	public static class trundle implements Behavior{
		// creates a local move pilot
		private MovePilot pilot; 
		
			trundle(MovePilot p) {
			// Save the (shared) pilot in a field	
			this.pilot = p;} 
		
		
		// actions for trundle, drives forward  , reverses slightly then turns round.
		public void action() {
			
			pilot.setLinearSpeed(LINEAR_SPEED);
			pilot.forward();
		
		
		Delay.msDelay(5000);
			
			
		pilot.travel(-50,false);
		pilot.arc(0,RETURNPATH);
		}
		
		//driving forward, always returns true
		public boolean takeControl() {
			return true;
		}
		
		//stops and allow the next to take control
		public void suppress() {
			pilot.stop();
			
		}
}
	//the back up behaviour stops and reverse slightly.
public static class Backup implements Behavior{
		//creation of private MovePilot, Sample provider and float distance
		private MovePilot pilot;
		private SampleProvider kk;
		private float[] distance = new float[1];
		//boolean to end the behaviour
		boolean test = true;
		//local fields for pilot and sampleprovider
		Backup(SampleProvider kk, MovePilot pilot ) {
		this.pilot = pilot;
		this.kk = kk; }

		
		//take control when something is within the certain distance
	@Override
	public boolean takeControl() {
    	kk.fetchSample(distance, 0);
		return (distance[0] < 0.10f); 
	
    }

		//runs a loop in place of a timer and reverses after 
	@Override
	public void action() {
		if(pilot.isMoving() == false) {
		for(int I=0; I>100;I++) {
        }

		pilot.travel(-50,false);
		}
    }
		
		//turns test to false to allow the next behaviour
	@Override
	public void suppress() {
		test = false;
	
    }
	
}

public static class dark implements Behavior{
		//creates private fields for movePilot, SampleProvider, LIGHT_AVERAGE and level 
		private MovePilot pilot;
		private SampleProvider light ;
		private float LIGHT_AVERAGE ;
		private float[] level;
		//creates a shared field for all the private fields 
	dark(MovePilot pilot, float LIGHT_AVERAGE, SampleProvider light, float[] levels) {
		this.pilot = pilot; // Save the (shared) pilot in a field
		this.LIGHT_AVERAGE = LIGHT_AVERAGE ;
		this.light = light ;
		this.level = levels;
		
    }

		
	//takes control when battery at a certain level 
	@Override
	public boolean takeControl() {
		light.fetchSample(levels, 0);
		return  level[0] > LIGHT_AVERAGE; 
    }
		
	//stops robot and turns off 
	@Override
	public void action() {
		boolean check = Battery.getVoltage() < 6.9f;
		while (check == true){
			LCD.drawString("Battery LOW!: " + Battery.getVoltage(), 0, 4);
			Delay.msDelay(5000);
			pilot.stop();
			System.exit(0);
		}
	}

	@Override
	public void suppress() {
			
	}
		
}
	public static class emergency_stop implements Behavior{
		//private private movepilot field
		private MovePilot p;

		//creates a shared field for movepilot
		emergency_stop(MovePilot p){
			this.p = p;
		}
		
		
	@Override
	public boolean takeControl() { //takes control when the button is pressed

			if(Button.ENTER.isDown()) 
			return true;
			return false;
	
    }

			
	@Override
	public void action() {
			p.stop();
			System.exit(0);
	
        }
		
	@Override
	public void suppress() {
			
	}
}
	// make robot exit the arbitrator if conditions are met e.g. battery low
		
public static class colour implements Behavior {
		//for red object
    	final static double RED_VALUE_R = 0.01; // >0.01 is red
		final static double RED_VALUE_Y = 0.01;//control value for taking control 
	    final static double GREEN_VALUE_Y = 0.009;//green 
		final static double BLUE_VALUE =  0.003 ;//Blue
		int counter = 0;	//counter instalized to 0 
		 private MovePilot p; private //shared feild for pilot 
		boolean control = false;	//a control for turning off the 
		private SampleProvider sp;

		public float[] level = new float[4];
		public colour(SampleProvider sp,MovePilot p) {
			this.sp = sp;
			this.p = p;
			sp.fetchSample(level, 0);
	
        }
	

	@Override
	public boolean takeControl() {
			sp.fetchSample(level,0);
			control = true;
			return( (level[0]>RED_VALUE_Y) ) ; //yellow colour red-y && green-y/ dosnt work on robot?
		
        }


	@Override
	public void action() {
		        //for loop to increment counter variable  

		for(int i=0; i<6; i++){ 

		//increment counter variable  
		if ( level[0]>RED_VALUE_R  && control == true) { //colour red detetcted
		    	LCD.drawString("Object detected", 0, 3);
		        Delay.msDelay(10000);
				LCD.clear();

		        //action
				p.rotate(LEFT);
			    counter+= 1;  
				control = false;
		}
		    
			 //for loop to increment counter variable  
			
		if( level[1]>GREEN_VALUE_Y  && control == true ) { //colour green detected
				LCD.drawString("Object detected", 0, 3);
				Delay.msDelay(10000);
				LCD.clear();	

                //action
                p.rotate(RIGHT);
			    counter+= 1;  
				control = false;
		
		}
		
		if (counter == 3) {
                // If 3 obticles are counted = halfway point
		    	LCD.drawString("You're halfway",0,3);  
				LCD.drawString("through the course:",0,4);
				Button.LEDPattern(1);
				Sound.twoBeeps();// sound used from sound librry
				Delay.msDelay(1000);
				LCD.clear()			; //screen cleared
	        
		}
			
		if(counter == 6) { //If 6 obsticles have been detected course = finished
			    LCD.drawString("You finished !",0,5);
				Button.LEDPattern(1);
				Sound.beepSequenceUp(); // sample sound from lejos library
				//victory
				Delay.msDelay(1000); //wait before resetting to home screen
				LCD.clear();
				System.exit(0); 
				p.stop(); // program stopped

		}
			       
        if( level[2]>BLUE_VALUE  && control == true ) { //blue colour detected
			    LCD.drawString("Object detected", 0, 3);
		        Delay.msDelay(10000);
				LCD.clear();
                //action
                
				control = false;
		}
     }
	}
			//switches control to false, robot drives forward triggering next behaviour to take control
			@Override
			public void suppress() {
				control = false;
				p.forward();
			}
		}

}
		
	


