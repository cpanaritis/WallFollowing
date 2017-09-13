package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	if(distance != 21474) {
		this.distance = distance;
	}
	int error = this.distance-bandCenter; //error between the actual error and the center
	//Maintain speed if the error is within the allowed error.
	if(Math.abs(error) <= bandwidth) {
    		WallFollowingLab.leftMotor.setSpeed(motorHigh);
    		WallFollowingLab.rightMotor.setSpeed(motorHigh);
   		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
	}
    //Increase the outside wheel speed and decrease the inside wheel speed if the robot is too far from the wall.
    else if(error > 0) {    
    		WallFollowingLab.leftMotor.setSpeed(motorLow);
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
    }
    //Do the opposite if the robot is too close to the wall.
    else if(error < 0 && error>=-13 ) {  
    		WallFollowingLab.leftMotor.setSpeed(motorHigh);
		WallFollowingLab.rightMotor.setSpeed(motorLow);
		WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
    }
    else if(error < -13) {
		WallFollowingLab.leftMotor.setSpeed(motorHigh*6);
		WallFollowingLab.rightMotor.setSpeed(50);
		WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
	}
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
  
  /*int calcCorrection (int diff) {
	  int correction; 
	  if (diff<0) {
		  diff= Math.abs(diff);
	  }
	  correction = (int)(proportionConstant*(double)(diff));
	  if(correction >= motorHigh) {
		  correction = motorLow;
	  }
	  return correction; 
  }*/
}
