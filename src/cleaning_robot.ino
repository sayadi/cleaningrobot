#include <Wire.h>
#include <LIDARLite.h>
#include <SimpleTimer.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

LIDARLite lidarLite;

ros::NodeHandle nodeHandle;

std_msgs::Int16 laserRange;
std_msgs::Int16 laserIntensity;

#define stepsPerRevolution 200
#define rightStepperDirectionPin 2
#define rightStepperStepPin 3
#define leftStepperDirectionPin 5
#define leftStepperStepPin 4
#define readingsNo 10

float rightStepperSpeed; // in m/s
float leftStepperSpeed; // in m/s
float rightStepperRPM;
short rightStepperDelay;
std_msgs::Int16 rightStepperDelaySTD;
short leftStepperDelay;
std_msgs::Int16 leftStepperDelaySTD;
float leftStepperRPM;
const float distanceBetweenTwoWheels = 0.35; // in m
const float wheelsRaduis = 0.039; // in m

SimpleTimer rightStepperTimer;
SimpleTimer leftStepperTimer;

std_msgs::Int16 batteryLevelSTD;
std_msgs::Int16 waterStatusSTD;
std_msgs::Int16 dustLevelSTD;

void rightStepperCallback()
{
  if(digitalRead(rightStepperStepPin) == HIGH)
  {
    digitalWrite(rightStepperStepPin, LOW);
    
  } // end if

  else // digitalRead(rightStepperStepPin) == LOW
  {
    digitalWrite(rightStepperStepPin, HIGH);
    
  } // end else

  nodeHandle.spinOnce();
  
} // end function rightStepperCallback

void leftStepperCallback()
{
  if(digitalRead(leftStepperStepPin) == HIGH)
  {
    digitalWrite(leftStepperStepPin, LOW);
    
  } // end if

  else // digitalRead(leftStepperStepPin) == LOW
  {
    digitalWrite(leftStepperStepPin, HIGH);
    
  } // end else
  
  nodeHandle.spinOnce();
  
} // end function leftStepperCallback

void baseControllerCallback (const geometry_msgs::Twist &twist)
{

    nodeHandle.spinOnce();
    rightStepperSpeed = (twist.angular.z * M_PI * distanceBetweenTwoWheels / 2) + twist.linear.x; // this is in m/s
    leftStepperSpeed =  (twist.linear.x * 2) - rightStepperSpeed;

    rightStepperRPM = rightStepperSpeed / (M_PI * wheelsRaduis);
    leftStepperRPM = leftStepperSpeed / (M_PI * wheelsRaduis);

    // calculate the delay for the rpm
    rightStepperDelay = abs( 600 / (4 * rightStepperRPM) );
    leftStepperDelay =  abs( 600 / (4 * leftStepperRPM) );

    // Set the directions
    if(rightStepperSpeed < 0)
    {
      digitalWrite(rightStepperDirectionPin, LOW);

    } // end if
    else // rightStepperSpeed >= 0
    {
      digitalWrite(rightStepperDirectionPin, HIGH);

    } // end else

    if(leftStepperSpeed < 0)
    {
      digitalWrite(leftStepperDirectionPin, LOW);

    } // end if

    else // leftStepperSpeed >= 0
    {
      digitalWrite(leftStepperDirectionPin, HIGH);

    } // end else


    // Move the motors
    rightStepperTimer.setInterval(rightStepperDelay, rightStepperCallback);
    leftStepperTimer.setInterval(leftStepperDelay, leftStepperCallback);

    rightStepperTimer.run();
    leftStepperTimer.run();

    nodeHandle.spinOnce();


} // end function baseControllerCallback

ros::Subscriber<geometry_msgs::Twist> cdm_velSubscriber("cmd_vel", &baseControllerCallback);

ros::Publisher rightStepperDelayPublisher("rightSteperDelayTopic", &rightStepperDelaySTD);
ros::Publisher leftStepperDelayPublisher("leftStepperDelayTopic", &leftStepperDelaySTD);

ros::Publisher laserRangePublisher("laserRangeTopic", &laserRange);
ros::Publisher laserIntensityPublisher("laserIntensityTopic", &laserIntensity);

ros::Publisher batteryLevelPublisher("batteryLevelTopic", &batteryLevelSTD);
ros::Publisher waterStatusPublisher("waterStatusTopic", &waterStatusSTD);
ros::Publisher dustLevelPublisher("dustLevelTopic", &dustLevelSTD);

void setup()
{
  nodeHandle.initNode();
  nodeHandle.subscribe(cdm_velSubscriber);

  // todo: comment these out
  rightStepperDelaySTD.data = -1;
  leftStepperDelaySTD.data = -1;
  
  // todo: comment these out
  batteryLevelSTD.data = 30;
  waterStatusSTD.data = 0;
  dustLevelSTD.data = 30;
  
  nodeHandle.advertise(rightStepperDelayPublisher);
  nodeHandle.advertise(leftStepperDelayPublisher);

  nodeHandle.advertise(laserRangePublisher);
  nodeHandle.advertise(laserIntensityPublisher);
  
  nodeHandle.advertise(batteryLevelPublisher);
  nodeHandle.advertise(waterStatusPublisher);
  nodeHandle.advertise(dustLevelPublisher);
  
  lidarLite.begin();

}

void loop()
{
  nodeHandle.spinOnce();
  
  laserRange.data = lidarLite.distance();
  laserIntensity.data = lidarLite.signalStrength();
  nodeHandle.spinOnce();
    


  laserRangePublisher.publish(&laserRange);
  laserIntensityPublisher.publish(&laserIntensity);
  nodeHandle.spinOnce();


  rightStepperDelaySTD.data = rightStepperDelay;
  leftStepperDelaySTD.data = leftStepperDelay;
  nodeHandle.spinOnce();
  
  rightStepperDelayPublisher.publish(&rightStepperDelaySTD);
  leftStepperDelayPublisher.publish(&leftStepperDelaySTD);
  
  batteryLevelPublisher.publish(&batteryLevelSTD);
  waterStatusPublisher.publish(&waterStatusSTD);
  dustLevelPublisher.publish(&dustLevelSTD);

  nodeHandle.spinOnce();  
  delay(1000-12);

}
