#include "BluetoothSerial.h" //Includes the header file that will be used to connect to the bluetooth/wireless terminal.
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//Blutooth object to connect to phone
BluetoothSerial SerialBT;

//FSR variables
int fsrPin = 37;     // the FSR and 10K pulldown are connected to pin 37.
int fsrReading;     // the analog reading from the FSR resistor divider.

//Accelerometer variables
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//Local HopTest variables
int attemptNumber = 0;
int Xorientation[120];
int Yorientation[120];
int Zorientation[120];
int arrayCounter = 0;
float timeTakenLocal;
float prevLSI;
float avgLSI = 0;
float hopTestAvgXAngle;
float hopTestAvgYAngle;
float hopTestAvgZAngle;
float injHopTestAvgXAngle;
float injHopTestAvgYAngle;
float injHopTestAvgZAngle;
  
class HopTest{
  //private variables/functions.
  private:
    //state variables that each Attempt object will have.
    int hopTestNo;
    int timeTaken;
    int* orientationX;
    int* orientationY;
    int* orientationZ;

  //public variables/functions that can be called from outside the class (the API).
  public:
    //generic constructor, takes in no arguments and does nothing but create a new Attempt object.
    HopTest(){};

    //a modified constructor that takes in each of the state variables as arguments and assigns them accordingly.
    HopTest(int hopTestNo, int timeTaken, int orientationX[], int orientaionY[], int orientationZ[]){
      //this->date = date;
      this->hopTestNo = hopTestNo;
      this->timeTaken = timeTaken;
      this->orientationX = orientationX;
      this->orientationY = orientationY;
      this->orientationZ = orientationZ;
      };

    //a getter method to retrieve the hop test number.
    int getHopTesttNo(){
      return this->hopTestNo;
      };

    //a getter method to retrieve the time taken to complete this hop test.
    int getTimeTaken(){
      return this->timeTaken;
      }

    //a getter method to retrieve the array of X orientations
    int *getOrientationX(){
      return this->orientationX;
      }

    //a getter method to retrieve the array of Y orientations
    int *getOrientationY(){
      return this->orientationY;
      }

    //a getter method to retrieve the array of Z orientations
    int *getOrientationZ(){
      return orientationZ;
      }
  };

class Attempt{
  //private variables/functions.
  private:
    HopTest injured;
    HopTest normal;
    float LSI;
  //public variables/functions that can be called from outside the class (the API).
  public:
    //default contructor
    Attempt(){};

    //official contructor, initializes state variables
    Attempt(HopTest injured, HopTest normal){
      this->injured = injured;
      this->normal = normal;
      this->LSI = (float(normal.getTimeTaken())/float(injured.getTimeTaken()))*100.0;
      };

    //getter method to retrieve the injured Hop Test from the Attempt object
    HopTest getInjuredTest(){
      return this->injured;
      };

    //getter method to retrieve the normal Hop Test from the Attempt object
    HopTest getNormalTest(){
      return this->normal;
      };

    //getter method to retrieve the LSI of the attempt
    float getLSI(){
      return this->LSI;
      };
  };

//HopTest variables used in the loop() section
HopTest injuredHopTest, uninjuredHopTest;
  
void setup() {
  Serial.begin(115200); //initializes serial monitor

  //Setting up bluetooth
  if(!SerialBT.begin("SA(c)LT")){
    Serial.println("An error occurred initializing Bluetooth");
  }else{
    Serial.println("Bluetooth initialized");
  }
  //checking if the BNO055 is connected successfully
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"); //Cannot connect to BNO055
    while(1); //Endless loop forcing the user to restart their code.
  }    
}


void loop() {

  //reset appropriate values to 0 depending if this is a normal or injured leg Hop Test
  if (attemptNumber%2 == 0){
    hopTestAvgXAngle = 0.0;
    hopTestAvgYAngle = 0.0;
    hopTestAvgZAngle = 0.0;
    }
  else{
    injHopTestAvgXAngle = 0.0;
    injHopTestAvgYAngle = 0.0;
    injHopTestAvgZAngle = 0.0;
  }

  //declares and initializes additional variables required
  timeTakenLocal=0;
  Attempt attempt;
  arrayCounter = 0;
  sensors_event_t event; //Set up a new sensor event.
  bno.getEvent(&event);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Set the variable 'euler' to be a Vector object which holds the euler angles.
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //Sets the variable 'linAccel' to be a Vector object which hold the linear acceleration values.

  fsrReading = analogRead(fsrPin); //takes analog reading of the FSR pin

  SerialBT.println("Enter 's' to start a new Hop Test.");
  while(SerialBT.available() == 0){};
  char startingTrigger = SerialBT.read();
  if (startingTrigger == 's'){
    Serial.println(startingTrigger);
    SerialBT.println("Lift the leg you will not be hopping with to begin.");
    while(fsrReading > 1000){
      fsrReading = analogRead(fsrPin);
      }

    SerialBT.print("Ready to begin HopTest...I will start the counter as soon as the hopping starts!\n");

    while(abs(linAccel.z()) < 15){
      linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //Sets the variable 'linAccel' to be a Vector object which hold the linear acceleration values.
      delay(100);
      }

    fsrReading = analogRead(fsrPin);
    SerialBT.println("Hop Test recording has started.");
    //determining when the users puts their other foot down
    while (fsrReading < 1000 && arrayCounter <= 119){
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      Xorientation[arrayCounter] = euler.x();
      Yorientation[arrayCounter] = euler.y();
      Zorientation[arrayCounter] = euler.z();
      arrayCounter++;
      if (attemptNumber%2 == 0){
        hopTestAvgXAngle += float(euler.x());
        hopTestAvgYAngle += float(euler.y());
        hopTestAvgZAngle += float(euler.z());
      }
      else{
        injHopTestAvgXAngle += float(euler.x());
        injHopTestAvgYAngle += float(euler.y());
        injHopTestAvgZAngle += float(euler.z());
        }
      delay(500);
      fsrReading = analogRead(fsrPin);
      }

    //calculates the average angles to be displayed later
    if (attemptNumber%2 == 0){
      hopTestAvgXAngle /= float(arrayCounter+1);
      hopTestAvgYAngle /= float(arrayCounter+1);
      hopTestAvgZAngle /= float(arrayCounter+1);
      }
    else{
      injHopTestAvgXAngle /= float(arrayCounter+1);
      injHopTestAvgYAngle /= float(arrayCounter+1);
      injHopTestAvgZAngle /= float(arrayCounter+1);
      }
    
    timeTakenLocal = float(arrayCounter+1.00)*0.50; //calculates the taken depending on the number of acceleration readings taken
    SerialBT.println("Hop Test ended. Indicate if this was a passed Hop Test by entering 'p'. Enter anything else to indicate a failed Hop Test...");
    while(SerialBT.available() == 0){};
    char success = SerialBT.read();

    //Records the hop test if this a passed hop test
    if(success == 'p'){
      attemptNumber++;

      //determines if it was an injured or normal leg hop test
      if (attemptNumber%2 == 1){
        uninjuredHopTest = HopTest(attemptNumber, timeTakenLocal, Xorientation, Yorientation, Zorientation);
        SerialBT.println("Uninjured leg Hop Test recorded.");
        }
      else{
        injuredHopTest = HopTest(attemptNumber, timeTakenLocal, Xorientation, Yorientation, Zorientation);
        SerialBT.println("Injured leg Hop Test recorded.");
        attempt = Attempt(injuredHopTest, uninjuredHopTest);
        if (attemptNumber >=4)
          avgLSI = ((avgLSI*((attemptNumber/2)-1))+attempt.getLSI())/((attemptNumber/2));
        else
          avgLSI = attempt.getLSI();

        //if there was a previous attempt, compare the previous LSI to the current LSI
        if (attemptNumber > 2){
          if (attempt.getLSI() > prevLSI){
            SerialBT.print("Your LSI has improved from ");
            SerialBT.print(prevLSI);
            SerialBT.print(" to ");
            SerialBT.print(attempt.getLSI());
            SerialBT.println(". Great work!");
            } 
          else if (attempt.getLSI() < prevLSI){
            SerialBT.print("Your LSI has gone down from ");
            SerialBT.print(prevLSI);
            SerialBT.print(" to ");
            SerialBT.print(attempt.getLSI());
            SerialBT.println(". You can do this!");
            }
          else SerialBT.println("Your LSI did not change. Strive for improvement!");
          }
        prevLSI = attempt.getLSI(); //save previous LSI

        //Display the results for the user
        SerialBT.println("___________________________________________________");
        SerialBT.println("_________________Attempt Summary________________");
        SerialBT.print("Uninjured leg time: ");
        SerialBT.println(float(uninjuredHopTest.getTimeTaken()));
        SerialBT.print("Injured leg time: ");
        SerialBT.println(float(injuredHopTest.getTimeTaken()));
        SerialBT.print(" This attempt's LSI is:   ");
        SerialBT.println(attempt.getLSI());
        SerialBT.print(" Your average LSI is:     ");
        SerialBT.println(avgLSI);
        SerialBT.print(" Normal leg Avg. X Angle: ");
        SerialBT.println(hopTestAvgXAngle);
        SerialBT.print(" Normal leg Avg. Y Angle: ");
        SerialBT.println(hopTestAvgYAngle);
        SerialBT.print(" Normal leg Avg. Z Angle: ");
        SerialBT.println(hopTestAvgZAngle);
        SerialBT.print("Injured leg Avg. X Angle: ");
        SerialBT.println(injHopTestAvgXAngle);
        SerialBT.print("Injured leg Avg. Y Angle: ");
        SerialBT.println(injHopTestAvgYAngle);
        SerialBT.print("Injured leg Avg. Z Angle: ");
        SerialBT.println(injHopTestAvgZAngle);
        SerialBT.println("___________________________________________________");
      }
    }
    //allow the user to record another hop test is this one was failed.
    else{
      SerialBT.println("Failed Hop Test not recorded.");
      }
    }
  }
