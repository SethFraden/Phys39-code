// Works! August 19, 2023
// All controls for heating and cooling are from Arduino. Matlab displays stripchart.
// Reads voltage into A0 from voltage divider with thermistor as one leg. Averages numMeasurements times.
// Reads voltage into A1 from variable resistor voltage divider to set pwm.
// Reads Heat or Cool command from Arduino pin 11.
// Outputs pwm to pair of pwm pins 9, 10 to heat or cool.

// Thermistor is in leg next to ground and the voltage is read by "tempIn" (pin A0)
// The thermistor is a Epcos, R/T 1008 with R_25 = 2000 ohms. B-parameter equation given in code.
// Outputs temperature (C) and elapsed time (s)
// Example serial line output:
//              Temperature (C): 27.73, Time (s): 645.06

// Matlab reads and plots the results in a stripchart: StripChartTempTimeV3.mlx




const int tempIn = A0;  // Analog input pin from thermistor voltage divider 
const int pwmIn = A1;  // Analog input pin from variable resistor for pwm signal
const int numMeasurements = 10000;  // Number of measurements

// power
#define pwmOut1 9
#define pwmOut2 10
#define direction 11


// Fixed resistor value in ohms
//const float fixedResistance = 4670.0; my board
const float fixedResistance = 2195.0; // board in 39 lab

// Thermistor parameters
const float beta = 3560.0;
const float R25 = 2000.0;
const float T0 = 298.15;  // 25°C in Kelvin

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  while (!Serial) {
    ;  // Wait for serial port to connect
  }
  pinMode(direction, INPUT); // Set digital pin 11 as input
}
unsigned long startTime = millis();  // Get the starting time

void loop() {


  // Perform temperature measurements and calculate average
float total = 0;
  for (int i = 0; i < numMeasurements; i++) {
    int sensorValue = analogRead(tempIn);
    total = total + sensorValue;
  }
float average = total / (float)numMeasurements;

    // Convert the analog value to voltage
    float voltage = average * (5.0 / 1023.0);  // Assuming 5V reference voltage

    // Calculate the resistance of the thermistor
    float thermistorResistance = (fixedResistance * voltage) / (5.0 - voltage);

    // Calculate the temperature using the Steinhart-Hart equation. See Wiki Thermistor entry.
    float steinhart = log(thermistorResistance / R25);
    steinhart /= beta;
    steinhart += 1.0 / T0;
    float temperature = 1.0 / steinhart - 273.15;  // temperature in Centigrade
    
    int pwmAnalogInput = analogRead(pwmIn)/4;  // Voltage divider input. Analog input is 10 bit, but pwm output is 8 bit so divide by 4
   
  // Check the input on digital pin "direction"
int HeatCool = digitalRead(direction); 
  if (HeatCool == 1) { // Heat
    analogWrite(pwmOut1,pwmAnalogInput);
    analogWrite(pwmOut2,0);  //  heating 
  } 
  else {  // Cool
      analogWrite(pwmOut1,0);
      analogWrite(pwmOut2,pwmAnalogInput);  // cooling 
    }
    
    // Print average temperature and time in seconds
 unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0;
  Serial.print("Temperature (C): ");
  Serial.print(temperature);
  Serial.print(", Time (s): ");
  Serial.print(elapsedTime);
  Serial.print(", PWM: ");
  Serial.print(pwmAnalogInput);
  Serial.print(", Heat/Cool: ");
  Serial.println(HeatCool);
}
    
    