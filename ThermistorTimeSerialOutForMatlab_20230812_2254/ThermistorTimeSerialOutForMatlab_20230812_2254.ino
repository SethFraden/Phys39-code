
// August 19, 2023
//Works!

// Reads voltage from voltage divider with thermistor as one leg. Averages numMeasurements times.
// Thermistor is in leg next to ground and the voltage is read by "tempIn"
// The thermistor is a Epcos, R/T 1008 with R_25 = ~2000 ohms. B-parameter equation given in code.
// Outputs temperature (C), elapsed time (s) and PWM (0-255)
// Example serial line output:
//              Temperature (C): 27.73, Time (s): 645.06, PWM: 127

// Works with Matlab app "testappV2"
// Matlab reads and plots the results in a stripchart
// Matlab sets pwmValue between [-255,255] and passes value to Arduino via Serial
// Matlab sets HeatCool as either 0 (cool) or 1 (heat) and passes value to Arduino via Serial


const int tempIn = A0;  // Analog input pin
const int numMeasurements = 5000;  // Number of measurements

// power
#define pwmOut1 9
#define pwmOut2 10


// Fixed resistor value in ohms
// const float fixedResistance = 4670.0; // my board
const float fixedResistance = 2000.0; // board in 39 lab

// Thermistor parameters for 100k ohm 
const float beta = 3560.0;
const float R25 = 2000.0;
const float T0 = 298.15;  // 25°C in Kelvin
int  pwmValue = 0;
int  HeatCool = 1;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  while (!Serial) {
    ;  // Wait for serial port to connect
  }
}
unsigned long startTime = millis();  // Get the starting time

void loop() {
  

  // Perform measurements and calculate average
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
   
// Matlab sends pwm and heat/cool information over the serial line
 if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if (command.startsWith("PWM")) {
      sscanf(command.c_str(), "PWM %d", &pwmValue);
    }

    if (command.startsWith("Heat/Cool")) {
      sscanf(command.c_str(), "Heat/Cool %d", &HeatCool);
    }
    if (HeatCool == 1){
      analogWrite(pwmOut1, abs(pwmValue));
      analogWrite(pwmOut2,0); 
    }
    if (HeatCool == 0){
      analogWrite(pwmOut2, abs(pwmValue));
      analogWrite(pwmOut1,0); 
    }
 
  }

  // Print average temperature and time in seconds
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0;
  Serial.print("Temperature (C): ");
  Serial.print(temperature);
  Serial.print(", Time (s): ");
  Serial.print(elapsedTime);
  Serial.print(", Heat/Cool: ");
  Serial.print(HeatCool);
  Serial.print(", PWM: ");
  Serial.println(pwmValue);
}