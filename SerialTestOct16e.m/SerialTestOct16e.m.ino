unsigned long startTime = 0;
String receivedString;

void setup() {
  Serial.begin(9600);
  startTime = millis(); // Record the start time
}

void loop() {
  unsigned long currentTime = millis();
  float elapsedSeconds = (currentTime - startTime) / 1000.0;

  if (Serial.available() > 0) {
    receivedString = Serial.readStringUntil('\n'); // Read until a newline character is encountered

    // Initialize variables
    String command;
    String mode;
    int pwm;
    float Tset;

    // Use a String tokenizer to split the received string
    int spaceIndex = receivedString.indexOf(' ');
    if (spaceIndex > 0) {
      command = receivedString.substring(0, spaceIndex);
      receivedString = receivedString.substring(spaceIndex + 1);

      spaceIndex = receivedString.indexOf(' ');
      if (spaceIndex > 0) {
        pwm = receivedString.substring(0, spaceIndex).toInt();
        Tset = receivedString.substring(spaceIndex + 1).toFloat();

      //   // Print the parsed values on one line with commas
      //   Serial.print("Received Command: ");
      //   Serial.print(command);
      //   Serial.print(", Received PWM: ");
      //   Serial.print(pwm);
      //   Serial.print(", Received Tset: ");
      //   Serial.print(Tset);

      //   // Send the elapsed time back to Matlab on the same line
      //   Serial.print(", Time = ");
      //   Serial.print(elapsedSeconds, 1); // Print elapsed time with 1 decimal place
      //   Serial.println(" seconds");
      // } else {
      //   Serial.println("Parsing Error: Invalid format (no third value).");
      // }
    } else {
      Serial.println("Parsing Error: Invalid format (no second value).");
    }
    }
  // Check the command and print the corresponding message
      if (strcmp(command.c_str(), "Manual") == 0) {
     
        mode = command.c_str();
        Serial.print("Mode: ");
        Serial.print(mode);
        Serial.print(", PWM: ");
        Serial.print(pwm);
        Serial.print(", Tset: ");
        Serial.print(Tset);
        Serial.print(", Time = ");
        Serial.print(elapsedSeconds, 1); // Print elapsed time with 1 decimal place
        Serial.println(" seconds");
      } 
        else if (strcmp(command.c_str(), "Proportional") == 0) {
        mode = command.c_str();
        Serial.print("Mode: ");
        Serial.print(mode);
        Serial.print(", PWM: ");
        Serial.print(pwm);
        Serial.print(", Tset: ");
        Serial.print(Tset);
        Serial.print(", Time = ");
        Serial.print(elapsedSeconds, 1); // Print elapsed time with 1 decimal place
        Serial.println(" seconds");
      } else {
        Serial.print("Received Command: ");
        Serial.println(command);
      }
  }
}
