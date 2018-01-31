char address = 0x01;    // UART address as 0x01

void setup() {
  Serial.begin(9600);   // Start serial communication
  Serial3.begin(9600);
  Serial.flush();   // Clear serial buffer

}

void loop() {

  char command, data;

  // Clear internal junction count of LSA08
  command = 'X';
  data = 0x00;
  sendCommand(command,data);
  delay(1000);

  // Setting LCD contrast to 90
  command = 'S';
  data = 90;
  sendCommand(command,data);
  delay(1000);
  
  // Setting LCD backlight to level 5
  command = 'B';
  data = 0x05;
  sendCommand(command,data);
  delay(1000);
  
  // Setting junction width to 6
  command = 'J';
  data = 0x06;
  sendCommand(command,data);
  delay(1000);
  
  // Setting threshold value to 5
  command = 'T';
  data = 0x05;
  sendCommand(command,data);
  delay(1000);
  
  // Setting line mode to Dark-On
  command = 'L';
  data = 0x01;
  sendCommand(command,data);
  delay(1000);
  
  // Setting UART ouput to mode 1
  command = 'D';
  data = 0x01;
  sendCommand(command,data);
  delay(1000);
  
  // Start calibration
  command = 'C';
  data = 0x00;
  sendCommand(command,data);
  delay(1000);
  Serial.print("complete");
  while(1);   // Stay here to prevent infinite loop
  
}

/*
 * Function to send command to LSA08 in 4 continuous bytes.
 * LSA08 will reply with "OK" for every successful command sent.
 * However, reading the reply is optional, and thus not showing here.
 */
void sendCommand(char command, char data) {
  
  char checksum = address + command + data;
  
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(data);
  Serial3.write(checksum);

}

