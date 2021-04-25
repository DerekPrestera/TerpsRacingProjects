/*
 * Terps Racing Telemetry Simulator
 * 
 * This program outputs a simple stream of data to the serial port
 * to test the Telemetry Viewer program that reads data from the
 * computer's serial port and plots the data as it comes in.
 */

void setup() {
  // Start the serial port with the highest baud rate
  Serial.begin(115200);
}

void loop() {
  // Create a sawtooth function and print the values to the
  //    serial port every 0.1 seconds.
  static int time_stamp = 0;  // simulated time stamp
  static int data_point = 0;  // simulated data point
  static int modifier = 25;

  static char buf[10]; // Buffer 

  /*
   * I transmit everything in groups of 8 bytes so that the serial
   * receive buffer on the recieving computer doesn't get part of
   * a message, fill up, and dump only part of a message into the
   * serial read function.
   */

  // Send the name of the data, the time stamp, and the data
  sprintf(buf, "testwave");
  Serial.write(buf, 8);
  
  // Send the x value
  sprintf(buf, "%8d", time_stamp);
  Serial.write(buf, 8);

  // Send the y value
  sprintf(buf, "%8d", data_point);
  Serial.write(buf, 8);

  // Send the "end of transmisison" message
  sprintf(buf, "end_msg\n");
  Serial.write(buf, 8);

  // Increment the values
  time_stamp += 1;
  data_point += modifier;
  if (data_point >= 100 || data_point <= -100) modifier = -modifier;
  delay(100);
}
