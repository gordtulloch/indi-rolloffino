 /*
 * A controller example for roof motor controller from the INDI rolloffino roof driver.   
 * 
 * tg August 2018  Original
 * tg February 2020 Generalize to make less installation specific
 *                  Add unspecified usage AUX switch and relay
 *                  Communication protocol in terms of function not switches/relays
 *                  ~ 15KB 2% of Due, 50% of Nano
 * tg November 2021 Break out commandReceived and requestReceived to make alernate actions more 
 *                  obvious/accessible, Remove Due specific code.
 * gt Sept 30 2024  Forked code to create rolloff-nano)
 */

#define BAUD_RATE 38400

# define OPEN_CONTACT HIGH    // Switch definition, Change to LOW if pull-down resistors are used.

// Define name to pin assignments
#define SWITCH_1 2
#define SWITCH_2 3
#define SWITCH_3 4
#define SWITCH_4 5

#define ROOFRELAY 0


// Indirection to define a functional name in terms of a switch
// Use 0 if switch not implemented
#define SWITCH_OPENED  SWITCH_1  // Fully opened is assigned to switch 1
#define SWITCH_CLOSED  SWITCH_2  // Fully closed is assigned to switch 2
#define SWITCH_RAPARK  SWITCH_3  // RA is Parked
#define SWITCH_DECPARK SWITCH_4  // Dec is Parked

// Indirection to define a functional name in terms of a relay
#define FUNC_OPEN  ROOFRELAY
#define FUNC_CLOSE ROOFRELAY
/*
 * For the relay that the function is mapped to indicate if that relay is to be momentarily closed 
 * or held in a closed position. 
 * If for example the relay is intended to provide power to a motor the need might be to 
 * keep it closed until a relay off is received. 
 * If a function such as OPEN is held on, it will be up to local code in the Arduino to determine 
 * when and how to turn it off. The host sends open and close requests but expects the roof to 
 * brings its self to a stop when complete. 
 * 
 * If HOLD is 0 then the relay will be activated for RELAY_DELAY milliseconds, then released.
 * If the relay is simulating a button push/release to the motor controller HOLD would be 0.
*/
#define FUNC_OPEN_HOLD 0
#define FUNC_CLOSE_HOLD 0

#define RELAY_PRIOR_DELAY 50
#define RELAY_ON_DELAY 500
#define RELAY_POST_DELAY 50

/*
 * Abort (stop) request is only meaningful if roof is in motion.
 *
 * On Abort for a single button controller, only want to activate relay and pulse the controller if
 * the roof is still moving, then it would stop. If it has already reached the end of its
 * travel, a pulse could set it off again in the opposite direction.
 *
 * In case the end of run switches are not reached, some way to know if it is moving
 * would be helpful. Short of that estimate how long it takes the roof to open or close
 */
#define ROOF_OPEN_MILLI 15000       

// Buffer limits
#define MAX_INPUT 45
#define MAX_RESPONSE 127
#define MAX_MESSAGE 63

enum cmd_input {
CMD_NONE,  
CMD_OPEN,
CMD_CLOSE,
CMD_STOP
} command_input;

unsigned long timeMove = 0;
const int cLen = 15;
const int tLen = 15;
const int vLen = MAX_RESPONSE;
char command[cLen+1];
char target[tLen+1];
char value[vLen+1];

//  Maximum length of messages = 63                                               *|
const char* ERROR1 = "The controller response message was too long";
const char* ERROR2 = "The controller failure message was too long";
const char* ERROR3 = "Command input request is too long";
const char* ERROR4 = "Invalid command syntax, both start and end tokens missing"; 
const char* ERROR5 = "Invalid command syntax, no start token found";
const char* ERROR6 = "Invalid command syntax, no end token found";
const char* ERROR7 = "Roof controller unable to parse command";
const char* ERROR8 = "Command must map to either set a relay or get a switch";
const char* ERROR9 = "Request not implemented in controller";
const char* ERROR10 = "Abort command ignored, roof already stationary";

const char* VERSION_ID = "V0.1GT";

void sendAck(char* val)
{
  char response [MAX_RESPONSE];
  if (strlen(val) > MAX_MESSAGE)
    sendNak(ERROR1);
  else
  {  
    strcpy(response, "(ACK:");
    strcat(response, target);
    strcat(response, ":");
    strcat(response, val);
    strcat(response, ")");
    Serial.println(response);
    Serial.flush();
  }
}

void sendNak(const char* errorMsg)
{
  char buffer[MAX_RESPONSE];
  if (strlen(errorMsg) > MAX_MESSAGE)
    sendNak(ERROR2);
  else
  {
    strcpy(buffer, "(NAK:ERROR:");
    strcat(buffer, value);
    strcat(buffer, ":");
    strcat(buffer, errorMsg);
    strcat(buffer, ")");
    Serial.println(buffer);
    Serial.flush();
  }
}

void setRelay(int id, int hold, char* value)
{
  if (strcmp(value, "ON") == 0)
  {
    digitalWrite(id, HIGH);            // NO RELAY would normally already be in this condition (open)
    delay(RELAY_PRIOR_DELAY);
    digitalWrite(id, LOW);           // Activate the NO relay (close it) 
    if (hold == 0)
    {  
      delay(RELAY_ON_DELAY);
      digitalWrite(id, HIGH);          // Turn NO relay off
    }
  }
  else
  {
    digitalWrite(id, HIGH);            // Turn NO relay off 
  }
  delay(RELAY_POST_DELAY);
} 

/*
 * Get switch value
 * Expect a NO switch configured with a pull up resistor.
 * NO switch: Inactive HIGH input to the pin with pull up resistor, logical 0 input. 
 * When switch closes The LOW voltage logical 1 is applied to the input pin. 
 * The off or on value is to be sent to the host in the ACK response
 */
void getSwitch(int id, char* value)
{
  if (digitalRead(id) == OPEN_CONTACT)
    strcpy(value, "OFF");
  else
    strcpy(value, "ON");  
}

bool isSwitchOn(int id)
{
  char switch_value[16+1];
  getSwitch(id, switch_value);
  if (strcmp(switch_value, "ON") == 0)
  {
    return true;
  }   
  return false;
}

bool parseCommand()           // (command:target:value)
{
  bool start = false;
  bool eof = false;
  int recv_count = 0;
  int wait = 0;
  int offset = 0;
  char startToken = '(';
  char endToken = ')';
  const int bLen = MAX_INPUT;
  char inpBuf[bLen+1];

  memset(inpBuf, 0, sizeof(inpBuf));
  memset(command, 0, sizeof(command));
  memset(target, 0, sizeof(target));
  memset(value, 0, sizeof(value));
    
  while (!eof && (wait < 20))
  {
    if (Serial.available() > 0)
    {
      Serial.setTimeout(1000);
      recv_count = Serial.readBytes((inpBuf + offset), 1);
      if (recv_count == 1)
      {
        offset++;
        if (offset >= MAX_INPUT)
        {
          sendNak(ERROR3);
          return false;        
        }
        if (inpBuf[offset-1] == startToken)
        {
          start = true;  
        }
        if (inpBuf[offset-1] == endToken) 
        {
          eof = true;
          inpBuf[offset] = '\0';           
        }
        continue;
      }
    }
    wait++;
    delay(100);
  }
    
  if (!start || !eof)
  {
    if (!start && !eof)  
      sendNak(ERROR4);
    else if (!start)
      sendNak(ERROR5);
    else if (!eof)
      sendNak(ERROR6);
    return false;
  }
  else
  {
    strcpy(command, strtok(inpBuf,"(:"));
    strcpy(target, strtok(NULL,":"));
    strcpy(value, strtok(NULL,")"));
    if ((strlen(command) >= 3) && (strlen(target) >= 1) && (strlen(value) >= 1))
    {
      return true;
    }
    else
    {  
      sendNak(ERROR7); 
      return false;
    }
  }              
}

/*
 * Use the parseCommand routine to decode message
 * Determine associated action in the message. Resolve the relay or switch associated 
 * pin with the target identity. Acknowledge any initial connection request. Return 
 * negative acknowledgement with message for any errors found.  Dispatch to commandReceived
 * or requestReceived routines to activate the command or get the requested switch state
 */
void readUSB()
{
  // Confirm there is input available, read and parse it.
  if (Serial && (Serial.available() > 0))
  {
    if (parseCommand())
    {
      unsigned long timeNow = millis();
      int hold = 0;
      int relay = -1;   // -1 = not found, 0 = not implemented, pin number = supported
      int sw = -1;      //      "                 "                    "
      bool connecting = false;
      const char* error = ERROR8;

      // On initial connection return the version
      if (strcmp(command, "CON") == 0)
      {
        connecting = true; 
        strcpy(value, VERSION_ID);  // Can be seen on host to confirm what is running       
        sendAck(value);
      }

      // Map the general input command term to the local action
      // SET: OPEN, CLOSE
      else if (strcmp(command, "SET") == 0)
      {
        // Prepare to OPEN
        if (strcmp(target, "OPEN") == 0)                     
        {
          command_input = CMD_OPEN;
          relay = FUNC_OPEN;
          hold = FUNC_OPEN_HOLD;
          timeMove = timeNow;
        }
        // Prepare to CLOSE
        else if (strcmp(target, "CLOSE") == 0)    
        { 
          command_input = CMD_CLOSE;
          relay = FUNC_CLOSE;
          hold = FUNC_CLOSE_HOLD;
          timeMove = timeNow;
        }
      }

      // Handle requests to obtain the status of switches   
      // GET: OPENED, CLOSED, LOCKED, AUXSTATE
      else if (strcmp(command, "GET") == 0)
      {
        if (strcmp(target, "OPENED") == 0)
          sw = SWITCH_OPENED;
        else if (strcmp(target, "CLOSED") == 0) 
          sw = SWITCH_CLOSED;
        else if (strcmp(target, "RAPARK") == 0) 
          sw = SWITCH_RAPARK;
        else if (strcmp(target, "DECPARK") == 0) 
          sw = SWITCH_DECPARK;
      }
    
      /*
       * See if there was a valid command or request 
       */
      if (!connecting)
      {
        if ((relay == -1) && (sw == -1))
        {
          sendNak(error);               // Unknown input or Abort command was rejected
        }

        // Command or Request not implemented
        else if ((relay == 0 || relay == -1) && (sw == 0 || sw == -1))
        {
          strcpy(value, "OFF");          // Request Not implemented
          //sendNak(ERROR9);   
          sendAck(value);
        }

        // Valid input received
        
        // A command was received
        // Set the relay associated with the command and send acknowlege to host
        else if (relay > 0)            // Set Relay response
        {
          commandReceived(relay, hold, value);
        }
        
        // A state request was received
        else if (sw > 0)               // Get switch response
        {
          requestReceived(sw);    
        }
      } // end !connecting
    }   // end command parsed
  }     // end Serial input found  
}


////////////////////////////////////////////////////////////////////////////////
// Action command received

// Here after pin associations resolved and request action known
// Default action is to set the associated relay to the requested state "ON" or "OFF" and
// send acknowledgement to the host. 
// target is the name associated with the relay "OPEN", "CLOSE", "STOP", "LOCK", "AUXSET".
// It will be used when  sending the acknowledgement to the host. Find out if a particular 
// command is being processed using if (strcmp(target, "OPEN") == 0) {do something}
//
// relay: pin id of the relay 
// hold:  whether relay is to be set permanently =0, or temporarily =1
// value: How to set the relay "ON" or "OFF" 
// 
//
void commandReceived(int relay, int hold, char* value)
{
  setRelay(relay, hold, value);
  sendAck(value);         // Send acknowledgement that relay pin associated with "target" was activated to value requested
}

////////////////////////////////////////////////////////////////////////////////

// Here after pin associations resolved and request action known
// Request to obtain state of a switch
// Default action is to read the associated switch and return result to the host
// target is the name associated with the switch "OPENED", "CLOSED" etc and will
// be used when sending the acknowledgement to the host. Find out if a certain request is being processed using
// if (strcmp(target, "OPENED") == 0) {do something}
//
// sw:     The switch's pin identifier.
// value   getSwitch will read the pin and set this to "ON" or "OFF" 
void requestReceived(int sw)
{
  getSwitch(sw, value);
  sendAck(value);            // Send result of reading pin associated with "target" 
}



// One time initialization
void setup() 
{
  // Initialize the input switches
  pinMode(SWITCH_1, INPUT_PULLUP); 
  pinMode(SWITCH_2, INPUT_PULLUP); 
  pinMode(SWITCH_3, INPUT_PULLUP); 
  pinMode(SWITCH_4, INPUT_PULLUP);

  // Initialize the relays
  //Pin Setups
  pinMode(ROOFRELAY, OUTPUT);

  //Turn Off the relays.
  digitalWrite(ROOFRELAY, HIGH);

  // Establish USB port.
  Serial.begin(BAUD_RATE);    // Baud rate to match that in the driver
}

// Wait here for command or switch request from host
void loop() 
{   
  while (Serial.available() <= 0) 
  {
    for (int cnt=0; cnt < 60; cnt++)
    {
      if (Serial.available() > 0)
        break;
      else
        delay(100);
    }
  }
  readUSB();
}       // end loop


  
