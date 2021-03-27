
#include <U8g2lib.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

// put this here so it can be detected later.
class KnobController;
// put this here so it can be detected later.

//Pin definitions

//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);



//Function to set outputs via matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

 ///////////////////////////////////
 //    CUSTOM GLOBAL VARIABLES    //
 ///////////////////////////////////

const uint8_t sine[1024] = {128,128,129,130,131,131,132,133,134,135,135,136,137,138,138,139,140,141,142,142,143,144,145,145,146,147,148,149,149,150,151,152,152,153,154,155,155,156,157,158,159,159,160,161,162,162,163,164,165,165,166,167,168,168,169,170,170,171,172,173,
173,174,175,176,176,177,178,178,179,180,181,181,182,183,183,184,185,186,186,187,188,188,189,190,190,191,192,192,193,194,194,195,196,196,197,198,198,199,200,200,201,202,202,203,204,204,205,205,206,207,207,208,208,209,210,210,211,211,212,213,213,214,214,215,215,216,217,
217,218,218,219,219,220,220,221,222,222,223,223,224,224,225,225,226,226,227,227,228,228,229,229,230,230,230,231,231,232,232,233,233,234,234,234,235,235,236,236,237,237,237,238,238,239,239,239,240,240,240,241,241,241,242,242,242,243,243,243,244,244,244,245,245,245,246,
246,246,247,247,247,247,248,248,248,248,249,249,249,249,250,250,250,250,250,251,251,251,251,251,252,252,252,252,252,252,253,253,253,253,253,253,253,254,254,254,254,254,254,254,254,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,254,254,254,254,254,254,254,254,254,254,253,253,253,253,253,253,253,253,252,252,252,252,252,251,251,251,251,251,251,250,250,250,250,249,249,249,249,248,248,248,248,247,247,247,247,246,246,246,245,245,245,245,244,244,244,243,243,
243,242,242,242,241,241,241,240,240,239,239,239,238,238,238,237,237,236,236,235,235,235,234,234,233,233,232,232,232,231,231,230,230,229,229,228,228,227,227,226,226,225,225,224,224,223,223,222,222,221,221,220,220,219,219,218,217,217,216,216,215,215,214,213,213,212,212,
211,211,210,209,209,208,208,207,206,206,205,204,204,203,203,202,201,201,200,199,199,198,197,197,196,195,195,194,193,193,192,191,191,190,189,189,188,187,187,186,185,185,184,183,182,182,181,180,180,179,178,177,177,176,175,175,174,173,172,172,171,170,169,169,168,167,166,
166,165,164,163,163,162,161,160,160,159,158,157,157,156,155,154,154,153,152,151,150,150,149,148,147,147,146,145,144,144,143,142,141,140,140,139,138,137,136,136,135,134,133,133,132,131,130,129,129,128,127,126,126,125,124,123,122,122,121,120,119,119,118,117,116,115,115,
114,113,112,111,111,110,109,108,108,107,106,105,105,104,103,102,101,101,100,99,98,98,97,96,95,95,94,93,92,92,91,90,89,89,88,87,86,86,85,84,83,83,82,81,80,80,79,78,78,77,76,75,75,74,73,73,72,71,70,70,69,68,68,67,66,66,65,64,64,63,62,62,61,60,60,59,58,58,57,56,56,55,54,
54,53,52,52,51,51,50,49,49,48,47,47,46,46,45,44,44,43,43,42,42,41,40,40,39,39,38,38,37,36,36,35,35,34,34,33,33,32,32,31,31,30,30,29,29,28,28,27,27,26,26,25,25,24,24,23,23,23,22,22,21,21,20,20,20,19,19,18,18,17,17,17,16,16,16,15,15,14,14,14,13,13,13,12,12,12,11,11,11,
10,10,10,10,9,9,9,8,8,8,8,7,7,7,7,6,6,6,6,5,5,5,5,4,4,4,4,4,4,3,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,4,5,5,5,5,5,6,6,6,6,7,7,7,7,8,8,8,8,9,9,9,10,
10,10,11,11,11,12,12,12,13,13,13,14,14,14,15,15,15,16,16,16,17,17,18,18,18,19,19,20,20,21,21,21,22,22,23,23,24,24,25,25,25,26,26,27,27,28,28,29,29,30,30,31,31,32,32,33,33,34,35,35,36,36,37,37,38,38,39,40,40,41,41,42,42,43,44,44,45,45,46,47,47,48,48,49,50,50,51,51,52,
53,53,54,55,55,56,57,57,58,59,59,60,61,61,62,63,63,64,65,65,66,67,67,68,69,69,70,71,72,72,73,74,74,75,76,77,77,78,79,79,80,81,82,82,83,84,85,85,86,87,87,88,89,90,90,91,92,93,93,94,95,96,96,97,98,99,100,100,101,102,103,103,104,105,106,106,107,108,109,110,110,111,112,
113,113,114,115,116,117,117,118,119,120,120,121,122,123,124,124,125,126,127,128};

const char intToHex[] = "0123456789ABCDEF";

const uint32_t stepSizes [] = {51076057, // a global array that stores the different step sizes for creation of saw tooth waves 
                               54113197, // in accordance to the key pressed for octave 4.
                               57330935,
                               60740010,
                               64351799,
                               68178356,
                               72232452,
                               76527617,
                               81078186,
                               85899346,
                               91007187, 
                               96418756
                               };

volatile uint32_t currentStepSize; // global variable that takes in current stepsize for when we creating a sawtooth wave.
volatile uint8_t keyArray[7]; // global variable that stores the given pressed button and knobs.
String outKey; // The current note being pressed down.

volatile uint8_t valueKnob_0 = 0; 
volatile uint8_t valueKnob_1 = 0; 
volatile uint8_t valueKnob_2 = 8; //Octave control has values 0-7 but remember its x2
volatile uint8_t valueKnob_3 = 8; //volume control has values 0-16

volatile bool valueKnobButton_0 = false;
volatile bool valueKnobButton_1 = false;
volatile bool valueKnobButton_2 = false;
volatile bool valueKnobButton_3 = false; // controls if the synth is slave or master. i.e. if it plays sound by its own keys or if it takes input from serial UART.

volatile bool keysDownArray[12]; //will be accessed also when keyArray so can reuse MUTEX.
volatile int8_t tuneCounter = 0; // will be accessed in ISR for handling Polyphonism.
volatile bool extKey = false;

volatile char noteMessage[] = "xxx"; //for sending to uart
volatile char incMessage[] = "xxx"; // for reading a message and putting it on screen.

QueueHandle_t msgOutQ; // Handle for handling communication via UART.

SemaphoreHandle_t keyArrayMutex; // Handle for implementing a MUTEX for the keyArray.
SemaphoreHandle_t outKeyMutex; // Handle for implementing a MUTEX for the outKey.
SemaphoreHandle_t incMessageMutex; // Handle for implementing a MUTEX for the incMessage from UART serial.
SemaphoreHandle_t outMessageMutex;


///////////////////////////////////     
// CUSTOM GLOBAL VARIABLES - END //
///////////////////////////////////

///////////////////////////////////
//         CUSTOM CLASS          //
///////////////////////////////////

class KnobController{
    // Implements the controls for the 4 knobs on the synth board.
    /* 
    / 
    /
    */
    private:
    // Set up some constants to signify each state and direction//
    // On the form -> 0bAB.
    // So on the form 0bBA it would be ->
    // STATE_0 = 0b00 -> 0b00.
    // STATE_1 = 0b01 -> 0b10.
    // STATE_2 = 0b10 -> 0b01.
    // STATE_3 = 0b11 -> 0b11.
     
    #define STATE_0 0b00
    #define STATE_1 0b01
    #define STATE_2 0b10
    #define STATE_3 0b11
    
    // One hot encoding for direction.
    #define NO_CHANGE 0b001
    #define INCREASING 0b010
    #define DECREASING 0b100

    // Some private properties.
    uint8_t old_state; // previous value of {B,A}. initial is 0b00;
    uint8_t currentValue; // current value : [lower_limit, upper_limit]. has starting value in the middle of these two.
    bool currentButtonState;
    uint8_t lower_limit;
    uint8_t upper_limit;
    uint8_t assigned_knob; // The knob an object is used for : [0, 3]
    uint8_t dir; //direction
    

    public:

    // Initialiser // 
    KnobController(uint8_t _assigned_knob, uint8_t _lower_limit, uint8_t _upper_limit){
        assigned_knob = _assigned_knob;
        lower_limit = _lower_limit;
        upper_limit = _upper_limit;
        currentValue = (upper_limit - lower_limit)/2;
        currentButtonState = false;
        dir = NO_CHANGE;
        old_state = 0b00; // should maybe always goto 00 ? or maybe not that's why this is here in case
        
    }

    void update_state(uint8_t _row3, uint8_t _row4, uint8_t _row5, uint8_t _row6){ //making it reentrant.
            
            uint8_t button_state;
            uint8_t state; 
            
            switch (assigned_knob){
            case 0:
                button_state = (_row6 >> 3) & 0b1;
                state = _row4 & 0b11;
                
                if (button_state != 0b1){
                  currentButtonState = !currentButtonState; // set the state of button to opposite of what it was
                  delay(250); //otherwise it might switch back too quickly.
                }
                break;
            case 1:
                button_state = (_row6 >> 2) & 0b1;
                state = (_row4 >> 2) & 0b11;
                
                if (button_state != 0b1){
                  currentButtonState = !currentButtonState; // set the state of button to opposite of what it was
                  delay(250); //otherwise it might switch back too quickly.
                }
                break;
            case 2: 
                button_state = (_row5 >> 3) & 0b1;
                state = _row3 & 0b11;

                if (button_state != 0b1){
                  currentButtonState = !currentButtonState; // set the state of button to opposite of what it was
                  delay(250); //otherwise it might switch back too quickly.
                }
                break;
            case 3:
                button_state = (_row5 >> 2) & 0b1;
                state = (_row3 >> 2) & 0b11;

                if (button_state != 0b1){
                  currentButtonState = !currentButtonState; // set the state of button to opposite of what it was
                  delay(250); //otherwise it might switch back too quickly.
                }
                break;
            default:
                button_state = (_row6 >> 3) & 0b1;
                state = _row4 & 0b11;

                if (button_state != 0b1){
                  currentButtonState = !currentButtonState; // set the state of button to opposite of what it was
                  delay(250); //otherwise it might switch back too quickly.
                }
                break;
        }
          
          switch (old_state){
              case STATE_0: //0b00
                  switch (state){ //0bAB
                      case STATE_0: //0b00
                          dir = NO_CHANGE;
                      break;
                      case STATE_1: //0b01
                          if (currentValue > lower_limit) currentValue -= 1;
                          else currentValue = lower_limit;
                          dir = DECREASING;
                      break;
                      case STATE_2: //0b10
                          if (currentValue < upper_limit) currentValue += 1;
                          else currentValue = upper_limit;
                          dir = INCREASING;
                      break;
                      case STATE_3: //0b11
                          if (dir == INCREASING){
                              if (currentValue < upper_limit-1) currentValue += 2;
                              else currentValue = upper_limit;
                              dir = INCREASING;
                          }
                          else if (dir == DECREASING){
                              if (currentValue > lower_limit+1) currentValue -= 2;
                              else currentValue = lower_limit;
                              dir = DECREASING;
                          }
                      break;
                      default:
                          //What to do 
                      break;
                  }
                  break;
              case STATE_1: //0b01
                  switch (state){
                      case STATE_0: //0b00
                          if (currentValue < upper_limit) currentValue += 1;
                          else currentValue = upper_limit;
                          dir = INCREASING;
                      break;
                      case STATE_1: //0b01
                          dir = NO_CHANGE;
                      break;
                      case STATE_2: //0b10
                          if (dir == INCREASING){
                              if (currentValue < upper_limit-1) currentValue += 2;
                              else currentValue = upper_limit;
                              dir = INCREASING;
                          }
                          else if (dir == DECREASING){
                              if (currentValue > lower_limit+1) currentValue -= 2;
                              else currentValue = lower_limit;
                              dir = DECREASING;
                          }
                      break;
                      case STATE_3: //0b11
                          if (currentValue > lower_limit) currentValue -= 1;
                          else currentValue = lower_limit;
                          dir = DECREASING;
                      break;
                      default:
                      // DO
                      break;
                  }
                  break;
              case STATE_2: //0b10
                  switch (state){
                      case STATE_0: //0b00
                          if (currentValue > lower_limit) currentValue -= 1;
                          else currentValue = lower_limit;
                          dir = DECREASING;
                      break;
                      case STATE_1: //0b01
                          if (dir == INCREASING){
                              if (currentValue < upper_limit-1) currentValue += 2;
                              else currentValue = upper_limit;
                              dir = INCREASING;
                          }
                          else if (dir == DECREASING){
                              if (currentValue > lower_limit+1) currentValue -= 2;
                              else currentValue = lower_limit;
                              dir = DECREASING;
                          }
                      break;
                      case STATE_2: //0b10
                          dir = NO_CHANGE;
                      break;
                      case STATE_3: //0b11
                          if (currentValue < upper_limit) currentValue += 1;
                          else currentValue = upper_limit;
                          dir = INCREASING;
                      break;
                      default:
                      // DO
                      break;
                  }
                  break;
              case STATE_3: //0b11
                  switch (state){
                      case STATE_0: //0b00
                           if (dir == INCREASING){
                              if (currentValue < upper_limit-1) currentValue += 2;
                              else currentValue = upper_limit;
                              dir = INCREASING;
                          }
                          else if (dir == DECREASING){
                              if (currentValue > lower_limit+1) currentValue -= 2;
                              else currentValue = lower_limit;
                              dir = DECREASING;
                          }
                      break;
                      case STATE_1: //0b01
                          if (currentValue < upper_limit) currentValue += 1;
                          else currentValue = upper_limit;
                          dir = INCREASING;
                      break;
                      case STATE_2: //0b10
                          if (currentValue > lower_limit) currentValue -= 1;
                          else currentValue = lower_limit;
                          dir = DECREASING;
                      break;
                      case STATE_3: //0b11
                          dir = NO_CHANGE;
                      break;
                      default:
                      // DO
                      break;
                  }
                  break;
              default:
                  switch (state){ // Same as STATE_0. WHY? GOOD QUESTION!
                      if (currentValue > lower_limit) currentValue -= 1;
                          else currentValue = lower_limit;
                          dir = DECREASING;
                      break;
                      case STATE_1: //0b01
                          if (dir == INCREASING){
                              if (currentValue < upper_limit-1) currentValue += 2;
                              else currentValue = upper_limit;
                              dir = INCREASING;
                          }
                          else if (dir == DECREASING){
                              if (currentValue > lower_limit+1) currentValue -= 2;
                              else currentValue = lower_limit;
                              dir = DECREASING;
                          }
                      break;
                      case STATE_2: //0b10
                          dir = NO_CHANGE;
                      break;
                      case STATE_3: //0b11
                          if (currentValue < upper_limit) currentValue += 1;
                          else currentValue = upper_limit;
                          dir = INCREASING;
                      break;
                      default:
                      // DO
                      break;
                  }
              break;
        }
        old_state = state;
    }

    uint8_t getCurrentValue(){
        return currentValue;
    }
    bool getCurrentButtonState(){
        return currentButtonState;
    }

    void setCurrentButtonState(bool _in){
        currentButtonState = _in;
    }
};

KnobController *knobController_0 = new KnobController(0, 0, 2); // wave controller.
KnobController *knobController_1 = new KnobController(1, 0, 4); // mode controller.
KnobController *knobController_2 = new KnobController(2, 0, 14); // Octave controller.
KnobController *knobController_3 = new KnobController(3, 0, 30); // volume cannot go higher than 15, otherwise the speaker cracks for some sounds :C

///////////////////////////////////
//      CUSTOM CLASS - END       //
///////////////////////////////////





///////////////////////////////////
//       CUSTOM FUNCTIONS        //
///////////////////////////////////

// Custom memcpy incase the memcpy built-in breaks whe casting to (void*)
volatile void *memcpy_v(volatile uint8_t *dest, const uint8_t *src, size_t n) { 
    const uint8_t *src_c = src;
    volatile uint8_t *dest_c = dest;

    while (n > 0) {
        n--;
        dest_c[n] = src_c[n];
    }
    return dest;
}

void msgInTask(void *pvParameters) { // Handling serial UART over a Queue buffer.
    const TickType_t xFrequency = 5/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime= xTaskGetTickCount();

    while (1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        while (Serial.available() > 0) {
            
            
            char inMsg[] = "xxx";
            uint8_t counter = 0;
            
            while (counter < 4){
                char inChar = Serial.read();
                if (inChar == '\n') break; //end of message.
                else inMsg[counter] = inChar;
                counter++;
            }
    
            xSemaphoreTake(incMessageMutex, portMAX_DELAY);
              memcpy((void*)incMessage, inMsg, sizeof(inMsg));
            xSemaphoreGive(incMessageMutex);
    
            if (inMsg[0] == 'R') __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED); // set current stepsize to 0.
            else {
                __atomic_store_n(&currentStepSize, stepSizes[hexToInt(inMsg[2])], __ATOMIC_RELAXED); // set the currentStepSize
                __atomic_store_n(&valueKnob_2, (inMsg[1] - '0')*2, __ATOMIC_RELAXED); // set the octave global variable remember to remove '0' from char to turn it into an int.
               
            }
        } 
    }
}

void msgOutTask(void *pvParameters) { // Handling serial UART over a Queue buffer.
    
    char outMsg[4];
    while (1) {
        
        xQueueReceive(msgOutQ, outMsg, portMAX_DELAY); // Get the msg to the queue to then be sent to serial.
        Serial.println(outMsg);
    }
}

uint8_t readCols(){

  // Reads the current C0-3 ports and then shifts them together to form an 8 bit number with only the 4 LSBs set to signify buttons pressed. //

  uint8_t result = 0;

  uint8_t read0 = digitalRead(C0_PIN);
  uint8_t read1 = digitalRead(C1_PIN);
  uint8_t read2 = digitalRead(C2_PIN);
  uint8_t read3 = digitalRead(C3_PIN);
  
  result = read0 << 3 | read1 << 2 | read2 << 1 | read3; 

  return result;
      
}

void setRow(uint8_t rowIdx){ //Sets the correct pins to HIGH/LOW to determine which of the 8 lines to read from.
  digitalWrite(REN_PIN, LOW); //change to low before changing the other pins
  
  switch (rowIdx){
    case 0b00000000:
      digitalWrite(RA0_PIN, LOW);
      digitalWrite(RA1_PIN, LOW);
      digitalWrite(RA2_PIN, LOW);
      break;
    case 0b00000001:
      digitalWrite(RA0_PIN, HIGH);
      digitalWrite(RA1_PIN, LOW);
      digitalWrite(RA2_PIN, LOW);
      break;
    case 0b00000010:
      digitalWrite(RA0_PIN, LOW);
      digitalWrite(RA1_PIN, HIGH);
      digitalWrite(RA2_PIN, LOW);
      break;
    case 0b00000011:
      digitalWrite(RA0_PIN, HIGH);
      digitalWrite(RA1_PIN, HIGH);
      digitalWrite(RA2_PIN, LOW);
      break;
    case 0b00000100:
      digitalWrite(RA0_PIN, LOW);
      digitalWrite(RA1_PIN, LOW);
      digitalWrite(RA2_PIN, HIGH);
      break;
    case 0b00000101:
      digitalWrite(RA0_PIN, HIGH);
      digitalWrite(RA1_PIN, LOW);
      digitalWrite(RA2_PIN, HIGH);
      break;
    case 0b00000110:
      digitalWrite(RA0_PIN, LOW);
      digitalWrite(RA1_PIN, HIGH);
      digitalWrite(RA2_PIN, HIGH);
      break;
    case 0b00000111:
      digitalWrite(RA0_PIN, HIGH);
      digitalWrite(RA1_PIN, HIGH);
      digitalWrite(RA2_PIN, HIGH);
      break;
    default:
      digitalWrite(RA0_PIN, LOW); // Default to 0b000.
      digitalWrite(RA1_PIN, LOW);
      digitalWrite(RA2_PIN, LOW);
      break;
    
  }

  digitalWrite(REN_PIN, HIGH); //change to high when done
}

uint8_t toKey(uint8_t input){
  switch (input){
    case 0x7:
      return 0;
      break;
    case 0xB:
      return 1;
      break;
    case 0xD:
      return 2;
      break;
    case 0xE:
      return 3;
      break;
    default:
      return 0;
      break;
  }  
}

uint8_t hexToInt(char _hex){
    switch (_hex){
        case '0':
            return 0;
            break;
        case '1':
            return 1;
            break;
        case '2':
            return 2;
            break;
        case '3':
            return 3;
            break;
        case '4':
            return 4;
            break;
        case '5':
            return 5;
            break;
        case '6':
            return 6;
            break;
        case '7':
            return 7;
            break;
        case '8':
            return 8;
            break;
        case '9':
            return 9;
            break;
        case 'A':
            return 10;
            break;
        case 'B':
            return 11;
            break;
        case 'C':
            return 12;
            break;
        case 'D':
            return 13;
            break;
        case 'E':
            return 14;
            break;
        case 'F':
            return 15;
            break;
        default:
            return 0;
            break; 
    }
}

String toNote(uint8_t input){ 
  // turns a given number into its corresponding note.
  String result;

  switch (input){
    case 0:
      result = "C";
      break;
    case 1:
      result = "C#";
      break;
    case 2:
      result = "D";
      break;
    case 3:
      result = "D#";
      break;
    case 4:
      result = "E";
      break;
    case 5:
      result = "F";
      break;
    case 6:
      result = "F#";
      break;
    case 7:
      result = "G";
      break;
    case 8:
      result = "G#";
      break;
    case 9:
      result = "A";
      break;
    case 10:
      result = "A#";
      break;
    case 11:
      result = "B";
      break;
    default:
      result = "A";
      break;
  }
  return result;
}



void sampleISR(){


    #define ECHO 1024*20
    static uint8_t echo_buffer[ECHO] = {0};
    static int echooffset = 1024*8; // change to delay more example int echooffset=1024*6;
    static int echooffsetplay = 0;
    int EFFECT = valueKnob_1/2;

    if (valueKnob_0/2 == 0){
        static uint32_t phaseAcc = 0;
        
        if (valueKnob_2/2 > 4) phaseAcc += (currentStepSize << (valueKnob_2/2 - 4));
        else phaseAcc += (currentStepSize >> (4 - valueKnob_2/2));
    
        uint8_t outValue = (phaseAcc >> 24) >> (8 - valueKnob_3/4); // divide by 2 two times. since one detent is a 2 step and we want it to symbolise 1 step.

        if (EFFECT > 0){

            if (EFFECT==1){ // reverb 
                    echo_buffer[echooffset] += outValue / 2;
                    
                    if (echooffset++ >= ECHO - 1) echooffset = 0;
                    outValue += echo_buffer[echooffsetplay];
                    echo_buffer[echooffsetplay] *= 0.45;// decay
                    
                    if (echooffsetplay++ >= ECHO-1) echooffsetplay = 0;
            }
            if (EFFECT==2){ // echo
        
                    echo_buffer[echooffset] = outValue / 2;
                    
                    if (echooffset++ >= ECHO-1) echooffset=0;
                    outValue += echo_buffer[echooffsetplay] * 0.4;
                    
                    if (echooffsetplay++ >= ECHO-1) echooffsetplay=0;
            }  
        }
        
        analogWrite(OUTR_PIN, outValue);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    else if (valueKnob_0/2 == 1){
        static uint8_t phaseAcc = 0;
        static uint16_t s_i = 0;
        uint8_t outValue;
        phaseAcc = sine[s_i];
        
        outValue = (phaseAcc) >> (8 - valueKnob_3/4);

        if (EFFECT > 0){

            if (EFFECT==1){ // reverb 
                    echo_buffer[echooffset] += outValue / 2;// original *  ( amount of original )
                    
                    if (echooffset++ >= ECHO - 1) echooffset = 0;
                    outValue += echo_buffer[echooffsetplay];
                    echo_buffer[echooffsetplay] *= 0.45;// decay
                    
                    if (echooffsetplay++ >= ECHO-1) echooffsetplay = 0;
            }
            if (EFFECT==2){ // echo
        
                    echo_buffer[echooffset] = outValue / 2;// original *  ( amount of original )
                    
                    if (echooffset++ >= ECHO-1) echooffset=0;
                    outValue += echo_buffer[echooffsetplay] * 0.4;// amount to mix into dry
                    
                    if (echooffsetplay++ >= ECHO-1) echooffsetplay=0;
            }  
        }
        
        analogWrite(OUTR_PIN, outValue);

        uint32_t steppy;  
        if (valueKnob_2/2 > 4) steppy = currentStepSize << (valueKnob_2/2 - 4);
        else steppy = currentStepSize >> (4 - valueKnob_2/2); 

        s_i = ((steppy >> 22) + s_i)%1024;
    
      
    } //uint8_t outValue = (phaseAcc) >> (8-knob3.rotation()/2);
    
}

void playJacob(){
    /* 
    C = 0, D = 2, E = 4, F = 5, G = 7, A = 9, B = 11.
    */
    uint32_t starting = millis();
    
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[2], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[4], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);

    delay(150);
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    
    
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[2], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[4], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);

    delay(150);
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    

    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[4], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[5], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[7], __ATOMIC_RELAXED);

    delay(500);
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    delay(50);

    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[4], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[5], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[7], __ATOMIC_RELAXED);

    delay(500);
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    delay(50);

    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[7], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[9], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[7], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[5], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[4], __ATOMIC_RELAXED);
    delay(700);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);
    delay(400);

    
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    delay(50);

    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[7], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[9], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[7], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[5], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[4], __ATOMIC_RELAXED);
    delay(700);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);
    delay(400);
    
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    delay(50);

    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[7] >> 1, __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);

    delay(500);
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    delay(100);

    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[0], __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[7] >> 1, __ATOMIC_RELAXED);
    delay(500);
    __atomic_store_n(&currentStepSize, stepSizes[0] >> 1, __ATOMIC_RELAXED);
    delay(800);

    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);

    Serial.println(millis() - starting);
}


void scanKeysTask(void * pvParameters){
    
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime= xTaskGetTickCount();

    
    static char oldLocalNoteMessage[] = "xxx";
    uint8_t localKeyPressed = 14; //just a placeholder since value can never be 14.
    static uint8_t old_localKeyPressed = 14; //just a placeholder since value can never be 14.
    static bool old_localKeysDownArray[12] = {false,false,false,false,false,false,false,false,false,false,false,false};
    static uint8_t old_octave = 4;
    
    while (1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint8_t localKeyArray[7];
        bool keyPressed = false;
        uint32_t localCurrentStepSize;
        bool localKeysDownArray[12] = {false,false,false,false,false,false,false,false,false,false,false,false};
        String localOutKey = "";
        

        for (uint8_t i = 0; i < 7; i++){
            setRow(i);
            delayMicroseconds(3);
            uint8_t keys = readCols();
            localKeyArray[i] = keys;
        }
    
        
        for (uint8_t i = 0; i < 3; i++){
        
            if (localKeyArray[i] != 0xF){
                keyPressed = true;
                localCurrentStepSize = stepSizes[toKey(localKeyArray[i]) + i*4];
                localKeyPressed = toKey(localKeyArray[i]) + i*4;
                localOutKey = toNote(localKeyPressed);
            

                for (uint8_t ii = 0; ii < 4; ii++){
                    if ( ((localKeyArray[i] >> ii) & 0b1) == 0 ) localKeysDownArray[3-ii+i*4] = true; // i.e. extract the 0 bits using a N-OR gate.
                    //else localKeysDownArray[3-ii+i*4] = false;
                }
            }
        }
        
        if (keyPressed == false){
            localCurrentStepSize = 0;
            old_localKeyPressed = 14; //just a placeholder since value can never be 14.
            localOutKey = "";    
        }

        
        bool playSongCheck = __atomic_load_n(&valueKnobButton_3, __ATOMIC_RELAXED);

        if (playSongCheck == true){ // frere jaque
            playJacob();
            
            knobController_3->setCurrentButtonState(false);
            __atomic_store_n(&valueKnobButton_3, false, __ATOMIC_RELAXED);
        }

        __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

        uint8_t counter = 0;
        for (uint8_t i = 0; i < 12; i++){
            char localNoteMessage[] = "xxx";
            
            if (localKeysDownArray[i] == true){
                
                if (old_localKeysDownArray[i] == false){
                  
                    localNoteMessage[0] = 'P';
                    localNoteMessage[1] = valueKnob_2/2 + '0';
                    localNoteMessage[2] = intToHex[i];

                    xSemaphoreTake(keyArrayMutex, portMAX_DELAY); // this is okay to use here because they are similar to access and this depends on keyArray.
                      memcpy((void*)noteMessage, localNoteMessage, sizeof(localNoteMessage));
                    xSemaphoreGive(keyArrayMutex); // this is okay to use here because they are similar to access and this depends on keyArray.
                    
                    xQueueSend( msgOutQ, (char*) localNoteMessage, portMAX_DELAY);

                }
                else if (valueKnob_2/2 != old_octave) {
                  
                    localNoteMessage[0] = 'R';
                    localNoteMessage[1] = old_octave + '0';
                    localNoteMessage[2] = intToHex[i];

                    xSemaphoreTake(keyArrayMutex, portMAX_DELAY); // this is okay to use here because they are similar to access and this depends on keyArray.
                      memcpy((void*)noteMessage, localNoteMessage, sizeof(localNoteMessage));
                    xSemaphoreGive(keyArrayMutex); // this is okay to use here because they are similar to access and this depends on keyArray.

                    xQueueSend( msgOutQ, (char*) localNoteMessage, portMAX_DELAY);

                    localNoteMessage[0] = 'P';
                    localNoteMessage[1] = valueKnob_2/2 + '0';
                    localNoteMessage[2] = intToHex[i];

                    xSemaphoreTake(keyArrayMutex, portMAX_DELAY); // this is okay to use here because they are similar to access and this depends on keyArray.
                      memcpy((void*)noteMessage, localNoteMessage, sizeof(localNoteMessage));
                    xSemaphoreGive(keyArrayMutex); // this is okay to use here because they are similar to access and this depends on keyArray.

                    xQueueSend( msgOutQ, (char*) localNoteMessage, portMAX_DELAY);
                }
            }
            else {

                if (old_localKeysDownArray[i] == true){
                  
                    localNoteMessage[0] = 'R';
                    localNoteMessage[1] = old_octave + '0';
                    localNoteMessage[2] = intToHex[i];

                    xSemaphoreTake(keyArrayMutex, portMAX_DELAY); // this is okay to use here because they are similar to access and this depends on keyArray.
                      memcpy((void*)noteMessage, localNoteMessage, sizeof(localNoteMessage));
                    xSemaphoreGive(keyArrayMutex); // this is okay to use here because they are similar to access and this depends on keyArray.
                    
                    xQueueSend( msgOutQ, (char*) localNoteMessage, portMAX_DELAY);
                }
            }

            old_localKeysDownArray[i] = localKeysDownArray[i];

        } // end of for loop with 12 iterations
        
        old_octave = valueKnob_2/2;
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          memcpy((void*)keyArray, localKeyArray, sizeof(localKeyArray));
          memcpy((void*)keysDownArray, localKeysDownArray, sizeof(localKeysDownArray));
        xSemaphoreGive(keyArrayMutex);

        xSemaphoreTake(outKeyMutex, portMAX_DELAY);
          outKey = localOutKey;
        xSemaphoreGive(outKeyMutex);


        knobController_0->update_state(localKeyArray[3], localKeyArray[4], localKeyArray[5], localKeyArray[6]);
        knobController_1->update_state(localKeyArray[3], localKeyArray[4], localKeyArray[5], localKeyArray[6]);
        knobController_2->update_state(localKeyArray[3], localKeyArray[4], localKeyArray[5], localKeyArray[6]);
        knobController_3->update_state(localKeyArray[3], localKeyArray[4], localKeyArray[5], localKeyArray[6]);

        __atomic_store_n(&valueKnob_0, knobController_0->getCurrentValue(), __ATOMIC_RELAXED);
        __atomic_store_n(&valueKnob_1, knobController_1->getCurrentValue(), __ATOMIC_RELAXED);
        __atomic_store_n(&valueKnob_2, knobController_2->getCurrentValue(), __ATOMIC_RELAXED);
        __atomic_store_n(&valueKnob_3, knobController_3->getCurrentValue(), __ATOMIC_RELAXED);

        __atomic_store_n(&valueKnobButton_0, knobController_0->getCurrentButtonState(), __ATOMIC_RELAXED);
        __atomic_store_n(&valueKnobButton_1, knobController_1->getCurrentButtonState(), __ATOMIC_RELAXED);
        __atomic_store_n(&valueKnobButton_2, knobController_2->getCurrentButtonState(), __ATOMIC_RELAXED);
        __atomic_store_n(&valueKnobButton_3, knobController_3->getCurrentButtonState(), __ATOMIC_RELAXED);
 
    }
    
}

void displayUpdateTask(void * pvParameters){

    
    const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime= xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr); 

        
        ///////////////////////////////////////////////////////////
        //                  Making the knob names                //
        ///////////////////////////////////////////////////////////

        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          u8g2.setCursor(83,10);
          u8g2.print(keyArray[0],HEX);
        
          u8g2.setCursor(90,10);
          u8g2.print(keyArray[1],HEX);
        
          u8g2.setCursor(97,10);
          u8g2.print(keyArray[2],HEX);

          //u8g2.drawStr(2,10,"IN:");
          u8g2.setCursor(2,10);
          u8g2.print((char*) noteMessage);
        xSemaphoreGive(keyArrayMutex);

        xSemaphoreTake(outKeyMutex, portMAX_DELAY);
          
          u8g2.drawStr(62, 10, outKey.c_str());
        xSemaphoreGive(outKeyMutex);

        xSemaphoreTake(incMessageMutex, portMAX_DELAY);
          //u8g2.drawStr(35,10,"OUT:");
          u8g2.setCursor(30,10);
          u8g2.print((char*) incMessage);
        xSemaphoreGive(incMessageMutex);

        uint8_t knob_0 = __atomic_load_n(&valueKnob_0, __ATOMIC_RELAXED);
        uint8_t knob_1 = __atomic_load_n(&valueKnob_1, __ATOMIC_RELAXED);
        uint8_t knob_2 = __atomic_load_n(&valueKnob_2, __ATOMIC_RELAXED)/2;
        uint8_t knob_3 = __atomic_load_n(&valueKnob_3, __ATOMIC_RELAXED)/2;

        bool button_0 = __atomic_load_n(&valueKnobButton_0, __ATOMIC_RELAXED);
        bool button_1 = __atomic_load_n(&valueKnobButton_1, __ATOMIC_RELAXED);
        bool button_2 = __atomic_load_n(&valueKnobButton_2, __ATOMIC_RELAXED);
        bool button_3 = __atomic_load_n(&valueKnobButton_3, __ATOMIC_RELAXED);

        u8g2.drawStr(2,20,"Wave");
        //u8g2.setCursor(2,30);
        if (knob_0/2 == 0) u8g2.drawStr(2,30,"Saw");
        else if (knob_0/2 == 1) u8g2.drawStr(2,30,"Sine");
        //u8g2.print(knob_0,HEX);

        u8g2.drawStr(40,20,"Mode");
        //u8g2.setCursor(44,30);
        if (knob_1/2 == 0) u8g2.drawStr(42,30,"Norm");
        else if (knob_1/2 == 1) u8g2.drawStr(42,30,"Revb");
        else if (knob_1/2 == 2) u8g2.drawStr(42,30,"Echo");

        u8g2.drawStr(77,20,"Oct");
        u8g2.setCursor(85,30);
        u8g2.print(knob_2,HEX);

        u8g2.drawStr(105,20,"Vol");
        u8g2.setCursor(110,30);
        u8g2.print(knob_3);

        u8g2.setCursor(115,10);
        u8g2.print(button_3,HEX);

        
        u8g2.sendBuffer();

    }   
}


///////////////////////////////////
//      CUSTOM FUNCTIONS END     //
///////////////////////////////////

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(115200);
  Serial.println("Hello World");

  //////////////////////////////////
  //         CUSTOM SETUPS        //
  //////////////////////////////////
  
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer= new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT); // changed from 22,000 to 12,000
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  TaskHandle_t scanKeysHandle = NULL;
  TaskHandle_t displayUpdateHandle = NULL;
  TaskHandle_t msgOutTaskHandle = NULL;
  TaskHandle_t msgInTaskHandle = NULL;
  
  
  xTaskCreate(scanKeysTask,       /* Function that implements the task */
              "scanKeys",         /* Text name for the task */
              64,                 /* Stack size in words, not bytes*/
              NULL,               /* Parameter passed into the task */
              2,                  /* Task priority*/
              &scanKeysHandle);   /* Pointer to store the task handle*/

  xTaskCreate(displayUpdateTask,       /* Function that implements the task */
              "displayUpdate",         /* Text name for the task */
              256,                 /* Stack size in words, not bytes*/
              NULL,               /* Parameter passed into the task */
              3,                  /* Task priority*/
              &displayUpdateHandle);   /* Pointer to store the task handle*/
              
  xTaskCreate(msgOutTask,       /* Function that impleements the task */
              "msgOut",         /* Text name for the task */
              32,                 /* Stack size in words, not bytes*/
              NULL,               /* Parameter passed into the task */
              2,                  /* Task priority*/
              &msgOutTaskHandle);   /* Pointer to store the task handle*/

  xTaskCreate(msgInTask,       /* Function that implements the task */
              "msgIn",         /* Text name for the task */
              32,                 /* Stack size in words, not bytes*/
              NULL,               /* Parameter passed into the task */
              1,                  /* Task priority*/
              &msgInTaskHandle);   /* Pointer to store the task handle*/

  msgOutQ = xQueueCreate( 8, 4 ); // setup communication buffer for UART.
  keyArrayMutex = xSemaphoreCreateMutex(); // creation of the mutex for the keyArray
  outKeyMutex = xSemaphoreCreateMutex(); // creation of the mutex for the outKey i.e. the current pressed note.
  incMessageMutex = xSemaphoreCreateMutex();
  
  vTaskStartScheduler(); // start threads. NOTE! do this after mutex assignment.
 
  ///////////////////////////////////
  //      CUSTOM SETUPS - END      //
  ///////////////////////////////////
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //~~empty

}
