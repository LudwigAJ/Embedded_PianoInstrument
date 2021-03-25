
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


const uint32_t stepSizes [] = {51076057, // a global array that stores the different step sizes for creation of saw tooth waves 
                               54113197, // in accordance to the key pressed.
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
volatile uint8_t valueKnob_2 = 0;
volatile uint8_t valueKnob_3 = 0;



SemaphoreHandle_t keyArrayMutex; // Handle for implementing a MUTEX for the keyArray.
SemaphoreHandle_t outKeyMutex; // Handle for implementing a MUTEX for the outKey.


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
    uint8_t lower_limit;
    uint8_t upper_limit;
    uint8_t assigned_knob; // The knob an object is used for : [0, 3]
    uint8_t row, col; // For finding the correct knobs from keyArray to read from.
    uint8_t button_row, button_col; // for when pressing on the knobs.
    uint8_t dir; //direction
    

    public:

    // Initialiser // 
    KnobController(uint8_t _assigned_knob, uint8_t _lower_limit, uint8_t _upper_limit){
        assigned_knob = _assigned_knob;
        lower_limit = _lower_limit;
        upper_limit = _upper_limit;
        currentValue = 0;
        dir = NO_CHANGE;
  
        switch (assigned_knob){
            case 0:
                row = 4;
                col = 0;
                button_row = 6; 
                button_col = 3;
                break;
            case 1:
                row = 4;
                col = 2;
                button_row = 6; 
                button_col = 2;
                break;
            case 2:
                row = 3;
                col = 0;
                button_row = 5; 
                button_col = 3;
                break;
            case 3:
                row = 3;
                col = 2;
                button_row = 5; 
                button_col = 2;
                break;
            default:
                row = 4;
                col = 2;
                button_row = 6; 
                button_col = 3;
                break;  
        }
        setRow(row);
        old_state = (readCols() >> col) & 0b11; // should maybe always goto 00 ? or maybe not that's why this is here in case.
    }

    void update_state(){

          xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
            uint8_t state = (keyArray[row] >> col) & 0b11; // setRow() will already have been done. now we access keyArray.
            switch (assigned_knob){
            case 0:
                currentValue = __atomic_load_n(&valueKnob_0, __ATOMIC_RELAXED);
                break;
            case 1:
                currentValue = __atomic_load_n(&valueKnob_1, __ATOMIC_RELAXED);
                break;
            case 2:
                currentValue = __atomic_load_n(&valueKnob_2, __ATOMIC_RELAXED);
                break;
            case 3:
                currentValue = __atomic_load_n(&valueKnob_3, __ATOMIC_RELAXED);
                break;
            default:
                currentValue = __atomic_load_n(&valueKnob_0, __ATOMIC_RELAXED);
                break;
        }
          xSemaphoreGive(keyArrayMutex);
          
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
        switch (assigned_knob){
            case 0:
                __atomic_store_n(&valueKnob_0, currentValue, __ATOMIC_RELAXED);
                break;
            case 1:
                __atomic_store_n(&valueKnob_1, currentValue, __ATOMIC_RELAXED);
                break;
            case 2:
                __atomic_store_n(&valueKnob_2, currentValue, __ATOMIC_RELAXED);
                break;
            case 3:
                __atomic_store_n(&valueKnob_3, currentValue, __ATOMIC_RELAXED);
                break;
            default:
                __atomic_store_n(&valueKnob_0, currentValue, __ATOMIC_RELAXED);
                break;
        }
    }
    
};

KnobController *knobController_0 = new KnobController(0, 0, 16);
KnobController *knobController_1 = new KnobController(1, 0, 16);
KnobController *knobController_2 = new KnobController(2, 0, 16);
KnobController *knobController_3 = new KnobController(3, 0, 16);

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

void setRow(uint8_t rowIdx){
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
    static uint32_t phaseAcc = 0;
    phaseAcc += currentStepSize;

    analogWrite(OUTR_PIN, phaseAcc >> 24);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void scanKeysTask(void * pvParameters){
    
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime= xTaskGetTickCount();

    while (1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint8_t localKeyArray[7];
        bool keyPressed = false;
        uint32_t localCurrentStepSize;
        String localOutKey;

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
                localOutKey = toNote(toKey(localKeyArray[i]) + i*4);
            }
            
        }


        
        if (keyPressed == false){
            localCurrentStepSize = 0;
            localOutKey = "";    
        }

        __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          memcpy((void*)keyArray, localKeyArray, sizeof(localKeyArray));
          //keyArray = localKeyArray;
        xSemaphoreGive(keyArrayMutex);

        xSemaphoreTake(outKeyMutex, portMAX_DELAY);
          outKey = localOutKey;
        xSemaphoreGive(outKeyMutex);

        //currentStepSize = localCurrentStepSize;
        
        knobController_0->update_state();
        knobController_1->update_state();
        knobController_2->update_state();
        knobController_3->update_state();
        
        
        
        
    }
    
}

void displayUpdateTask(void * pvParameters){

    
    const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime= xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr); 
        
      
        u8g2.drawStr(2,10,"Helllo World!");

        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          u8g2.setCursor(2,20);
          u8g2.print(keyArray[0],HEX);
        
          u8g2.setCursor(10,20);
          u8g2.print(keyArray[1],HEX);
        
          u8g2.setCursor(18,20);
          u8g2.print(keyArray[2],HEX);
        xSemaphoreGive(keyArrayMutex);

        xSemaphoreTake(outKeyMutex, portMAX_DELAY);
          u8g2.drawStr(2, 30, outKey.c_str());
        xSemaphoreGive(outKeyMutex);

        uint8_t knob_0;
        uint8_t knob_1;
        uint8_t knob_2;
        uint8_t knob_3;

        knob_0 = __atomic_load_n(&valueKnob_0, __ATOMIC_RELAXED);
        knob_1 = __atomic_load_n(&valueKnob_1, __ATOMIC_RELAXED);
        knob_2 = __atomic_load_n(&valueKnob_2, __ATOMIC_RELAXED);
        knob_3 = __atomic_load_n(&valueKnob_3, __ATOMIC_RELAXED);


        //__atomic_load (&knob_3, &valueKnob_3, __ATOMIC_RELAXED);

        u8g2.setCursor(30,30);
        u8g2.print(knob_0,HEX);
        
        u8g2.setCursor(50,30);
        u8g2.print(knob_1,HEX);

        u8g2.setCursor(70,30);
        u8g2.print(knob_2,HEX);
        
        u8g2.setCursor(90,30);
        u8g2.print(knob_3,HEX);

        
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
              1,                  /* Task priority*/
              &displayUpdateHandle);   /* Pointer to store the task handle*/


  keyArrayMutex = xSemaphoreCreateMutex(); // creation of the mutex for the keyArray
  outKeyMutex = xSemaphoreCreateMutex(); // creation of the mutex for the outKey i.e. the current pressed note.
  
  vTaskStartScheduler(); // start threads. NOTE! do this after mutex assignment.


 
  ///////////////////////////////////
  //      CUSTOM SETUPS - END      //
  ///////////////////////////////////
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //~~empty

}
