#include "header.h"
#include "userInterface.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);

extern uint8_t CVmode;
extern uint8_t assistControl;
extern uint8_t activateVentilatorOperation;
extern struct setpointStatus setpoint;


void Display_menu_1(void);
void Display_menu_2(void);
void get_value(void);
void Input_Validation(void);
void clear_value(void);
void KeyPressed();           //ISR function

bool key_int_flag = false;
unsigned int key_value = 0;
unsigned int display_screen_state = 1;
unsigned int display_screen_Next_state = 0;
unsigned int new_value = 0;

/*
 * Default Parameters
 */
#define VENT_MODE_VCV       0
#define VENT_MODE_PCV       1
#define VENT_MODE_AC_VCV    2
#define VENT_MODE_AC_PCV    3

uint8_t IE_R_Value[3][2] = { {1,1},{1,2},{1,3}};
char VentMode[4][7]= {  "VCV   ",  "PCV   ",  "AC-VCV",  "AC-PCV" };
/*
unsigned int Param_FiO2     = 75;     // FiO2
unsigned int Param_TV       = 200;    // Tidal Volume
unsigned int Param_RR       = 20;     // Respiratory Rate
unsigned int Param_PC       = 20;     // Pressure Control
unsigned int Param_IE_R     = 1;      // I:E Ratio
float        Param_TRIG     = 0.5;             // Triggering
unsigned int Param_vMode    = VENT_MODE_VCV;

unsigned int Param_New_vMode  = 0;
float        Param_New_TRIG   = 0;
unsigned int Param_New_IE_R   = 1;
*/

uint8_t Param_FiO2     = 75;     // FiO2
uint16_t Param_TV       = 200;    // Tidal Volume
uint8_t Param_RR       = 20;     // Respiratory Rate
uint8_t Param_PC       = 20;     // Pressure Control
uint8_t Param_IE_R     = 1;      // I:E Ratio
float        Param_TRIG     = 0.5;             // Triggering
uint8_t Param_vMode    = VENT_MODE_VCV;

uint8_t Param_New_vMode  = 0;
float        Param_New_TRIG   = 0;
uint8_t Param_New_IE_R   = 1;



void LCD_setup() {
  attachInterrupt(digitalPinToInterrupt(2), KeyPressed, FALLING); 
  
  lcd.init();                      
  lcd.backlight();
  lcd.clear();
  DDRA = 0;
  lcd.blink();
  key_int_flag = true;
}

void InitializeParams() {
  Param_FiO2     = setpoint.reqFiO2;     // FiO2
  Param_TV       = setpoint.reqVolume;    // Tidal Volume
  Param_RR       = setpoint.reqBPM;     // Respiratory Rate
  Param_PC       = setpoint.reqPressure;     // Pressure Control
  Param_TRIG     = setpoint.flowTriggerSenstivity;             // Triggering
  Param_IE_R     = setpoint.reqI_E_Section;      // I:E Ratio

  if (CVmode == VOL_CONT_MODE) {
    if (assistControl == 1) Param_vMode = VENT_MODE_AC_VCV;
    else Param_vMode = VENT_MODE_VCV;
  }
  else { 
    //PRESS_CONT_MODE
    if (assistControl == 1) Param_vMode = VENT_MODE_AC_PCV;
    else Param_vMode = VENT_MODE_PCV;
  }
}

void Display_menu_1(void)
{
    char displayStr[21]={};
    lcd.clear();
    
    // Display FiO2
    lcd.setCursor(0, 0);
    sprintf(displayStr, "FiO2= %u %%", Param_FiO2);
    lcd.print(displayStr);
    
    // Display Tidal Volume
    lcd.setCursor(0, 1);
    displayStr[20]={};
    sprintf(displayStr, "T.V = %u ml", Param_TV);
    lcd.print(displayStr);

    // Display Respiratory Rate
    lcd.setCursor(0, 2);
    displayStr[20]={};
    sprintf(displayStr, "RR= %u BPM", Param_RR);
    lcd.print(displayStr);

    // Display Pressure Control
    lcd.setCursor(0, 3);
    displayStr[20]={};
    sprintf(displayStr, "PC= %u cm H2O", Param_PC);
    lcd.print(displayStr);
}

void Display_menu_2(void)
{
    char displayStr[21]={};
    lcd.clear();

    // Display I:E Ratio
    lcd.setCursor(0, 0);
    sprintf(displayStr, "I.E = %u:%u", IE_R_Value[Param_IE_R][0], IE_R_Value[Param_IE_R][1]);
    lcd.print(displayStr);

    // Display Triggering
    lcd.setCursor(0, 1);
    lcd.print("Trig= ");
    lcd.print(Param_TRIG);
    lcd.print(" SLPM");
    
    // Display Vent mode
    lcd.setCursor(0, 2);
    lcd.print("Vent Mode = ");
    lcd.print(VentMode[Param_vMode]);
    
    // for future values
    //lcd.setCursor(0, 3);
    //lcd.print("enter new parm");
}

void get_value(void)
{
  // if the display state is edit mode
  if( display_screen_Next_state>=1 && display_screen_Next_state <=7)
  {
    // check the key pressed
    if (key_value>=0 && key_value <=9)
    {
        new_value = new_value *10;
        new_value= new_value + key_value;        
        lcd.print(key_value);
    }
  }
}

void Input_Validation(void) ///
{
    switch(display_screen_Next_state)
    {
      case DISPLAY_FIO2:
          if( new_value >=50 && new_value <= 100)
          {
            Param_FiO2 = new_value;
            setpoint.reqFiO2 = (float)(Param_FiO2);
            new_value = 0;
          }
          else
          {
            new_value = 0;
          }
          break;
       case DISPLAY_T_V:
          if( new_value >= 200 && new_value <=800)
          {
            Param_TV = new_value;
            setpoint.reqVolume = (float)(Param_TV);
            new_value = 0;
          }
          else
          {
            new_value = 0;
          }
          break;
       case DISPLAY_RR:
          if( new_value >= 8 && new_value <=35)
          {
            Param_RR = new_value;
            setpoint.reqBPM = (float)(Param_RR);
            new_value = 0;
          }
          else
          {
            new_value = 0;
          }
          break;
        case DISPLAY_PC:
          if( new_value >= 0 && new_value <=40)
          {
            Param_PC = new_value;
            setpoint.reqPressure = (float)(Param_PC);
            new_value = 0;
          }
          else
          {
            new_value = 0;
          }
          break;
        case DISPLAY_TRIG:
          Param_TRIG = Param_New_TRIG;
          setpoint.flowTriggerSenstivity = Param_TRIG;
          break;
        case DISPLAY_I_E:
          Param_IE_R = Param_New_IE_R;
          setpoint.reqI_E_Section = Param_IE_R;
          break;
        case DISPLAY_VMODE:
          Param_vMode = Param_New_vMode;
          switch (Param_vMode)
          {
          case VENT_MODE_VCV:
            CVmode = VOL_CONT_MODE;
            break;
          case VENT_MODE_PCV:
            CVmode = PRESS_CONT_MODE;
            break;
          case VENT_MODE_AC_VCV:
            CVmode = VOL_CONT_MODE;
            assistControl = 1;
            break;
          case VENT_MODE_AC_PCV:
            CVmode = PRESS_CONT_MODE;
            assistControl = 1;
            break;          
          default:
            break;
          }
          break;
        default:
          new_value = 0;
          break;
    }
}

// clear input value
void clear_value(void)
{
  if( display_screen_Next_state>=1 && display_screen_Next_state <=7)
  {
    lcd.setCursor(0, 3);
    lcd.print("New Value =        ");
    lcd.setCursor(12, 3);
    new_value= 0;
  }
}

void Display(void)
{
  if(key_int_flag)
  {
      char displayStr[21]={};
      key_int_flag = false;
      switch(display_screen_state)
      {
        case 0 :// welcome screen
          lcd.clear();
          lcd.setCursor(0, 3);
          lcd.setCursor(0, 1);
          lcd.setCursor(0, 2);
          lcd.setCursor(0, 0);
          lcd.print("Welcome Screen");
          break;
        case DISPLAY_FIO2 :
          Display_menu_1();
          display_screen_Next_state = DISPLAY_SET_FiO2;
          lcd.setCursor(19, 0);
          break;
        case DISPLAY_T_V:
          Display_menu_1();
          display_screen_Next_state = DISPLAY_SET_T_V;
          lcd.setCursor(19, 1);
          break;
         case DISPLAY_RR:
          Display_menu_1();
          display_screen_Next_state = DISPLAY_SET_RR;
          lcd.setCursor(19, 2);
          break;
         case DISPLAY_PC:
          Display_menu_1();
          display_screen_Next_state = DISPLAY_SET_PC;
          lcd.setCursor(19, 3);
          break;
         case DISPLAY_I_E :
          Display_menu_2();
          display_screen_Next_state = DISPLAY_SET_I_E;
          lcd.setCursor(19, 0);
          break;
         case DISPLAY_TRIG:
          Display_menu_2();
          display_screen_Next_state = DISPLAY_SET_TRIG;
          lcd.setCursor(19, 1);
          break;
         case DISPLAY_VMODE:
          Display_menu_2();
          display_screen_Next_state = DISPLAY_SET_VMODE;
          lcd.setCursor(19, 2);
          break;
         case 8:
          break;
         case DISPLAY_SET_FiO2:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("FiO2");
          lcd.setCursor(0, 1);
          lcd.print("Range 50 to 100 %");
          lcd.setCursor(0, 2);
          displayStr[20]={};
          sprintf(displayStr, "Set Value= %u", Param_FiO2);
          lcd.print(displayStr);
          lcd.setCursor(0, 3);
          lcd.print("New Value = ");
          display_screen_Next_state = DISPLAY_FIO2;
          break;
         case DISPLAY_SET_T_V:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Tidal Volume");
          lcd.setCursor(0, 1);
          lcd.print("Range 200 to 800 ml");
          lcd.setCursor(0, 2);
          displayStr[20]={};
          sprintf(displayStr, "Set Value = %u", Param_TV);
          lcd.print(displayStr);
          lcd.setCursor(0, 3);
          lcd.print("New Value = ");
          display_screen_Next_state = DISPLAY_T_V;
          break;         
         case DISPLAY_SET_I_E:
          if(display_screen_Next_state != DISPLAY_I_E)
          {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("I:E Ratio");
            lcd.setCursor(0, 1);
            lcd.print("Range 1:1 to 1:3");
            lcd.setCursor(0, 2);
            lcd.print("Set Value = ");
            lcd.print(IE_R_Value[Param_IE_R][0]); lcd.print(":"); lcd.print(IE_R_Value[Param_IE_R][1]);
            lcd.setCursor(0, 3);
            lcd.print("New Value = ");
            lcd.print(IE_R_Value[Param_New_IE_R][0]); lcd.print(":"); lcd.print(IE_R_Value[Param_New_IE_R][1]);
            display_screen_Next_state = DISPLAY_I_E;
            Param_New_IE_R = Param_IE_R;
          }
          else
          {
            lcd.setCursor(0, 3);
            lcd.print("New Value = ");
            lcd.print(IE_R_Value[Param_New_IE_R][0]); lcd.print(":"); lcd.print(IE_R_Value[Param_New_IE_R][1]);
          }
          break;         
         case DISPLAY_SET_TRIG:
          if(display_screen_Next_state != DISPLAY_TRIG)
          {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Triggering");
            lcd.setCursor(0, 1);
            lcd.print("Range -0.5 to 5 SLPM");
            lcd.setCursor(0, 2);
            lcd.print("Set Value = ");
            lcd.print(Param_TRIG);
            lcd.setCursor(0, 3);
            lcd.print("New Value = ");
            lcd.print(Param_TRIG);
            lcd.setCursor(0, 3);
            lcd.print("New Value = ");
            lcd.print(Param_TRIG);
            lcd.print(" ");
            display_screen_Next_state = DISPLAY_TRIG;
            Param_New_TRIG = Param_TRIG;
          }
          else
          {
            lcd.setCursor(0, 3);
            lcd.print("New Value = ");
            lcd.print(Param_New_TRIG);
            lcd.print(" ");
          }
          break;
         case DISPLAY_SET_RR:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Respiratory Rate");
          lcd.setCursor(0, 1);
          lcd.print("Range 8 to 35 BPM");
          lcd.setCursor(0, 2);
          displayStr[20]={};
          sprintf(displayStr, "Set Value = %u", Param_RR);
          lcd.print(displayStr);
          lcd.setCursor(0, 3);
          lcd.print("New Value = ");
          display_screen_Next_state = DISPLAY_RR;
          break;
         case DISPLAY_SET_PC:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Pressure Control");
          lcd.setCursor(0, 1);
          lcd.print("Range 0 to 40 cm H2O");
          lcd.setCursor(0, 2);
          displayStr[20]={};
          sprintf(displayStr, "Set Value = %u", Param_PC);
          lcd.print(displayStr);
          lcd.setCursor(0, 3);
          lcd.print("New Value = ");
          display_screen_Next_state = DISPLAY_PC;
          break;
         case DISPLAY_SET_VMODE:
            if(display_screen_Next_state != DISPLAY_VMODE)
            {
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Vent Mode");
              lcd.setCursor(0, 1);
              lcd.print("VCV PCV AC-VCV/PCV");
              lcd.setCursor(0, 2);
              lcd.print("Set Mode = ");
              lcd.print(VentMode[Param_vMode]);
              lcd.setCursor(0, 3);
              lcd.print("New Mode = ");
              lcd.print(VentMode[Param_vMode]);
              Param_New_vMode = Param_vMode;
              display_screen_Next_state = DISPLAY_VMODE;
            }
            else
            {
              lcd.setCursor(0, 3);
              lcd.print("New Mode = ");
              lcd.print(VentMode[Param_New_vMode]);
            }
          break;  
         case DISPLAY_INPUT:
          get_value();
          break;
         case DISPLAY_CLEAR:
          clear_value();
          break;
         default:
           break;
      }
    // testing
    // Serial.print(new_value);
    // Serial.print("  ");
    // Serial.print(display_screen_state);
    // Serial.print("  ");
    // Serial.println(key_value);
  }
}

void KeyPressed()           //ISR function
{
  if(key_int_flag == false)
  {
    byte val = (PINA & 0xF0) >> 4; // get PORTA value
    switch(val)
    {
        case KEY_0: key_value = 0;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_1: key_value = 1;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_2: key_value = 2;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_3: key_value = 3;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_4: key_value = 4;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_5: key_value = 5;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_6: key_value = 6;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_7: key_value = 7;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_8: key_value = 8;
            display_screen_state = DISPLAY_INPUT;
            break;
        case KEY_9: key_value = 9;
            display_screen_state = DISPLAY_INPUT;
            break;

        // char '+' (go up in main menu)
        case KEY_A:
          // Menu go up or change screen
          if(display_screen_state >= 2 && display_screen_state <=7 )
          {
            display_screen_state--;
          }
          else if( display_screen_state == 1)
          {
            display_screen_state = 7;
          }
          // Change Triggering
          else if( display_screen_Next_state == DISPLAY_TRIG )
          {
            if( Param_New_TRIG < 5)
            {
               Param_New_TRIG = Param_New_TRIG + 0.5;
               display_screen_state = DISPLAY_SET_TRIG; // remove this after - function is made
            }
          }
          // Change I:E Ratio        
          else if (display_screen_Next_state == DISPLAY_I_E)
          {
            if(Param_New_IE_R < 2)
            {
              Param_New_IE_R++;
              display_screen_state = DISPLAY_SET_I_E;
            }
          }
          // Change Vent mode
          else if (display_screen_Next_state == DISPLAY_VMODE)
          {
            if(Param_New_vMode < 3)
            {
              Param_New_vMode++;
              display_screen_state = DISPLAY_SET_VMODE;
            }
          }         
          else
          {
            display_screen_state = DISCARD_INPUT;
          }
          break;

        /*  Main Menu = Go Down, Setting Menu    */
        case KEY_B: 
          // Menu go down or change screen
          if(display_screen_state >= 0 && display_screen_state <=6 )
          {
            display_screen_state++;
          }
          else if( display_screen_state == 7)
          {
            display_screen_state = 1;
          }
          // Change Triggering Value
          else if( display_screen_Next_state == DISPLAY_TRIG )
          {
            if( Param_New_TRIG > -0.5)
            {
               Param_New_TRIG = Param_New_TRIG - 0.5;
               display_screen_state = DISPLAY_SET_TRIG;
            }
          }
          // Change I:E Ratio
          else if (display_screen_Next_state == DISPLAY_I_E)
          {
            if(Param_New_IE_R > 0)
            {
              Param_New_IE_R--;
              display_screen_state = DISPLAY_SET_I_E;
            }
          }
          // Change Vent mode
          else if (display_screen_Next_state == DISPLAY_VMODE)
          {
            if(Param_New_vMode > 0)
            {
              Param_New_vMode--; 
              display_screen_state = DISPLAY_SET_VMODE;
            }
          }
          else
          {
            display_screen_state = DISCARD_INPUT;
          }
          break;


        /* char 'x'   clear Input */
        case KEY_C: 
          if( display_screen_Next_state >= 1 && display_screen_Next_state <= 4)
          {
            display_screen_state = DISPLAY_CLEAR;
          }
          break;  
          
        /* key '/'  NOT USED     */
        case KEY_D: 
            if( display_screen_Next_state >= 1 && display_screen_Next_state <= 7)
            {
              display_screen_state = DISCARD_INPUT;
            }
            break;
          
          
        /*  edit settings in main menu, save settings in settings menu  (in simulation key '=') */
        case KEY_HASH: 
          Input_Validation();
          display_screen_state = display_screen_Next_state;
          display_screen_Next_state = 0;
          new_value= 0;
          break;
          
        /*  Discard settings in settings menu  (in simulation key 'on/c') */
        case KEY_STAR:  
          if( display_screen_Next_state>=1 && display_screen_Next_state <=6)
          {
            display_screen_state = display_screen_Next_state;
            new_value= 0;
          }
          break;
    }
    key_int_flag = true;
  }
  
}


void readSwitches()
{
  if (digitalRead(pin_Switch_START) == HIGH)
    activateVentilatorOperation = 1;
  else
    activateVentilatorOperation = 0;
}