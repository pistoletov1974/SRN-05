/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dac.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "max7219.h"
#include "main.h"
#include "TM_stm32_hd44780.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t time[4];
uint8_t z_state=0;
uint8_t period_step_up=0;
uint8_t period_step_down=0;
uint16_t coil_counter=0;
uint8_t  active_line=0;
uint8_t  elapsed=0; 
float step,real_step_f;
char buf[20];
uint32_t delay_counter_l,delay_counter_r,delay_counter;
uint32_t delay_counter_setup;
uint32_t delay_counter_plus,delay_counter_minus;
uint32_t delay_counter_up,delay_counter_down;
uint16_t coil_break;
uint16_t real_step; // step with error dividing


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
 int8_t write_to_backup_sram( uint8_t *data, uint16_t bytes, uint16_t offset );
 int8_t read_from_backup_sram( uint8_t *data, uint16_t bytes, uint16_t offset );
 uint8_t Crc8(uint8_t *pcBlock, uint8_t len);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000
#define VALVE_OFF        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
#define VALVE_ON        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)


struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
      if (DEMCR & TRCENA)
				{
        while (ITM_Port32(0) == 0);
        ITM_Port8(0) = ch;
        }
  return(ch);
        
}







/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
   uint32_t data; 
   uint8_t prev_state=0;
   uint8_t pressed=0; 
   uint8_t pressed_up=0;
   uint8_t pressed_down=0;   
   uint8_t pressed_up_prev=0;
   uint8_t pressed_down_prev=0;     
   uint8_t holded=0; 
   uint8_t holded_up=0;
   uint8_t holded_down=0;
   uint8_t presed_start_r=0;
   uint8_t presed_start_l=0;
   uint8_t holded_start_r=0;
   uint8_t holded_start_l=0;
   uint8_t presed_start_r_prev=0;
   uint8_t presed_start_l_prev=0;    
   uint8_t done=0,up=0;
   uint8_t crc, crc2;
   uint32_t speed;
   uint32_t speed_brake;
   
   typedef struct {
   uint16_t  coil;
   uint16_t   speed;
   uint8_t   divider;
   float     step;
   float     length;        
   }  program_t;    
   program_t program; 
   program_t program_max;
   program_t program_min;   


   
   typedef enum { 
   IDLE=0x00,                      
   SETUP,
   RUN,
   EMERGENCY    
   } states;
   states run_state, run_state_prev;

    
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  // enable preload for autoreload register 
//  (&htim3)->Instance->CR1|=(TIM_CR1_ARPE);
  (&htim5)->Instance->CR1|=(TIM_CR1_ARPE);
   
  

  /* USER CODE BEGIN 2 */
    DWT_Init();
    HAL_Delay(300);
    init_max7219(14);
    step=0.29;
    run_state=IDLE;
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
    printf("Hello from MCU via SWO\n");
    
    
    
    AT_HD44780_CursorOn();
    AT_HD44780_BlinkOn();
    
    
    // start-stop motor for initial break
    AT_HD44780_Init(20, 4);
    AT_HD44780_Puts(0, 0, "Запуск");
    HAL_Delay(1000);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,120); 
	__HAL_TIM_SetAutoreload(&htim3,250);
      speed=200;            
    _Set_Motor_freq(speed);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    _Motor_Break_off();
    _Motor_Start();
    HAL_Delay(500);
    _Motor_Break();
	_Motor_Start_off();
    
    
    
    AT_HD44780_Puts(0, 0, "Кол. витков  ");
    AT_HD44780_Puts(0, 1, "Шаг  ");
    AT_HD44780_Puts(0, 2, "Длинна  ");
    AT_HD44780_Puts(0, 3, "Cкорость  ");
    // setup max & min values of programm
    program_max.coil = 9999;                          
    program_min.coil =1;
    program_max.speed=90;
    program_min.speed=1;
    program_max.step=1.5;
    program_min.step=0.02;
    
    program.coil=0;
    program.step=0;
    
   
	

	//AT_HD44780_PutCustom(19,1, 0xc8);

	

	 
	 
		
	 

	 printf("%d %d %d\n", SystemCoreClock,HAL_RCC_GetPCLK1Freq(),HAL_RCC_GetPCLK2Freq());
     //initialize settings from sram memory
     crc=read_from_backup_sram((uint8_t*)&program, sizeof(program),0x01 );
     printf("%04x",crc);
     crc2=Crc8((uint8_t*)&program,sizeof(program));
     printf("%04x",crc2);
     
     if (crc!=crc2) 
      { program.step=0.2;
        program.coil=150;
        program.length=30;
        program.speed=50;
        program.divider=25;
      }
     program_max.speed   =  (uint16_t) (25/program.step-3); 
     program_max.speed=(program_max.speed>99)?99:program_max.speed;
     sprintf(buf,"%5d",program.coil);
     AT_HD44780_Puts(14,0,buf);
     sprintf(buf,"%5.2f",program.step);
     AT_HD44780_Puts(14,1,buf);
     sprintf(buf,"%5.2f",program.length);
     AT_HD44780_Puts(14,2,buf);     
     sprintf(buf,"%5d",program.speed);
     AT_HD44780_Puts(14,3,buf);
    
     run_state_prev=IDLE; 
      _STEPPER_DISABLE();
     displayNumberHigh(program.coil);
     displayNumberLow(0); 
     _Motor_Break();
      HAL_Delay(1);
     _Motor_Break_off();
	 _Motor_Start_off();
      run_state=IDLE;
    
 
      
      
	 // 18 equals 0.28 mm step tim1 use for dividing 
	 //initialize divider for stepper 

	 
	 
	 // tim3 using for freq generation for owen speed control

   
     //HAL_TIM_Base_Start_IT(&htim6);
	 
	 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
        
	// setup mode
  
    if (run_state==SETUP) { 

    _RED_LED_ON();
        
    // down button
    if  (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_3)==GPIO_PIN_RESET) 
    {
        pressed_down=1;
   
    }   else pressed_down=0;
    
        
    if (  (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_3)==GPIO_PIN_RESET) && (pressed_down_prev==0))   
    {
            // button presed start counters 
            delay_counter_down=0;
            holded_down=1;
            done=0;
    }
    // check holded key
    if ((pressed_down==0)&&(holded_down==1) ) holded_down=0;
    //check short press
    if ((delay_counter_down>10)&&(holded_down==1) && (done==0)) 
    {
        if (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_3)==GPIO_PIN_RESET)
            {
        AT_HD44780_PutCustom(19,active_line,' ');
        active_line=(active_line<3)?active_line+1:0;
        if (active_line==2) active_line++;        
        AT_HD44780_PutCustom(19,active_line, 0xc8);
        done=1;    
           } 
    }
    //check long press
    
        if ( (delay_counter_down>1000) && (holded_down==1)&&(done==1) )
    {  
        program.divider= (uint8_t) (5/program.step);
        write_to_backup_sram((uint8_t*)&program, sizeof(program),0x01);
        AT_HD44780_PutCustom(19,active_line,0x20);         // clear pointer sign
        run_state=IDLE;
        
        printf("run_state-%d",run_state);
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
        printf("exti disabled");
        holded_down=0;
        holded_up=0;
        _RED_LED_OFF();
        displayNumberHigh(program.coil);
        real_step= ((500/program.divider));
        displayNumberLow(real_step);
         printf("real step=%d",real_step);
       // displayNumberLow(program.divider);
    }
    
    pressed_down_prev=pressed_down;
   // down buttons end 
    
    
    // up button
    if  (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_4)==GPIO_PIN_RESET) 
    {
        pressed_up=1;
   
    }   else pressed_up=0;
    
        
    if (  (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_4)==GPIO_PIN_RESET) && (pressed_up_prev==0))   
    {
            // button presed start counters 
            delay_counter_up=0;
            holded_up=1;
            up=0;
    }
    // check holded key
    if ((pressed_up==0)&&(holded_up==1) ) holded_up=0;
    //check short press
    if ((delay_counter_up>10)&&(holded_up==1) && (up==0)) 
    {
        if (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_4)==GPIO_PIN_RESET)
            {
        AT_HD44780_PutCustom(19,active_line,' ');
        active_line=(active_line>0)?active_line-1:3;
        if (active_line==2) active_line--;        
        AT_HD44780_PutCustom(19,active_line, 0xc8);
        up=1;    
           } 
    }
    //check long press
    
        if ( (delay_counter_up>1000) && (holded_up==1)&&(up==1) )
    {  
        program.divider= (uint8_t) (5/program.step);
        write_to_backup_sram((uint8_t*)&program, sizeof(program),0x01);
       
        AT_HD44780_PutCustom(19,active_line,0x20);
        run_state=IDLE;
        printf("run_state-%d",run_state);
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
        printf("exti disabled");
        holded_down=0;
        holded_up=0;
        
        _RED_LED_OFF();
        displayNumberHigh(program.coil);
        real_step=500/program.divider;
        displayNumberLow(real_step);
        printf("real step=%d",real_step);
      //  displayNumberLow(program.divider);
    }
    
    pressed_up_prev=pressed_up;
   // up buttons end     
    







    
        
	if (period_step_up==1) {
        
	    period_step_up=0;
    switch (active_line)
    {
        case 0:
            program.coil= (program.coil<program_max.coil)?program.coil+1:program.coil;
            sprintf(buf,"%5d",program.coil);
            AT_HD44780_Puts(14,active_line,"     ");
	        AT_HD44780_Puts(14,active_line,buf);
            program.length=program.coil*program.step;
            sprintf(buf,"%6.2f",program.length);
            AT_HD44780_Puts(13,2,"      ");
            AT_HD44780_Puts(13,2,buf);
            
        break;
        case 1:
            program.step= (program.step<program_max.step)?program.step+(float)0.01:program.step;
            sprintf(buf,"%5.2f",program.step);
            AT_HD44780_Puts(14,active_line,"     ");
	        AT_HD44780_Puts(14,active_line,buf);
            program_max.speed = (uint16_t) (25/program.step-3);
            program_max.speed=(program_max.speed>99)?99:program_max.speed;
            if (program.speed>=program_max.speed) 
                {
                    program.speed=program_max.speed;
                    sprintf(buf,"%5d",program.speed);
                    AT_HD44780_Puts(14,3,"     ");
	                AT_HD44780_Puts(14,3,buf);
                
                }
            program.length=program.coil*program.step;
            sprintf(buf,"%6.2f",program.length);
            AT_HD44780_Puts(13,2,"      ");
            AT_HD44780_Puts(13,2,buf);                
            
        break;
        case 3:  
            program.speed = (program.speed<=program_max.speed)?program.speed+1:program.speed;
            sprintf(buf,"%5d",program.speed);
            AT_HD44780_Puts(14,active_line,"     ");
	        AT_HD44780_Puts(14,active_line,buf);            
        break;
        default: break;

                
     }//switch         
        //speed up timer 
        data=__HAL_TIM_GetAutoreload(&htim5);
        if(data>30) data=data-10;
        __HAL_TIM_SetAutoreload(&htim5,data);
		
	} // if step_up
    
	if (period_step_down==1) {
	    period_step_down=0;
            switch (active_line)
    {
        case 0:
            program.coil= (program.coil>program_min.coil)?program.coil-1:program.coil;
            sprintf(buf,"%5d",program.coil);
            AT_HD44780_Puts(14,active_line,"     ");
	        AT_HD44780_Puts(14,active_line,buf);
            program.length=program.coil*program.step;
            sprintf(buf,"%6.2f",program.length);
            AT_HD44780_Puts(13,2,"       ");
            AT_HD44780_Puts(13,2,buf);        
        break;
        case 1:
            program.step= (program.step>program_min.step)?program.step-(float)0.01:program.step;
            sprintf(buf,"%5.2f",program.step);
            AT_HD44780_Puts(14,active_line,"     ");
	        AT_HD44780_Puts(14,active_line,buf);
            program_max.speed = (uint16_t) (25/program.step-3);
            program_max.speed=(program_max.speed>99)?99:program_max.speed;
            if (program.speed>=program_max.speed) 
                {
                    program.speed=program_max.speed;
                    sprintf(buf,"%5d",program.speed);
                    AT_HD44780_Puts(14,3,"     ");
	                AT_HD44780_Puts(14,3,buf);
                
                }
            program.length=program.coil*program.step;
            sprintf(buf,"%6.2f",program.length);
            AT_HD44780_Puts(13,2,"       ");
            AT_HD44780_Puts(13,2,buf);                
        break;
        case 3:  
            program.speed = (program.speed>program_min.speed)?program.speed-1:program.speed;
            sprintf(buf,"%5d",program.speed);
            AT_HD44780_Puts(14,active_line,"     ");
	        AT_HD44780_Puts(14,active_line,buf); 
        break;
        default:break;
        
     }//switch   
        //speed up timer
        data=__HAL_TIM_GetAutoreload(&htim5);
        if(data>50) data=data-10;
        __HAL_TIM_SetAutoreload(&htim5,data);
       // HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_3);
        printf("%d /n",data);

	
	}     //if step_down
	
}  // end setup mode
   
    // idle mode 

if (run_state==IDLE)  {
//	 butons pressed together
     _GREEN_LED_ON();
   if ((HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_3)==GPIO_PIN_RESET) &&  ( HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_4)==GPIO_PIN_RESET) )           
        pressed=1;    
           else pressed=0; 
   
   
   
   if ((pressed==1) && (prev_state==0) )
   {    
           
            //how much time buttons pressed?

            delay_counter_setup=0;  
            holded=1;
       } 
   if ((pressed==0) && (holded==1)) holded=0;
             // delay counter updates in the systick handler func
       
       
                if ((delay_counter_setup>1000)  && (holded==1))
            {
                 printf("holded setup\n");
                 if ((HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_3)==GPIO_PIN_RESET) && ( HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_4)==GPIO_PIN_RESET) ) 
                      {
                      
                delay_counter_setup=0;
                pressed_down_prev=1;
                pressed_up_prev=1;          
                run_state=SETUP;
                _GREEN_LED_OFF();
                active_line=0;
                printf("second stage\n");
                printf("state-%d",run_state);          
                AT_HD44780_PutCustom(19,active_line, 0xc8);
                HAL_NVIC_EnableIRQ(EXTI1_IRQn);
                HAL_NVIC_EnableIRQ(EXTI2_IRQn);
                holded=0;
                     }  
            }
         prev_state=pressed;
            
        
       // start/direction buttons 
            
            
       // start right     
       if (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_5)==GPIO_PIN_RESET) 
           
              presed_start_r=1;
               else presed_start_r=0;   
       if ((presed_start_r==1) && (presed_start_r_prev==0)) 
       {
         delay_counter_r=0;
         holded_start_r=1;
       }           
       if ((presed_start_r==0) && (holded_start_r==1))  holded_start_r=0;
       
       if ( (delay_counter_r>500) && (holded_start_r==1) ) 
       {
             printf("st.R");
             holded_start_r=0;
             run_state=RUN;
            _GREEN_LED_OFF();
           _STEPPER_RIGHT();       
       }
       
       //start left
       if (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==GPIO_PIN_RESET) 
           
              presed_start_l=1;
               else presed_start_l=0;   
       if ((presed_start_l==1) && (presed_start_l_prev==0)) 
       {
         delay_counter_l=0;
         holded_start_l=1;
       }           
       if ((presed_start_l==0) && (holded_start_l==1))  holded_start_l=0;
       
       if ( (delay_counter_l>500) && (holded_start_l==1) ) 
       {
             printf("st L");
             holded_start_l=0;
             run_state=RUN;
            _GREEN_LED_OFF();
             _STEPPER_LEFT();       
       } 


        

                
       
       
       
       
       presed_start_l_prev=presed_start_l;    
       presed_start_r_prev=presed_start_r;

        }      // end idle mode

        
        
        
      //run mode  
        
    if (run_state==RUN)  {
        _BLUE_LED_ON();
        if (run_state_prev!=RUN) 
        {
            
         __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,120); 
	     __HAL_TIM_SetAutoreload(&htim3,250);
           
            
         speed= (program.speed<30)?program.speed*25:program.speed*7;            
         _Set_Motor_freq(speed);
	       HAL_TIM_Base_Start(&htim3);
	       HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
          //  (&htim3)->Instance->CR1|=(TIM_CR1_ARPE);
          
	    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,program.divider); 
	    __HAL_TIM_SetAutoreload(&htim1,program.divider);
	      HAL_TIM_Base_Start(&htim1);
	      HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
          coil_counter=0;  
          _STEPPER_ENABLE();  
          _Motor_Break_off();  
         // HAL_Delay(20);
          _Motor_Start();
                     
         coil_break = (uint16_t) program.coil- (uint16_t)program.speed*0.25 ;  
         printf("coil_break=%d\n",coil_break);  
         printf("divider=%d\n",program.divider);
        }    
          
  

           
      
       if (z_state==1) {
            
            
			//printf("c=%d\n", coil_counter);
			displayNumberLow(coil_counter);
			z_state=0;
		
			// test for speed changes
		    if ((coil_counter > 5)  && (coil_counter <9) )
                { 
                speed=program.speed*50;                    
			    _Set_Motor_freq( speed);
                printf("speed_max=%d\n",speed); 
                printf("prescaller=%d\n",(&htim3)->Instance->PSC);                     
				}
                           
	
            // breaking mode near stop     
				if (coil_counter == coil_break )
					{ 
                    
                    speed_brake=program.speed*6; 
                    speed_brake=(speed_brake>500)?speed_brake:500; 
                     
					_Set_Motor_freq(speed_brake);
                    printf("stop=%d , %d \n",coil_counter,speed_brake);

				}
					

                // write prescaller if button pressed
          
                
					

					
				if (coil_counter >= program.coil ) {
                    _Motor_Start_off(); 				
                    _Motor_Break();
                 
					
                    run_state=IDLE;
                    _BLUE_LED_OFF();
                    _STEPPER_DISABLE();
                    _Set_Motor_freq(500);
				}   	



                
			}
       
     //check emergency stop       
     if (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6)==GPIO_PIN_RESET)

            {
                
                    _Motor_Start_off();
                    _Motor_Break();
                                                   
					 run_state=IDLE;
                    _BLUE_LED_OFF();
                    _STEPPER_DISABLE();  
                     HAL_Delay(1);
                    _Set_Motor_freq(500);
            
            }
            
       
        }   // end run mode
  
       run_state_prev=run_state;          

        
        
        HAL_Delay(2);   
	}     //end while

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */


int8_t write_to_backup_sram( uint8_t *data, uint16_t bytes, uint16_t offset ) {
  const uint16_t backup_size = 0x1000;
  uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
  uint16_t i;
  uint8_t crc; 
  if( bytes + offset >= backup_size ) {
    /* ERROR : the last byte is outside the backup SRAM region */
    return -1;
  }
 
  /* disable backup domain write protection */

  /** enable the backup regulator (used to maintain the backup SRAM content in
    * standby and Vbat modes).  NOTE : this bit is not reset when the device
    * wakes up from standby, system reset or power reset. You can check that
    * the backup regulator is ready on PWR->CSR.brr, see rm p144 */


  for( i = 0; i < bytes; i++ ) {
    *(base_addr + offset + i) = *(data + i);
  }
  i++;
  crc =  Crc8(data, bytes);
   *(base_addr + offset + i) =  crc;
  //HAL_PWREx_DisableBkUpReg(); // reset PWR->CR.dbp = 0;
  return crc;
}



int8_t read_from_backup_sram( uint8_t *data, uint16_t bytes, uint16_t offset ) {
  const uint16_t backup_size = 0x1000;
  uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
  uint16_t i;
  uint8_t crc;
  uint8_t crc_sram;    
    
   
  if( bytes + offset >= backup_size ) {
    /* ERROR : the last byte is outside the backup SRAM region */
    return -1;
  }
 
  for( i = 0; i < bytes; i++ ) {
    *(data + i) = *(base_addr + offset + i);
  }
  i++;
  crc_sram =  *(base_addr + offset + i);
  crc= Crc8(data, bytes);
  
  return crc_sram;
}






/*
  Name  : CRC-8
  Poly  : 0x31    x^8 + x^5 + x^4 + 1
  Init  : 0xFF
  Revert: false
  XorOut: 0x00
  Check : 0xF7 ("123456789")
  MaxLen: 15 байт(127 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
uint8_t Crc8(uint8_t *pcBlock, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t i;

    while (len--)
    {
        crc ^= *pcBlock++;

        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
