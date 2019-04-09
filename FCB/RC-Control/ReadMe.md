# Radio Control

Beskrivelse: Denne Unmaned Flying Vehicle kan modtage instruktioner fra jord kontrol.

##### Jordstation: Spectrum DX7
##### Modtager: Spectrum AR8000
| port  | Int/Ext |  Description |
|-------|---------|--------------| 
| PB.04 | ext     | TIM3_CH1 (Receiver PWM input) |
| PB.05 | ext     | TIM3_CH2 (Receiver PWM input) |
| PD.03 | ext | TIM2_CH1 (Receiver PWM input) |
| PD.04 | ext | TIM2_CH2 (Receiver PWM input) |
| PD.06 | ext | TIM2_CH4 (Receiver PWM input) |
| PD.07 | ext | TIM2_CH3 (Receiver PWM input) |

/** 
  * @brief  TIM Input Capture Init structure definition  
  */

typedef struct
{

	uint16_t	TIM_Channel;      /*!< Specifies the TIM channel.
                                  This parameter can be a value of @ref TIM_Channel */

	uint16_t	TIM_ICPolarity;   /*!< Specifies the active edge of the input signal.
                                  This parameter can be a value of @ref TIM_Input_Capture_Polarity */

 	uint16_t	TIM_ICSelection;  /*!< Specifies the input.
                                  This parameter can be a value of @ref TIM_Input_Capture_Selection */

	uint16_t	TIM_ICPrescaler;  /*!< Specifies the Input Capture Prescaler.
                                  This parameter can be a value of @ref TIM_Input_Capture_Prescaler */

	uint16_t	TIM_ICFilter;     /*!< Specifies the input capture filter.
                                  This parameter can be a number between 0x0 and 0xF */
} TIM_ICInitTypeDef;

## 21.3.6 PWM input mode
This mode is a particular case of input capture mode. The procedure is the same except:

	• Two ICx signals are mapped on the same TIx input.
	• These 2 ICx signals are active on edges with opposite polarity.
	• One of the two TIxFP signals is selected as trigger input and the slave mode controller is configured in reset mode.


For example, you can measure the period (in TIMx_CCR1 register) and the duty cycle (in TIMx_CCR2 register) of the PWM applied on TI1 using the following procedure (depending on CK_INT frequency and prescaler value):

	1. Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1
	register (TI1 selected).
	
	2. Select the active polarity for TI1FP1 (used both for capture in TIMx_CCR1 and counter
	clear): write the CC1P to ‘0’ and the CC1NP bit to ‘0’ (active on rising edge).
	
	3. Select the active input for TIMx_CCR2: write the CC2S bits to 10 in the TIMx_CCMR1
	register (TI1 selected).
	
	4. Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2): write the CC2P
	bit to ‘1’ and the CC2NP bit to ’0’ (active on falling edge).
	
	5. Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register
	(TI1FP1 selected).
	
	6. Configure the slave mode controller in reset mode: write the SMS bits to 100 in the
	TIMx_SMCR register.
	
	7. Enable the captures: write the CC1E and CC2E bits to ‘1 in the TIMx_CCER register.

