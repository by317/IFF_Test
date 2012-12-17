/*
 * main.c
 */

#include "DSP2802x_Device.h"
#include "DSP2802x_Examples.h"
#include "easy2802x_sci_v7.3.h"
#include "DSP2802x_GlobalPrototypes.h"
#include "Piccolo_PWM.h"
#include "IQmathLib.h"

#define VIN_SCALE		430		//Scaling for Q15 Format
#define VIN_OFFSET 		0
#define VBUS_SCALE_INIT	667		//Scaling for Q15 Format
#define	I_SCALE_INIT	100		//Scaling for Q15 Format
#define I_OFFSET_INIT		47
#define MPPT_UPDATE_PERIOD_MS 500
#define Q2_PULSE_PERCENT	100
#define Q1_MAX_PULSE	300
#define DEADTIME	8
#define MAX_PERIOD	3000
#define MAX_CMD		(MAX_PERIOD + Q1_MAX_PULSE)
#define MAX_COMP	MAX_CMD*32768
#define MAX_REF_Q15	3276800;
#define TEST_GAIN	32768


//#define b0	182880
//#define b1	113670
#define b0	18290
#define b1	11370

#define EPWM3_CMPA	Q1_PULSE
#define EPWM4_CMPA	Q1_PULSE + DEADTIME
#define EPWM4_CMPB	Q1_PULSE + DEADTIME + Q2_PULSE

interrupt void pwm_int(void);
void pwm_setup(void);
void initVariables(void);
void InitAdc(void);
void SetupAdc(void);
void ms_delay(unsigned int);

int32 Input_Voltage_Q15;
int32 Bus_Voltage_Q15;
int32 Input_Current_Q15;

int32 Input_Reference_Q15;
int32 Voltage_Error_Q15;
int32 Numerator_Delay_Q15;
int32 Denominator_Delay_Q15;
int32 Compensator_Output_Q15;
int input_voltage_prescale;
int Comp_Out_Int;

unsigned int q1_pulse;
unsigned int q2_pulse;
unsigned int period_cmd;
int R_Factor;
unsigned int period;

void main(void) {
	DINT;
	InitSysCtrl();
	InitPieCtrl();
	InitAdc();
	SetupAdc();
	IER = 0x0000;
	IFR = 0x0000;
	initVariables();
	InitPieVectTable();
	easyDSP_SCI_Init();
	InitEPwm3();
	InitEPwm4();
	pwm_setup();
	EALLOW;
	PieVectTable.EPWM3_INT = &pwm_int;
	PieCtrlRegs.PIEIER3.bit.INTx3 = 0x1;
	EDIS;
	IER |= M_INT3;
	EINT;
	InitEPwm3Gpio();
	InitEPwm4Gpio();
}

interrupt void pwm_int()
{
	GpioDataRegs.GPASET.bit.GPIO2 = 1;
	input_voltage_prescale = ((int) AdcResult.ADCRESULT0 - VIN_OFFSET);
	Input_Voltage_Q15 = ((long int) input_voltage_prescale*VIN_SCALE);

	Voltage_Error_Q15 = Input_Reference_Q15 - Input_Voltage_Q15;

	Compensator_Output_Q15 = _IQ15mpy(Voltage_Error_Q15, b0) + _IQ15mpy(Numerator_Delay_Q15, b1) + Denominator_Delay_Q15;

	Compensator_Output_Q15 = _IQsat(Compensator_Output_Q15, MAX_COMP, -MAX_COMP);
	Compensator_Output_Q15 = _IQ15mpy(Compensator_Output_Q15, TEST_GAIN);

	Comp_Out_Int = ((int) _IQ15int(Compensator_Output_Q15));

	period_cmd = MAX_CMD + Comp_Out_Int;

	Numerator_Delay_Q15 = Voltage_Error_Q15;
	Denominator_Delay_Q15 = Compensator_Output_Q15;

	if (period_cmd > MAX_PERIOD)
	{
		period = MAX_PERIOD;
		R_Factor = period_cmd - MAX_PERIOD;
	}
	else
	{
		period = period_cmd;
		R_Factor = 0;
	}
	if (R_Factor > Q1_MAX_PULSE)
		R_Factor = Q1_MAX_PULSE;

	q1_pulse = Q1_MAX_PULSE - R_Factor;

	q2_pulse = ((long) (q1_pulse*Q2_PULSE_PERCENT)>>10);

	EPwm3Regs.TBPRD = period;
	EPwm4Regs.TBPRD = period;

	EPwm3Regs.CMPA.half.CMPA = q2_pulse;
	EPwm4Regs.CMPA.half.CMPA = q2_pulse + DEADTIME;
	EPwm4Regs.CMPB = q1_pulse + q2_pulse + DEADTIME;

	EPwm3Regs.ETCLR.bit.INT = 0x1;  			//Clear the Interrupt Flag
	GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  	//Acknowledge the interrupt
	return;
}

void pwm_setup()
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0x0;
	GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
	EDIS;

	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0x0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0x0;
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;
	EDIS;

	EALLOW;
	EPwm3Regs.ETSEL.bit.INTEN = 0x1;
	EPwm3Regs.ETSEL.bit.INTSEL = 0x1;
	EPwm3Regs.ETPS.bit.INTPRD = 0x1;
	EPwm3Regs.ETSEL.bit.SOCAEN = 1;
	EPwm3Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPA;
	EPwm3Regs.ETPS.bit.SOCAPRD = 0x1;
	EDIS;
}

void initVariables (void)
{
	period_cmd = MAX_CMD;
	Input_Voltage_Q15 = 0;
	Bus_Voltage_Q15 = 0;
	Input_Current_Q15 = 0;
	Input_Reference_Q15 = MAX_REF_Q15;
	Voltage_Error_Q15 = 0;
	Numerator_Delay_Q15 = 0;
	Denominator_Delay_Q15 = 0;
	Compensator_Output_Q15 = 0;
	input_voltage_prescale = 0;
	Comp_Out_Int = 0;
}

void SetupAdc(void)
{
	EALLOW;
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN0 = 1;
	////Input Voltage & Current Sampling on SOC0
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x09;
	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x3;
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 0x6;
	EDIS;
}

void ms_delay(unsigned int wait_time)
{
	volatile unsigned int i;
	CpuTimer1Regs.PRD.all = 0x0000EA60;
	for (i=0; i < wait_time; i++)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;
		CpuTimer1Regs.TCR.bit.TRB = 1;
		CpuTimer1Regs.TCR.bit.TSS = 0;
		while (CpuTimer1Regs.TCR.bit.TIF == 0)
		{
		}
	}
}

void InitAdc(void)
{
    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
        (*Device_cal)();
        EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the DSP2802x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
    EDIS;

    ms_delay(10);         // Delay before converting ADC channels
}
