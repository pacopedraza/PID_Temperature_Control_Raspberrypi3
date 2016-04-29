#include <bcm2835.h>
#include <stdio.h>

// Asignacion de GPIO disponible para pin de PWM
#define PIN RPI_GPIO_P1_12
// Canal 0 para PWM
#define PWM_CHANNEL 0
// Controla el rango maximo del PWM
#define RANGE 600

// Constante de temperatura donde queremos regular
#define TEMP 25

// Cabecera de funcion para lectura de ADC
float a2dRead (int canal);

// Cabecera de funcion para control PID
float PID_Controller (float set_point, float measured_value);

// Variables del PID, se ajustan Kp, Ki y Kd a valores determinados mediante prueba y error.
float actual_error, error_previous, P, I, D, Kp=0.5, Ki=0.01, Kd=0.5;


int main(int argc, char **argv)
{
    if (!bcm2835_init())
    {
		printf("Falla en inicializacion de bcm2835_init. Estas corriendo como root??\n");
		return 1;
    }

    if (!bcm2835_spi_begin())
    {
		printf("Falla en inicializacion de bcm2835_spi_begin. Estas corriendo como root??\n");
		return 1;
    }

    //*********************************************** Seccion de inicializacion del PWM *******************************************************
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_ALT5);			// Poner el GPIO a modo ALT5, para permitir que el Canal 0 de PWM salga de ese pin
    bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16);	// Divisor de reloj puesto a 16.
    bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1);				// Con un divisor de 16 y un rango de 1024, la frecuencia de pulsacion de un PWM,
    bcm2835_pwm_set_range(PWM_CHANNEL, RANGE);				// sera de 1.2MHz/600 = 2KHz, buena para hacer driving de un motor de DC.
    //*****************************************************************************************************************************************

	//*********************************************** Seccion de inicializacion del SPI *******************************************************
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);		// Default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);						// Default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536);	// Default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);						// Default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);		// Default
    //*****************************************************************************************************************************************

    float volt_0;	//Variable para lecturas de voltaje del sensor LM35.
    int canal_0;	//Variable para lectura del canal 0 del ADC MCP3202.

	while (1) //loop infinito de medicion
	{
		float pid_control;
		int pwm_value;
		float temp_read;

		canal_0	= a2dRead(0);		//Valor de la lectura de funcion ADC en canal 0.
    	volt_0	= canal_0*5.0/4096;   //Voltaje de entrada en CH0, el cual tiene un Vref=5V y 12 bits de granularidad.
    	temp_read	= volt_0*100;	//Conversion de voltaje a temperatura (el LM35 tiene una relacion de 10mV por 1 grado Celsius)

    	pid_control = PID_Controller(TEMP, temp_read);
    	pwm_value = (pid_control*-1) + 400; //conversion a valor para PWM
    	printf("El valor del PWM es %i y la temperatura es %.2f \r:  ",  pwm_value, temp_read);

    	if ((pwm_value>400) &&( pwm_value<4000))		//Rango ajustado para hacer funcionar el ventilador
    		bcm2835_pwm_set_data(PWM_CHANNEL,pwm_value);
    	else if (pwm_value<400)							//Si el valor del de conversion para el PWM es menor de 400, entonces se apaga el ventilador
    		bcm2835_pwm_set_data(PWM_CHANNEL,0);
	}

	bcm2835_spi_end();
	bcm2835_close();
	return 0;
}


float a2dRead (int canal)
    {
	    int a2dVal;
    	char dato[3];

    	dato[0]=1;  // Primer byte transmitido
    	dato[1]=( ((canal +2) << 6)); // Segundo Byte transmitido
    	dato [2]=0; // Tercer Byte transmitido

    	bcm2835_spi_transfern(dato, sizeof(dato));

    	// Leyendo los datos
    	a2dVal = 0;
    	a2dVal = (dato[1]&0b00001111)<<8; // Byte mï¿½s significativo
    	a2dVal |=  (dato[2] & 0xff);  // Byte menos significativo

    	// buf will now be filled with the data that was read from the slave

    	return a2dVal;
    }

//This is the PID definition, Kp, Ki and Kd are proportional, integral and differential gains
//float PID_Out = Kp * actual_error + Ki * SUM(previous_errors) + Kd * (actual_error - last_error);
//Define the error
//float actual_error = set_point - measured_value;

float PID_Controller (float set_point, float measured_value)
	{
		error_previous = actual_error;  //error_previous holds the previous error
		actual_error = set_point - measured_value;
		// PID
		P  = actual_error;   //Current error
		I += error_previous;  //Sum of previous errors
		D  = actual_error - error_previous;  //Difference with previous error
		return Kp*P + Ki*I + Kd*D;   //adjust Kp, Ki, Kd empirically or by using online method such as ZN
	}
