LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;
USE work.FE_BME280.all; -- Para tipos de datos del BME280

ENTITY Estacion IS
    PORT(
        -- Puertos del Sistema
        CLK_IN          : IN  STD_LOGIC; -- Reloj principal del FPGA (ej. 50 MHz)
        RST_BTN_N       : IN  STD_LOGIC; -- Botón de Reset asíncrono (activo en bajo)
        START_SW        : IN  STD_LOGIC; -- Switch de encendido/inicio

        -- Pines I2C (conectados al i2c_master)
        SCL_PIN         : INOUT STD_LOGIC := 'Z';
        SDA_PIN         : INOUT STD_LOGIC := 'Z';

        -- Salida de Alarma (ej. LED o Buzzer)
        ALARM_OUT       : OUT STD_LOGIC;

        -- Interfaz LCD (Ejemplo de 4-bit)
        LCD_RS          : OUT STD_LOGIC;
        LCD_EN          : OUT STD_LOGIC;
        LCD_DATA        : OUT STD_LOGIC_VECTOR(3 DOWNTO 0)
        -- Otros pines como R/W, si es necesario
    );
END Estacion;

ARCHITECTURE rtl OF Estacion IS
    -- Declaración del componente I2C Master (asumiendo que está en un archivo separado)
    COMPONENT i2c_master IS
        PORT (
            clk         : IN  STD_LOGIC;
            reset_n     : IN  STD_LOGIC;
            ena         : IN  STD_LOGIC;
            addr        : IN  STD_LOGIC_VECTOR(6 DOWNTO 0);
            rw          : IN  STD_LOGIC;
            data_wr     : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
            busy        : OUT STD_LOGIC;
            data_rd     : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
            ack_error   : OUT STD_LOGIC;
            scl         : INOUT STD_LOGIC;
            sda         : INOUT STD_LOGIC
        );
    END COMPONENT;

    -- Declaración del componente BME280_Controller (el que acabamos de debuggear)
    COMPONENT BME280_Controller IS
        PORT (
            clk         : IN  STD_LOGIC;
            reset_n     : IN  STD_LOGIC;
            start_read  : IN  STD_LOGIC;
            i2c_ena     : OUT STD_LOGIC;
            i2c_rw      : OUT STD_LOGIC;
            i2c_addr    : OUT STD_LOGIC_VECTOR(6 DOWNTO 0);
            i2c_data_wr : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
            i2c_data_rd : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
            i2c_busy    : IN  STD_LOGIC;
            i2c_ack_error : IN STD_LOGIC;
            data_ready  : OUT STD_LOGIC;
            temperature : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
            pressure    : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
            humidity    : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
        );
    END COMPONENT;
    
    -- Declaración del componente LCD (Necesitas crear esta entidad)
    COMPONENT lcd_controller IS
        PORT (
            clk         : IN  STD_LOGIC;
            reset_n     : IN  STD_LOGIC;
            temp_data   : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
            press_data  : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
            humid_data  : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
            data_valid  : IN  STD_LOGIC;
            
            lcd_rs      : OUT STD_LOGIC;
            lcd_en      : OUT STD_LOGIC;
            lcd_data    : OUT STD_LOGIC_VECTOR(3 DOWNTO 0)
        );
    END COMPONENT;

    -- Señales de conexión
    SIGNAL s_start_read     : STD_LOGIC := '0'; -- Señal de inicio (controlada por switch)
    SIGNAL s_temp_raw       : STD_LOGIC_VECTOR(31 DOWNTO 0); -- Temperatura (salida del BME280_Controller)

    -- Señales de la interfaz I2C
    SIGNAL s_i2c_ena        : STD_LOGIC;
    SIGNAL s_i2c_rw         : STD_LOGIC;
    SIGNAL s_i2c_addr       : STD_LOGIC_VECTOR(6 DOWNTO 0);
    SIGNAL s_i2c_data_wr    : STD_LOGIC_VECTOR(7 DOWNTO 0);
    SIGNAL s_i2c_data_rd    : STD_LOGIC_VECTOR(7 DOWNTO 0);
    SIGNAL s_i2c_busy       : STD_LOGIC;
    SIGNAL s_i2c_ack_error  : STD_LOGIC;
    SIGNAL s_data_ready     : STD_LOGIC;
    
    -- Datos del sensor
    SIGNAL s_press_raw      : STD_LOGIC_VECTOR(31 DOWNTO 0);
    SIGNAL s_humid_raw      : STD_LOGIC_VECTOR(31 DOWNTO 0);
    
    -- Umbral de Alarma (Ej: 30.00 grados Celsius) - Requiere un valor en binario compensado.
    -- Asumiremos que el resultado compensado de la temperatura es un entero con dos decimales implícitos (e.g., 3000).
    CONSTANT TEMP_ALARM_THRESHOLD : SIGNED(31 DOWNTO 0) := to_signed(3000, 32); 

BEGIN

	-- 1. Instancia I2C Master
	U_I2C_MASTER : i2c_master
		 PORT MAP (
			  clk       => CLK_IN,      -- Ojo: Usar el reloj principal (CLK_IN) si el master tiene divisor interno
			  reset_n   => RST_BTN_N,
			  ena       => s_i2c_ena,
			  addr      => s_i2c_addr,
			  rw        => s_i2c_rw,
			  data_wr   => s_i2c_data_wr,
			  busy      => s_i2c_busy,
			  data_rd   => s_i2c_data_rd,
			  ack_error => s_i2c_ack_error,
			  scl       => SCL_PIN,
			  sda       => SDA_PIN
		 );

	-- 2. Instancia BME280 Controller
	U_BME280_CONTROLLER : BME280_Controller
		 PORT MAP (
			  clk         => CLK_IN,      -- El BME280_Controller usa CLK_IN para generar el i2c_clk interno
			  reset_n     => RST_BTN_N,
			  start_read  => s_start_read,
			  
			  -- Conexión a I2C Master
			  i2c_ena     => s_i2c_ena,
			  i2c_rw      => s_i2c_rw,
			  i2c_addr    => s_i2c_addr,
			  i2c_data_wr => s_i2c_data_wr,
			  i2c_data_rd => s_i2c_data_rd,
			  i2c_busy    => s_i2c_busy,
			  i2c_ack_error => s_i2c_ack_error,
			  
			  -- Salidas de Datos
			  data_ready  => s_data_ready,
			  temperature => s_temp_raw,
			  pressure    => s_press_raw,
			  humidity    => s_humid_raw
		 );

	-- 3. Instancia LCD Driver
	U_LCD_DRIVER : lcd_controller
		 PORT MAP (
			  clk         => CLK_IN,
			  reset_n     => RST_BTN_N,
			  temp_data   => s_temp_raw,
			  press_data  => s_press_raw,
			  humid_data  => s_humid_raw,
			  data_valid  => s_data_ready, -- Usar data_ready para saber cuándo actualizar el display
			  lcd_rs      => LCD_RS,
			  lcd_en      => LCD_EN,
			  lcd_data    => LCD_DATA
		 );
		 
END ARCHITECTURE RTL;