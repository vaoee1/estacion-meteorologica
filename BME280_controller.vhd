LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;
USE work.FE_BME280.all;

ENTITY BME280_Controller IS
    PORT(
        clk : IN STD_LOGIC;
        reset_n : IN STD_LOGIC;
        
        start_read : IN STD_LOGIC;
        
        -- Interfaz i2c_master
        i2c_ena : OUT STD_LOGIC;
        i2c_rw : OUT STD_LOGIC;
        i2c_addr : OUT STD_LOGIC_VECTOR(6 DOWNTO 0);
        i2c_data_wr : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        i2c_data_rd : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
        i2c_busy : IN STD_LOGIC;
        i2c_ack_error : IN STD_LOGIC;

        -- Salidas de Datos Procesados
        data_ready : OUT STD_LOGIC;
        temperature : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
        pressure : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
        humidity : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
    );
END BME280_Controller;

ARCHITECTURE rtl OF BME280_Controller IS

    -- Declaraciones de COMPONENTES (i2c_master y Divisor) - Se mantienen, pero deben ser completas
    COMPONENT i2c_master
        PORT(
            clk : IN STD_LOGIC;
            reset_n : IN STD_LOGIC;
            ena : IN STD_LOGIC;
            addr : IN STD_LOGIC_VECTOR(6 DOWNTO 0);
            rw : IN STD_LOGIC;
            data_wr : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
            busy : OUT STD_LOGIC;
            data_rd : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
            ack_error : OUT STD_LOGIC;
            scl : INOUT STD_LOGIC;
            sda : INOUT STD_LOGIC
        );
    END COMPONENT;

    COMPONENT Divisor IS
        GENERIC ( N : INTEGER := 250); -- 250 es un buen valor para 100kHz @ 50MHz
        PORT ( clk : IN STD_LOGIC;
               div_clk : OUT STD_LOGIC );
    END COMPONENT;
    
    -- ----------------------------------------------------------------
    -- ESTADOS Y SEÑALES INTERNAS
    -- ----------------------------------------------------------------
    TYPE state_type IS (
        STATE_RESET,
        STATE_CONFIG_1, STATE_CONFIG_1_WAIT,
        STATE_CONFIG_2, STATE_CONFIG_2_WAIT,
        STATE_READ_CALIB_ADDR, STATE_READ_CALIB_ADDR_WAIT,
        STATE_READ_CALIB,
        STATE_DECODE_CALIB,
        STATE_START_MEAS, -- Solo si se usa modo forzado, sino se salta
        STATE_READ_DATA_ADDR, STATE_READ_DATA_ADDR_WAIT,
        STATE_READ_DATA,
        STATE_COMPENSATE,
        STATE_IDLE
    );

    SIGNAL edo_actual : state_type := STATE_RESET;
    SIGNAL edo_sig : state_type;

    -- Buffers y Contadores
    SIGNAL raw_cal_regs_s : raw_cal_reg; -- Vector de 32 bytes (tipo definido en FE_BME280)
	 SIGNAL cal_decode_vals_s : cal_recs;
    SIGNAL actual_cal_regs_s : actual_cal_reg; -- Vector de coeficientes decodificados (tipo definido en FE_BME280)
    SIGNAL t_fine_s : SIGNED(31 DOWNTO 0); -- Vital para compensación

    SIGNAL calib_byte_cnt : INTEGER RANGE 0 TO 32 := 0;
    SIGNAL data_byte_cnt : INTEGER RANGE 0 TO 8 := 0;
	 
	 SIGNAL humid_state_s : INTEGER RANGE 0 TO 8 := 0; 
	 SIGNAL temp_reg1_s : SIGNED(31 DOWNTO 0) := (others => '0');
	 SIGNAL temp_reg2_s : SIGNED(31 DOWNTO 0) := (others => '0');
	 SIGNAL humidity_unsigned_s : UNSIGNED(31 DOWNTO 0) := (others => '0');
	 ----------------------------------------------------------------------

    -- I2C / Buses
    SIGNAL i2c_clk : STD_LOGIC;
    SIGNAL i2c_scl : STD_LOGIC := 'Z'; -- Necesitas conectar estos pines en el Top.vhd
    SIGNAL i2c_sda : STD_LOGIC := 'Z';

    SIGNAL i2c_ena_int : STD_LOGIC := '0';
    SIGNAL i2c_rw_int : STD_LOGIC := '0';
    SIGNAL i2c_addr_int : STD_LOGIC_VECTOR(6 DOWNTO 0) := (others => '0');
    SIGNAL i2c_data_wr_int : STD_LOGIC_VECTOR(7 DOWNTO 0) := (others => '0');

    -- Almacenamiento temporal de bytes leídos (8 bytes para P, T, H)
    TYPE byte_array_t IS ARRAY (0 TO 7) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
    SIGNAL data_buf : byte_array_t := (others => (others => '0'));

    -- Temporales para datos raw ensamblados (20 bits P/T, 16 bits H)
    SIGNAL temp_raw_s : STD_LOGIC_VECTOR(19 DOWNTO 0) := (others => '0');
    SIGNAL pressure_raw_s : STD_LOGIC_VECTOR(19 DOWNTO 0) := (others => '0');
    SIGNAL humid_raw_s : STD_LOGIC_VECTOR(15 DOWNTO 0) := (others => '0');
	 
	 

BEGIN
    -- Mapeo salidas a puertos de la entidad
    i2c_ena <= i2c_ena_int;
    i2c_rw <= i2c_rw_int;
    i2c_addr <= i2c_addr_int;
    i2c_data_wr <= i2c_data_wr_int;
	 humidity <= STD_LOGIC_VECTOR(humidity_unsigned_s);

    -- Instanciaciones
    U2: Divisor
        GENERIC MAP ( N => 250 ) 
        PORT MAP ( clk => clk, div_clk => i2c_clk );

    -- El i2c_master DEBE INSTANCIARSE EN EL TOP LEVEL (o aquí si mueves los puertos físicos al Top)
    -- U1: i2c_master PORT MAP (i2c_clk, reset_n, i2c_ena_int, ...);

    ----------------------------------------------------------------
    -- PROCESO SINCRÓNICO (Actualización de Estado)
    ----------------------------------------------------------------
    PROCESS (clk, reset_n)
    BEGIN
        IF reset_n = '0' THEN
            edo_actual <= STATE_RESET;
        ELSIF rising_edge(clk) THEN
            edo_actual <= edo_sig;
        END IF;
    END PROCESS;

    ----------------------------------------------------------------
    -- PROCESO COMBINACIONAL (Lógica de Control de la FSM)
    ----------------------------------------------------------------
    PROCESS (edo_actual, start_read, i2c_busy, i2c_data_rd, i2c_ack_error, calib_byte_cnt, data_byte_cnt, actual_cal_regs_s, temp_raw_s, pressure_raw_s, humid_raw_s, t_fine_s)
        
        -- Variables locales para cálculos combinacionales (se reinician en cada ciclo)
		  VARIABLE v_humid_raw : STD_LOGIC_VECTOR(15 DOWNTO 0);
		  VARIABLE v_state_in : INTEGER;
		  VARIABLE v_temp_in : SIGNED(31 DOWNTO 0);
		  VARIABLE v_temp2_in : SIGNED(31 DOWNTO 0);
		  VARIABLE v_state_out : INTEGER;
		  VARIABLE v_temp_out : SIGNED(31 DOWNTO 0);
		  VARIABLE v_temp2_out : SIGNED(31 DOWNTO 0);
		  VARIABLE temp_vals_v : cal_temp_vals;
		  VARIABLE pressure_comp_v : unsigned(31 DOWNTO 0);

    BEGIN
        -- Valores por defecto
        edo_sig <= edo_actual;
        i2c_ena_int <= '0';
        i2c_rw_int <= '0';
        i2c_data_wr_int <= (others => '0');
        i2c_addr_int <= BME320_I2C_ADDR_SDO_0; -- 0x76 por defecto
        data_ready <= '0';

        CASE edo_actual IS
            WHEN STATE_RESET =>
                -- Reinicio de contadores y buffers
                calib_byte_cnt <= 0;
                data_byte_cnt <= 0;
                data_ready <= '0';
                IF start_read = '1' THEN
                    edo_sig <= STATE_CONFIG_1;
                ELSE
                    edo_sig <= STATE_RESET;
                END IF;

            -- -------------------- CONFIGURACIÓN --------------------
            WHEN STATE_CONFIG_1 =>
                -- Paso 1: Escribir la dirección del registro de humedad (0xF2)
                IF i2c_busy = '0' THEN
                    i2c_ena_int <= '1'; i2c_rw_int <= '0'; i2c_data_wr_int <= x"F2";
                    edo_sig <= STATE_CONFIG_1_WAIT;
                END IF;
            WHEN STATE_CONFIG_1_WAIT =>
                -- Paso 2: Escribir el dato de configuración (Ej. osrs_h = 1 -> x"01")
                IF i2c_busy = '0' THEN
                    i2c_ena_int <= '1'; i2c_rw_int <= '0'; i2c_data_wr_int <= x"01";
                    edo_sig <= STATE_CONFIG_2;
                END IF;
            
            WHEN STATE_CONFIG_2 =>
                -- Paso 1: Escribir la dirección del registro de control (0xF4)
                IF i2c_busy = '0' THEN
                    i2c_ena_int <= '1'; i2c_rw_int <= '0'; i2c_data_wr_int <= x"F4";
                    edo_sig <= STATE_CONFIG_2_WAIT;
                END IF;
            WHEN STATE_CONFIG_2_WAIT =>
                -- Paso 2: Escribir el dato de configuración (Ej. osrs_t=1, osrs_p=1, Modo Normal -> x"27")
                IF i2c_busy = '0' THEN
                    i2c_ena_int <= '1'; i2c_rw_int <= '0'; i2c_data_wr_int <= x"27";
                    edo_sig <= STATE_READ_CALIB_ADDR;
                END IF;
            
            -- -------------------- LECTURA DE CALIBRACIÓN --------------------
            WHEN STATE_READ_CALIB_ADDR =>
                -- Enviar dirección de inicio (0x88)
                IF i2c_busy = '0' THEN
                    i2c_ena_int <= '1'; i2c_rw_int <= '0'; i2c_data_wr_int <= x"88";
                    edo_sig <= STATE_READ_CALIB_ADDR_WAIT;
                END IF;
            WHEN STATE_READ_CALIB_ADDR_WAIT =>
                IF i2c_busy = '0' THEN
                    calib_byte_cnt <= 0;
                    edo_sig <= STATE_READ_CALIB;
                END IF;
            WHEN STATE_READ_CALIB =>
                -- Lectura secuencial de 32 bytes
                IF i2c_busy = '0' AND calib_byte_cnt < 32 THEN
                    i2c_ena_int <= '1'; i2c_rw_int <= '1';
                    
                    -- Almacenar el byte leído (i2c_data_rd se actualiza al final de la transferencia)
                    raw_cal_regs_s(calib_byte_cnt) <= i2c_data_rd;
                    calib_byte_cnt <= calib_byte_cnt + 1;
                    
                    IF calib_byte_cnt = 31 THEN -- El contador va de 0 a 31
                        edo_sig <= STATE_DECODE_CALIB;
                    ELSE
                        edo_sig <= STATE_READ_CALIB;
                    END IF;
                END IF;
                
            WHEN STATE_DECODE_CALIB =>
                -- Decodificar coeficientes (combinacional)
                actual_cal_regs_s <= decode_comp_registers(cal_decode_vals_s,raw_cal_regs_s);
                edo_sig <= STATE_START_MEAS;

            -- -------------------- LECTURA DE DATOS --------------------
            WHEN STATE_START_MEAS =>
                -- Si se usa modo normal, este estado es solo una transición.
                edo_sig <= STATE_READ_DATA_ADDR;

            WHEN STATE_READ_DATA_ADDR =>
                -- Enviar dirección de inicio de datos (0xF7)
                IF i2c_busy = '0' THEN
                    i2c_ena_int <= '1'; i2c_rw_int <= '0'; i2c_data_wr_int <= READ_SENSOR_ADDR; -- x"F7"
                    edo_sig <= STATE_READ_DATA_ADDR_WAIT;
                END IF;
            WHEN STATE_READ_DATA_ADDR_WAIT =>
                IF i2c_busy = '0' THEN
                    data_byte_cnt <= 0;
                    edo_sig <= STATE_READ_DATA;
                END IF;
				WHEN STATE_READ_DATA =>
						 -- Leer 8 bytes secuenciales
						IF i2c_busy = '0' AND data_byte_cnt < 8 THEN
						  i2c_ena_int <= '1'; i2c_rw_int <= '1';
							  
						  -- Guardar el byte leído
						  data_buf(data_byte_cnt) <= i2c_data_rd;
						  data_byte_cnt <= data_byte_cnt + 1;
						  
						  IF data_byte_cnt = 7 THEN
									-- Ensamblaje de datos crudos (CRUCIAL):
								pressure_raw_s <= data_buf(0) & data_buf(1) & data_buf(2)(7 DOWNTO 4);
								temp_raw_s <= data_buf(3) & data_buf(4) & data_buf(5)(7 DOWNTO 4);
								humid_raw_s <= data_buf(6) & data_buf(7);
								
								-- **INICIALIZACIÓN DE COMPENSACIÓN (CORREGIDO):**
								humid_state_s <= 0;
								temp_reg1_s <= (others => '0');
								temp_reg2_s <= (others => '0');
									
								edo_sig <= STATE_COMPENSATE;
							 ELSE
								edo_sig <= STATE_READ_DATA;
						  END IF;
						END IF;		 

            -- -------------------- COMPENSACIÓN --------------------
            WHEN STATE_COMPENSATE =>
                            
					v_humid_raw := humid_raw_s; 
					v_state_in := humid_state_s;
					v_temp_in := temp_reg1_s;
					v_temp2_in := temp_reg2_s;
					
					-- Lógica combinacional de P/T (se hace solo en el primer estado del subproceso)
					IF humid_state_s = 0 THEN
						 -- 1. Compensar Temperatura y obtener t_fine
						 temp_vals_v := TempRawToActual(temp_raw_s, actual_cal_regs_s(0), actual_cal_regs_s(1), actual_cal_regs_s(2));
						 
						 -- 2. Compensar Presión
						 pressure_comp_v := PressureRawToActual(pressure_raw_s, actual_cal_regs_s(3), actual_cal_regs_s(4), actual_cal_regs_s(5), actual_cal_regs_s(6), actual_cal_regs_s(7), actual_cal_regs_s(8), actual_cal_regs_s(9), actual_cal_regs_s(10), actual_cal_regs_s(11), temp_vals_v.t_fine);
						 
						 -- Asignar las salidas T y P (combinacional) y el factor t_fine
						 temperature <= STD_LOGIC_VECTOR(temp_vals_v.temp_actual);
						 pressure <= STD_LOGIC_VECTOR(pressure_comp_v);
						 t_fine_s <= temp_vals_v.t_fine;
					END IF;

					-- 3. Iteración de Humedad (8 ciclos)
					IF humid_state_s < 8 THEN
						 -- Llamada al PROCEDURE HumidRawToActual con todas las variables requeridas
						 HumidRawToActual( 
							  v_humid_raw,
							  actual_cal_regs_s(12), actual_cal_regs_s(13), actual_cal_regs_s(14),
							  actual_cal_regs_s(15), actual_cal_regs_s(16), actual_cal_regs_s(17),
							  t_fine_s,
							  v_state_in, v_temp_in, v_temp2_in,   -- Entradas (Variables locales)
							  v_state_out, v_temp_out, v_temp2_out, -- Salidas (Variables locales)
							  humidity_unsigned_s -- Salida final (Signal)
						 );
						 
						 -- Actualizar señales de estado/temporales para el siguiente ciclo de clk
						 humid_state_s <= v_state_out;
						 temp_reg1_s <= v_temp_out;
						 temp_reg2_s <= v_temp2_out;
						 
						 edo_sig <= STATE_COMPENSATE; -- Repetir el estado en el siguiente flanco de clk
						 
					ELSE -- humid_state_s = 8 (Finalizado)
						 data_ready <= '1';
						 edo_sig <= STATE_IDLE;
					END IF;
            -- -------------------- IDLE --------------------
            WHEN STATE_IDLE =>
                data_ready <= '1';
                IF start_read = '1' THEN
                    edo_sig <= STATE_CONFIG_1; -- Reiniciar el proceso
                ELSE
                    edo_sig <= STATE_IDLE;
                END IF;

            WHEN OTHERS =>
                edo_sig <= STATE_RESET;
        END CASE;
    END PROCESS;

END ARCHITECTURE rtl;