LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

-- Nota: Este driver asume que el BME280 Controller está enviando
-- un valor entero donde los dos LSBs son decimales implícitos (ej: 2545 -> 25.45)

ENTITY lcd_controller IS
    PORT (
        CLK         : IN  STD_LOGIC;
        RESET_N     : IN  STD_LOGIC;
        TEMP_DATA   : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
        PRESS_DATA  : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
        HUMID_DATA  : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
        DATA_VALID  : IN  STD_LOGIC;

        LCD_RS      : OUT STD_LOGIC; -- Register Select (0: Comando, 1: Dato)
        LCD_EN      : OUT STD_LOGIC; -- Enable
        LCD_DATA    : OUT STD_LOGIC_VECTOR(3 DOWNTO 0) -- Bus de 4 bits
    );
END ENTITY lcd_controller;

ARCHITECTURE Behavioral OF lcd_controller IS
    
    -- Tipos de Estado para la FSM
    TYPE state_type IS (
        S_RESET,                -- Espera inicial de Power-up
        S_INIT_1, S_INIT_2, S_INIT_3,    -- Secuencia inicial de 8-bit a 4-bit
        S_INIT_4_4BIT_SWITCH,    -- Transición a 4-bit
        S_CONFIG_LOOP,          -- Bucle para comandos de 2-nibbles
        S_CONFIG_DONE,          -- Inicialización completa
        S_WAIT_DATA,            -- Espera por nuevos datos del BME280
        S_DISPLAY_DATA          -- Bucle para escribir los datos del sensor
    );
    SIGNAL current_state, next_state : state_type;

    -- Señales de Control de la LCD
    SIGNAL s_rs_control     : STD_LOGIC := '0';
    SIGNAL s_en_control     : STD_LOGIC := '0';
    SIGNAL s_data_bus       : STD_LOGIC_VECTOR(3 DOWNTO 0) := (others => '0');
    
    -- Señales para la FSM de Escritura (manejo de los 2 nibbles)
    SIGNAL s_current_byte   : STD_LOGIC_VECTOR(7 DOWNTO 0) := (others => '0');
    SIGNAL s_current_rs     : STD_LOGIC := '0';
    SIGNAL s_byte_sent      : STD_LOGIC := '0'; -- '1' cuando el byte (2 nibbles) fue enviado

    -- Señales para Conversión de Datos (Corregidas y persistentes)
    SIGNAL s_temp_val_int	: INTEGER RANGE 0 TO 1000000 := 0;
    SIGNAL s_humid_val_int : INTEGER RANGE 0 TO 1000000 := 0;
    SIGNAL s_press_val_int : INTEGER RANGE 0 TO 1000000 := 0;
    
    -- Contador de ciclos para el pulso de Enable (E) y para los retrasos (DELAY)
    SIGNAL s_tick_counter : INTEGER RANGE 0 TO 50000000 := 0; -- Hasta 1 segundo a 50MHz
    SIGNAL s_delay_cycles : INTEGER RANGE 0 TO 50000000 := 0; 
    
    -- Contador de posición para recorrer la RAM de visualización
    SIGNAL s_display_step : INTEGER RANGE 0 TO 30 := 0;
    
    -- RAM de Comandos y Datos (similar a un buffer de visualización)
    -- Contiene: RS (bit 8), Data/Command (bits 7-0)
    TYPE RAM_TYPE IS ARRAY (0 TO 30) OF STD_LOGIC_VECTOR(8 DOWNTO 0);
    SIGNAL s_display_ram : RAM_TYPE;
    
    -- Comandos Constantes (RS, CMD)
    FUNCTION CMD (data : STD_LOGIC_VECTOR(7 DOWNTO 0)) RETURN STD_LOGIC_VECTOR IS
        BEGIN RETURN '0' & data; 
    END FUNCTION CMD;
    FUNCTION DAT (data : STD_LOGIC_VECTOR(7 DOWNTO 0)) RETURN STD_LOGIC_VECTOR IS
        BEGIN RETURN '1' & data; 
    END FUNCTION DAT;
    FUNCTION TO_ASCII(digit : INTEGER) RETURN STD_LOGIC_VECTOR IS
        BEGIN RETURN STD_LOGIC_VECTOR(to_unsigned(digit, 8) + X"30");
    END FUNCTION TO_ASCII;
    
BEGIN
    
    -- Conexiones de Salida del Driver
    LCD_RS <= s_rs_control;
    LCD_EN <= s_en_control;
    LCD_DATA <= s_data_bus;
    
    
    -- -------------------------------------------------------------------------
    -- 1. PROCESO DE CONVERSION DE DATOS (Asíncrono/Síncrono)
    -- -------------------------------------------------------------------------
    -- Convierte los vectores de entrada del BME280 a enteros para facilitar el cálculo
    P_DATA_CONVERSION: PROCESS(CLK, RESET_N)
    BEGIN
        IF RESET_N = '0' THEN
				s_temp_val_int  <= 0;
            s_humid_val_int <= 0;
            s_press_val_int <= 0;
        ELSIF rising_edge(CLK) THEN
            IF DATA_VALID = '1' THEN
                -- Asumiendo que TEMP_DATA es el valor compensado (32 bits), lo truncamos o usamos como entero.
                -- Aquí usamos el valor completo.
					 s_temp_val_int  <= to_integer(unsigned(TEMP_DATA));
                s_humid_val_int <= to_integer(unsigned(HUMID_DATA));
                s_press_val_int <= to_integer(unsigned(PRESS_DATA));
            END IF;
        END IF;
    END PROCESS P_DATA_CONVERSION;


    -- -------------------------------------------------------------------------
    -- 2. PROCESO DE COMUNICACIÓN (Envío de Nibble y Pulso E)
    -- -------------------------------------------------------------------------
    -- Esta FSM secundaria se encarga de enviar los 2 nibbles de s_current_byte.
    P_LCD_COMM: PROCESS(CLK, RESET_N)
        TYPE comm_state_type IS (S_IDLE, S_NIBBLE_MSB, S_NIBBLE_LSB, S_WAIT_E);
        VARIABLE comm_state : comm_state_type := S_IDLE;
        VARIABLE v_en_tick : INTEGER RANGE 0 TO 2 := 0; -- Contador para pulso E
		  VARIABLE v_single_nibble_init : BOOLEAN := FALSE;
    BEGIN
        IF RESET_N = '0' THEN
            s_en_control <= '0';
            s_byte_sent <= '0';
            comm_state := S_IDLE;
        ELSIF rising_edge(CLK) THEN
            s_byte_sent <= '0'; -- Reset en cada ciclo
				
				v_single_nibble_init := (current_state = S_INIT_1 OR
												current_state = S_INIT_2 OR
												current_state = S_INIT_3 OR
												current_state = S_INIT_4_4BIT_SWITCH);
            
            CASE comm_state IS
                WHEN S_IDLE =>
                    s_en_control <= '0';
                    IF s_delay_cycles = 0 AND s_display_step /= 0 THEN
                        comm_state := S_NIBBLE_MSB; -- Iniciar el envío
                    END IF;

                WHEN S_NIBBLE_MSB =>
                    s_rs_control <= s_current_rs;
                    s_data_bus <= s_current_byte(7 DOWNTO 4);
                    s_en_control <= '1';
                    v_en_tick := 0;
                    comm_state := S_WAIT_E;

                WHEN S_NIBBLE_LSB =>
                    s_rs_control <= s_current_rs;
                    s_data_bus <= s_current_byte(3 DOWNTO 0);
                    s_en_control <= '1';
                    v_en_tick := 0;
                    comm_state := S_WAIT_E;

                WHEN S_WAIT_E =>
                    v_en_tick := v_en_tick + 1;
                    IF v_en_tick >= 1 THEN -- Pulso mínimo de 1 ciclo
                        s_en_control <= '0';
								IF v_single_nibble_init THEN
									s_byte_sent <= '1';
									comm_state := S_IDLE;
								ELSE
									IF s_data_bus = s_current_byte(7 DOWNTO 4) THEN 
										comm_state := S_NIBBLE_LSB;
									ELSE
										s_byte_sent <= '1'; -- Byte completo enviado
										comm_state := S_IDLE;
									END IF;
                        END IF;
                    END IF;
            END CASE;
        END IF;
    END PROCESS P_LCD_COMM;

    
    -- -------------------------------------------------------------------------
    -- 3. PROCESO DE CONTROL DE ESTADOS PRINCIPAL (FSM)
    -- -------------------------------------------------------------------------
    P_FSM: PROCESS(CLK, RESET_N)
        VARIABLE v_temp_display : INTEGER;
        VARIABLE v_humid_display : INTEGER;
		  VARIABLE v_press_display : INTEGER;
    BEGIN
        IF RESET_N = '0' THEN
            current_state <= S_RESET;
            s_tick_counter <= 0;
            s_delay_cycles <= 0;
            s_display_step <= 0;
        ELSIF rising_edge(CLK) THEN
            s_tick_counter <= s_tick_counter + 1;
            current_state <= next_state;
				
            IF s_delay_cycles > 0 THEN
                s_delay_cycles <= s_delay_cycles - 1;
                -- Detener el avance de la FSM
            ELSIF s_byte_sent = '1' OR (current_state = S_INIT_4_4BIT_SWITCH AND s_en_control = '0') OR (current_state < S_CONFIG_LOOP AND s_display_step = 0) THEN
                -- Lógica de Transición y Carga de Bytes
                CASE current_state IS
                    
                    WHEN S_RESET =>
                        s_delay_cycles <= 5000000; -- ~100ms delay para Power-up (50MHz)
                        next_state <= S_INIT_1;
								s_display_step <= 1;
								s_current_rs <= '0';
                        s_current_byte <= X"00";

                    -- Secuencia de Inicialización (Modo 8-bit, 3 veces)
                    WHEN S_INIT_1 =>
                        s_current_byte <= X"30";
                        s_delay_cycles <= 200000; -- 4ms delay
                        next_state <= S_INIT_2;
                    
                    WHEN S_INIT_2 =>
                        s_current_byte <= X"30";
                        s_delay_cycles <= 2500; -- 50us delay
                        next_state <= S_INIT_3;

                    WHEN S_INIT_3 =>
                        s_current_byte <= X"30";
                        s_delay_cycles <= 50; -- 1us delay
                        next_state <= S_INIT_4_4BIT_SWITCH;
								
						  WHEN S_INIT_4_4BIT_SWITCH =>
								s_current_byte <= X"20";
								s_delay_cycles <= 50;
								next_state <= S_CONFIG_LOOP;
								s_display_step <= 1;
                        
                    WHEN S_CONFIG_LOOP =>
                        CASE s_display_step IS
                            WHEN 1 => -- 0x28: Function Set (2 Lines, 5x8 dots) - Ahora en modo 4-bit
                                s_current_byte <= X"28";
                                s_delay_cycles <= 50;
                            WHEN 2 => -- 0x08: Display Off (Display, Cursor, Blink Off)
                                s_current_byte <= X"08";
                                s_delay_cycles <= 50;
                            WHEN 3 => -- 0x01: Display Clear
                                s_current_byte <= X"01";
                                s_delay_cycles <= 200000;  -- 4ms delay for Clear
                            WHEN 4 => -- 0x06: Entry Mode Set (Increment, No Shift)
                                s_current_byte <= X"06";
                                s_delay_cycles <= 50;
                            WHEN 5 => -- 0x0C: Display On (Display On, Cursor Off, Blink Off)
                                s_current_byte <= X"0C";
                                s_delay_cycles <= 50;
                                next_state <= S_CONFIG_DONE; -- Fin de configuración
                            WHEN OTHERS =>
                                NULL;
                        END CASE;
                        s_current_rs <= '0'; -- Todos son comandos
                        s_display_step <= s_display_step + 1;

                    WHEN S_CONFIG_DONE =>
                        s_display_step <= 0; -- Reinicia el contador para la escritura de datos
                        next_state <= S_WAIT_DATA;

                    WHEN S_WAIT_DATA =>
                        IF DATA_VALID = '1' THEN
                            -- Recargar RAM de visualización y pasar a S_DISPLAY_DATA
                            v_temp_display := s_temp_val_int;
                            v_humid_display := s_humid_val_int;
                            v_press_display := s_press_val_int;
									 
                            -- Definición de la RAM (ej. 2 líneas, 16 caracteres)
                            -- 1. Limpiar pantalla
                            s_display_ram(1) <= CMD(X"01"); 
                            
                            -- 2. Línea 1: Temperatura (Pos: 0x80)
                            s_display_ram(2) <= CMD(X"80");
                            s_display_ram(3) <= DAT(X"54"); -- T
                            s_display_ram(4) <= DAT(X"3A"); -- :
                            s_display_ram(5) <= DAT(TO_ASCII(v_temp_display / 1000 MOD 10));
                            s_display_ram(6) <= DAT(TO_ASCII(v_temp_display / 100 MOD 10));
                            s_display_ram(7) <= DAT(X"2E"); -- .
                            s_display_ram(8) <= DAT(TO_ASCII(v_temp_display / 10 MOD 10));
                            s_display_ram(9) <= DAT(TO_ASCII(v_temp_display MOD 10));
                            s_display_ram(10) <= DAT(X"43"); -- C
                            
                            -- 3. Línea 2: Humedad (Pos: 0xC0)
                            s_display_ram(11) <= CMD(X"C0");
                            s_display_ram(12) <= DAT(X"48"); -- H
                            s_display_ram(13) <= DAT(X"3A"); -- :
                            s_display_ram(14) <= DAT(TO_ASCII(v_humid_display / 1000 MOD 10));
                            s_display_ram(15) <= DAT(TO_ASCII(v_humid_display / 100 MOD 10));
                            s_display_ram(16) <= DAT(X"2E"); -- .
                            s_display_ram(17) <= DAT(TO_ASCII(v_humid_display / 10 MOD 10));
									 s_display_ram(18) <= DAT(TO_ASCII(v_humid_display MOD 10));
                            s_display_ram(19) <= DAT(X"25"); -- %
									 
									 s_display_ram(20) <= CMD(X"20");
                            s_display_ram(21) <= DAT(X"50"); -- P
                            s_display_ram(22) <= DAT(X"3A"); -- :
                            s_display_ram(23) <= DAT(TO_ASCII(v_press_display / 100000 MOD 10));
                            s_display_ram(24) <= DAT(TO_ASCII(v_press_display / 10000 MOD 10));
									 s_display_ram(25) <= DAT(TO_ASCII(v_press_display / 1000 MOD 10));
                            s_display_ram(26) <= DAT(X"2E"); -- .
                            s_display_ram(27) <= DAT(TO_ASCII(v_press_display / 100 MOD 10));
									 s_display_ram(28) <= DAT(TO_ASCII(v_press_display / 10 MOD 10));

                            s_display_step <= 1; -- Iniciar en el primer comando/dato útil
                            next_state <= S_DISPLAY_DATA;
                        ELSE
                            next_state <= S_WAIT_DATA;
                        END IF;
                        
                    WHEN S_DISPLAY_DATA =>
                        IF s_display_step < 29 THEN -- Recorrer hasta el último índice cargado
                            s_current_rs <= s_display_ram(s_display_step)(8);
                            s_current_byte <= s_display_ram(s_display_step)(7 DOWNTO 0);
                            
                            IF s_byte_sent = '1' THEN
                                s_display_step <= s_display_step + 1;
                                s_delay_cycles <= 50; -- Pequeño delay entre bytes (1us)
                            END IF;
									 next_state <= S_DISPLAY_DATA;
                        ELSE
                            s_display_step <=0; -- Reset para el próximo ciclo
                            next_state <= S_WAIT_DATA;
                        END IF;

                END CASE;
            END IF;
        END IF;
    END PROCESS P_FSM;
    
END ARCHITECTURE Behavioral;
