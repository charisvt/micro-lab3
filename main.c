#include "main.h"
#include "stdio.h"

#define BUFF_SIZE 128
#define DHT11_PIN PA_0
#define TOUCH_SENSOR_PIN PA_1
#define LED_PIN PB_0

#define MODE_B_LED_BLINK_INTERVAL_MS 1000 // Blink interval for Mode B alert LED (total period)

#define NO_HW_TESTING 1 // Set to 1 to disable actual hardware reads and use dummy data


// Application Constants
#define DEFAULT_PASSPHRASE "1234"
#define AEM_SUM_DEFAULT_INTERVAL_S 5 // Default interval if AEM parsing for sum fails
#define AEM_SUM_MIN_INTERVAL_S 2     // Minimum interval for sensor read after AEM sum calculation
#define AEM_MAX_LEN 6 // 5 chars + null terminator
#define INPUT_LINE_MAX_LEN 32 // Max length for UART input lines

#define TEMP_ALERT_THRESHOLD 25
#define HUMIDITY_ALERT_THRESHOLD 60
#define TEMP_PANIC_THRESHOLD 35
#define HUMIDITY_PANIC_THRESHOLD 80
#define CONSECUTIVE_MEASUREMENTS_FOR_ALERT_CLEAR 5
#define CONSECUTIVE_MEASUREMENTS_FOR_PANIC 3

#define MIN_READ_INTERVAL_S 2
#define MAX_READ_INTERVAL_S 10
#define DEFAULT_READ_INTERVAL_S 2

// Enums
typedef enum {
  MODE_A_NORMAL,
  MODE_B_ALERT
} SystemMode;

typedef enum {
  DISPLAY_TEMP,
  DISPLAY_HUMIDITY,
  DISPLAY_BOTH
} DisplayPreference;

typedef enum {
  DHT11_OK = 0,
  DHT11_ERR_TIMEOUT_NO_LOW_RESPONSE,
  DHT11_ERR_TIMEOUT_NO_HIGH_RESPONSE,
  DHT11_ERR_TIMEOUT_NO_DATA_START,
  DHT11_ERR_TIMEOUT_READING_BYTE,
  DHT11_ERR_CHECKSUM
} DHT11_StatusTypeDef;

// System State Structure
typedef struct {
  char entered_passphrase[INPUT_LINE_MAX_LEN]; // This field is not directly used by handle_login_sequence anymore, consider removal if not needed elsewhere
  char aem_id[AEM_MAX_LEN];
  bool is_logged_in;
  // int login_attempts; // Removed as per user request

  SystemMode current_mode;
  DisplayPreference display_preference;
  uint32_t read_interval_ms;
  uint32_t last_read_time_ms;

  uint8_t last_temp_integral;
  uint8_t last_temp_decimal;
  uint8_t last_rh_integral;
  uint8_t last_rh_decimal;
  uint8_t last_checksum;
  DHT11_StatusTypeDef last_dht_status;

  int profile_changes_count;
  bool led_state;

  int consecutive_high_temp_alert_count;
  int consecutive_high_humidity_alert_count;
  int consecutive_panic_temp_count;
  int consecutive_panic_humidity_count;

} SystemState_t;

SystemState_t g_system_state; // Global system state variable
static Queue rx_queue; // RX queue for UART
static char msg_buffer[BUFF_SIZE]; // General purpose message buffer for sprintf

static volatile uint32_t system_ms_counter = 0;
static volatile bool g_touch_sensor_pressed_flag = false; // Set by Touch Sensor ISR

// State for Mode B LED blinking and recovery
static uint32_t g_mode_b_led_blink_last_toggle_ms = 0;
static uint8_t g_consecutive_normal_readings_mode_b = 0;
static bool g_mode_b_led_state = false;
static uint8_t g_touch_press_count = 0; // Counter for touch sensor presses

void timer_1ms_callback(void) { 
  system_ms_counter++;
}

void uart_rx_isr(uint8_t rx_data) { 
  if (queue_is_full(&rx_queue)) { 
    return;
  }
  queue_enqueue(&rx_queue, rx_data); 
}

// ISR for the touch sensor
void touch_sensor_isr(int status) {
	(void)status;
  g_touch_sensor_pressed_flag = true;
}

int get_line_from_uart(char* buffer, int max_len);

// Helper function to read a byte from DHT11
// Returns 0 on success, -1 on timeout
static int dht11_read_byte_raw(uint8_t *byte) {
  uint8_t i;
  uint16_t timeout_counter;
  *byte = 0; 

  for (i = 0; i < 8; i++) {
    timeout_counter = 0;
    while (gpio_get(DHT11_PIN) == 0) {
      timeout_counter++;
      delay_us(1);
      if (timeout_counter > 75) return -1; 
    }

    timeout_counter = 0;
    while (gpio_get(DHT11_PIN) == 1) {
      timeout_counter++;
      delay_us(1);
      if (timeout_counter > 90) return -1; 
    }

    *byte <<= 1; 
    if (timeout_counter > 6) { // Adjusted threshold based on MEMORY[35ca59b2-92e1-409e-a7d0-69b9bda474be]
      *byte |= 1;
    }
  }
  return 0; 
}


DHT11_StatusTypeDef DHT11_Read(uint8_t* rh_int, uint8_t* rh_dec, uint8_t* temp_int, uint8_t* temp_dec, uint8_t* checksum_val) {
#if NO_HW_TESTING
  // Simulate a successful read with dummy data
  *rh_int = 55;    // Dummy Humidity Integer Part
  *rh_dec = 0;     // Dummy Humidity Decimal Part
  *temp_int = 24;  // Dummy Temperature Integer Part
  *temp_dec = 5;   // Dummy Temperature Decimal Part
  *checksum_val = (*rh_int + *rh_dec + *temp_int + *temp_dec) & 0xFF; // Calculate dummy checksum
  return DHT11_OK;
#else
  // Original hardware-dependent code follows
  uint8_t dht_data_bytes[5];
  uint16_t timeout_counter;

  uint32_t primask_state = __get_PRIMASK();
  __disable_irq();

  gpio_set_mode(DHT11_PIN, Output);
  gpio_set(DHT11_PIN, 0); 
  delay_ms(18); 
  gpio_set(DHT11_PIN, 1); 
  delay_us(30); 
  gpio_set_mode(DHT11_PIN, PullUp);
  delay_us(10); 

  timeout_counter = 0;
  while (gpio_get(DHT11_PIN) == 1) {
    timeout_counter++;
    delay_us(1);
    if (timeout_counter > 100) {
      if (!primask_state) __enable_irq();
      return DHT11_ERR_TIMEOUT_NO_LOW_RESPONSE;
    }
  }
  timeout_counter = 0;
  while (gpio_get(DHT11_PIN) == 0) {
    timeout_counter++;
    delay_us(1);
    if (timeout_counter > 100) {
      if (!primask_state) __enable_irq();
      return DHT11_ERR_TIMEOUT_NO_HIGH_RESPONSE;
    }
  }
  timeout_counter = 0;
  while (gpio_get(DHT11_PIN) == 1) {
    timeout_counter++;
    delay_us(1);
    if (timeout_counter > 100) {
      if (!primask_state) __enable_irq();
      return DHT11_ERR_TIMEOUT_NO_DATA_START;
    }
  }

  for (int i = 0; i < 5; i++) {
    if (dht11_read_byte_raw(&dht_data_bytes[i]) != 0) {
      if (!primask_state) __enable_irq();
      return DHT11_ERR_TIMEOUT_READING_BYTE;
    }
  }

  if (!primask_state) {
    __enable_irq();
  }

  uint8_t calculated_checksum = dht_data_bytes[0] + dht_data_bytes[1] + dht_data_bytes[2] + dht_data_bytes[3];
  if (calculated_checksum != dht_data_bytes[4]) {
    return DHT11_ERR_CHECKSUM;
  }

  *rh_int = dht_data_bytes[0];
  *rh_dec = dht_data_bytes[1];
  *temp_int = dht_data_bytes[2];
  *temp_dec = dht_data_bytes[3];
  *checksum_val = dht_data_bytes[4];

  return DHT11_OK;
#endif // NO_HW_TESTING
}

// Function to handle the login sequence
// Loops indefinitely until login is successful, then returns true.
bool handle_login_sequence(void) {
  char local_input_buffer[INPUT_LINE_MAX_LEN]; // Local buffer for passphrase

  while (!g_system_state.is_logged_in) { // Loop indefinitely until logged in
    uart_print("Enter passphrase: ");
    uart_print("> "); // Print prompt
    get_line_from_uart(local_input_buffer, INPUT_LINE_MAX_LEN);

    if (strcmp(local_input_buffer, DEFAULT_PASSPHRASE) == 0) {
      uart_print("Passphrase correct.\r\n");
      uart_print("Enter AEM ID (up to 5 chars): ");
      uart_print("> "); // Print prompt
      get_line_from_uart(g_system_state.aem_id, AEM_MAX_LEN);
      g_system_state.aem_id[AEM_MAX_LEN - 1] = '\0'; // Ensure null-termination
            
      g_system_state.is_logged_in = true;
      uart_print("Login successful. Welcome, AEM: ");
      uart_print(g_system_state.aem_id);
      uart_print("\r\n");
      return true; // Login successful
    } else {
      uart_print("Incorrect passphrase. Please try again.\r\n");
    }
  }
}

// Helper function to get a line of input from UART
// Returns number of characters read (excluding null terminator)
// Buffer will be null-terminated. Echoes input, handles backspace.
// IMPORTANT: This function is BLOCKING until a line is entered.
int get_line_from_uart(char* buffer, int max_len) {
  int char_count = 0;
  uint8_t c_byte; // Use uint8_t for dequeued char

  while (char_count < (max_len - 1)) {
    if (!queue_is_empty(&rx_queue)) {
      if (queue_dequeue(&rx_queue, &c_byte)) { // Correct usage of queue_dequeue
        char c = (char)c_byte; // Cast to char for comparison and storage

        if (c == '\r' || c == '\n') { // Enter pressed
          uart_print("\r\n");
          break;
        }
        else if (c == '\b' || c == 127) { // Backspace (ASCII DEL)
          if (char_count > 0) {
            char_count--;
            uart_tx('\b'); // Move cursor back
            uart_tx(' ');  // Erase character on terminal
            uart_tx('\b'); // Move cursor back again
          }
        }
        else if (c >= ' ' && c <= '~') { // Printable characters
          buffer[char_count++] = c;
          uart_tx(c); // Echo character - Correct usage of uart_tx
        }
      }
    } else {
      // Queue is empty, so we're waiting for input.
      // Add a small delay to reduce CPU load during this busy-wait.
      delay_ms(1);
    }
  }
  buffer[char_count] = '\0'; // Null-terminate the string
  return char_count;
}

void init_system_state(void) {
  g_system_state.is_logged_in = false;
  strcpy(g_system_state.aem_id, "N/A");
  g_system_state.current_mode = MODE_A_NORMAL;
  g_system_state.display_preference = DISPLAY_BOTH;
  g_system_state.read_interval_ms = DEFAULT_READ_INTERVAL_S * 1000;
  g_system_state.last_read_time_ms = 0;
  g_system_state.profile_changes_count = 0;
  g_system_state.last_temp_integral = 0;
  g_system_state.last_temp_decimal = 0;
  g_system_state.last_rh_integral = 0;
  g_system_state.last_rh_decimal = 0;
  g_system_state.last_checksum = 0;
  g_system_state.last_dht_status = DHT11_OK;
  g_system_state.consecutive_high_temp_alert_count = 0;
  g_system_state.consecutive_high_humidity_alert_count = 0;
  g_system_state.consecutive_panic_temp_count = 0;
  g_system_state.consecutive_panic_humidity_count = 0;
  g_system_state.led_state = false;
}

void print_main_menu(void) {
  uart_print("\r\n--- Main Menu ---\r\n");
  uart_print("OPTIONS:\r\n");
  uart_print("  a) Decrease data read frequency (-1s, min 2s)\r\n"); // Corrected 'a' and 'b' descriptions
  uart_print("  b) Increase data read frequency (+1s, max 10s)\r\n");
  uart_print("  c) Change display (Temp/Humidity/Both)\r\n");
  uart_print("  d) Print latest values and system state\r\n");
  uart_print("  status) Print detailed system status\r\n");
  uart_print("Enter command: \r\n");
}

int main(void) {
  // Hardware and System Initialization
  queue_init(&rx_queue, BUFF_SIZE); // Initialize RX queue
  uart_init(115200);          // Initialize UART
  uart_set_rx_callback(uart_rx_isr); // Set the UART receive callback function
  uart_enable();                     // Enable UART operation (if required by driver)

  // Initialize a 1ms system timer
  timer_init(1000); // 1000us = 1ms interval
  timer_set_callback(timer_1ms_callback); // Set the timer callback function
  timer_enable();                         // Enable the timer

  gpio_set_mode(TOUCH_SENSOR_PIN, Input); // Set as Input (sensor drives high)
  gpio_set_trigger(TOUCH_SENSOR_PIN, Falling);
  gpio_set_callback(TOUCH_SENSOR_PIN, touch_sensor_isr); 
 

  // Initialize LED Pin (PB0) for output
  gpio_set_mode(LED_PIN, Output);
  gpio_set(LED_PIN, false); // Start with LED off

  __enable_irq(); // Enable global interrupts after peripheral setup

  init_system_state(); // Initialize global system state

  uart_print("System Initializing...\r\n");
  delay_ms(100); // Short delay

  handle_login_sequence(); // Loops until successful login

  print_main_menu();
  g_system_state.last_read_time_ms = system_ms_counter; // Initialize for first read timing

  while (1) {
    // --- Handle Touch Sensor Press ---
    if (g_touch_sensor_pressed_flag) {
      g_touch_press_count++;

      if (g_touch_press_count >= 3) {
        g_touch_press_count = 0; // Reset counter
        int new_interval_s = AEM_SUM_DEFAULT_INTERVAL_S; // Default if AEM parsing fails
        int aem_len = strlen(g_system_state.aem_id);

        if (aem_len > 0) {
          char units_char = g_system_state.aem_id[aem_len - 1];
          char tens_char = (aem_len >= 2) ? g_system_state.aem_id[aem_len - 2] : '0'; // Treat as '0' if AEM has only 1 digit

          if (units_char >= '0' && units_char <= '9' && tens_char >= '0' && tens_char <= '9') {
            // Both characters are digits. This covers aem_len >= 2 and aem_len == 1 (where tens_char is '0')
            new_interval_s = (tens_char - '0') + (units_char - '0');
          } 
          // If not valid digits, new_interval_s remains AEM_SUM_DEFAULT_INTERVAL_S
        }
        
        // Clamp the interval to a minimum sensible value for DHT11
        if (new_interval_s < AEM_SUM_MIN_INTERVAL_S) {
             new_interval_s = AEM_SUM_MIN_INTERVAL_S;
        }
        // Max possible sum is 9+9 = 18. This is a reasonable max interval, no explicit upper clamp for now.

        g_system_state.read_interval_ms = (uint32_t)new_interval_s * 1000;
        g_system_state.profile_changes_count++; // Count this as a profile change

        sprintf(msg_buffer, "\r\n[INFO] Sensor read interval updated to %d seconds (from AEM ID: %s).\r\n", new_interval_s, g_system_state.aem_id);
        uart_print(msg_buffer);
        // Re-print menu or current prompt if necessary
        if (g_system_state.current_mode == MODE_A_NORMAL && g_system_state.is_logged_in) {
            uart_print("Enter command: \r\n");
        }
      }

      // Existing logic to switch to MODE_B_ALERT (happens on every touch)
      if (g_system_state.current_mode != MODE_B_ALERT) { // Prevent re-triggering if already in Mode B by touch
        g_system_state.current_mode = MODE_B_ALERT;
        g_consecutive_normal_readings_mode_b = 0;
        g_mode_b_led_state = false; // Ensure Mode B starts with its LED logic fresh
        gpio_set(LED_PIN, g_mode_b_led_state); // Turn off LED initially for Mode B
        g_mode_b_led_blink_last_toggle_ms = system_ms_counter; // Sync blink timer
        uart_print("\r\nTouch sensor pressed. Entering Alert Mode B.\r\n");
      }
      g_touch_sensor_pressed_flag = false; // Consume the flag
    }

    // --- Alert Mode B Logic ---
    if (g_system_state.current_mode == MODE_B_ALERT) {
      // Check conditions for blinking: Temp > 25C AND Hum > 60%
      if (g_system_state.last_dht_status == DHT11_OK && 
          g_system_state.last_temp_integral > TEMP_ALERT_THRESHOLD && 
          g_system_state.last_rh_integral > HUMIDITY_ALERT_THRESHOLD) {
        
        g_consecutive_normal_readings_mode_b = 0; // Reset normal readings counter
        // Blink LED every 1 second (0.5s on, 0.5s off)
        if ((system_ms_counter - g_mode_b_led_blink_last_toggle_ms) >= (MODE_B_LED_BLINK_INTERVAL_MS / 2)) {
          g_mode_b_led_state = !g_mode_b_led_state;
          gpio_set(LED_PIN, g_mode_b_led_state);
          g_mode_b_led_blink_last_toggle_ms = system_ms_counter;
        }
      } else if (g_system_state.last_dht_status == DHT11_OK) { // Values are within limits (or DHT error)
        // Turn LED off if it was blinking due to Mode B high conditions
        if (g_mode_b_led_state) {
          g_mode_b_led_state = false;
          gpio_set(LED_PIN, g_mode_b_led_state);
        }
        g_consecutive_normal_readings_mode_b++;
        if (g_consecutive_normal_readings_mode_b >= 5) {
          g_system_state.current_mode = MODE_A_NORMAL;
          g_consecutive_normal_readings_mode_b = 0; // Reset for next time
          // Ensure LED is off when exiting Mode B.
          gpio_set(LED_PIN, false);
          uart_print("\r Exiting Alert Mode B. System returned to Normal Mode A.\r\n");
        }
      } else {
        // DHT11 read error while in Mode B, turn off LED as a safe state for this mode's blinking
        if (g_mode_b_led_state) {
          g_mode_b_led_state = false;
          gpio_set(LED_PIN, g_mode_b_led_state);
        }
      }
    }

    // --- Periodic Sensor Reading --- 
    if ((system_ms_counter - g_system_state.last_read_time_ms) >= g_system_state.read_interval_ms) {
      g_system_state.last_dht_status = DHT11_Read(
        &g_system_state.last_rh_integral,
        &g_system_state.last_rh_decimal,
        &g_system_state.last_temp_integral,
        &g_system_state.last_temp_decimal,
        &g_system_state.last_checksum
      );
      g_system_state.last_read_time_ms = system_ms_counter;

      if (g_system_state.last_dht_status == DHT11_OK) {
        if (g_system_state.display_preference == DISPLAY_TEMP) {
          sprintf(msg_buffer, "Sensor: Temp %d.%dC\r\n",
            g_system_state.last_temp_integral, g_system_state.last_temp_decimal);
          uart_print(msg_buffer);
        }
        else if (g_system_state.display_preference == DISPLAY_HUMIDITY) {
          sprintf(msg_buffer, "Sensor: Hum %d.%d%%\r\n",
            g_system_state.last_rh_integral, g_system_state.last_rh_decimal);
          uart_print(msg_buffer);
        } else { // DISPLAY_BOTH or any other case defaults to both
          sprintf(msg_buffer, "Sensor: Temp %d.%dC, Hum %d.%d%%\r\n",
            g_system_state.last_temp_integral, g_system_state.last_temp_decimal,
            g_system_state.last_rh_integral, g_system_state.last_rh_decimal);
          uart_print(msg_buffer);
        }
        // Panic condition checks
        if (g_system_state.last_temp_integral > TEMP_PANIC_THRESHOLD) {
          g_system_state.consecutive_panic_temp_count++;
        } else {
          g_system_state.consecutive_panic_temp_count = 0;
        }

        if (g_system_state.last_rh_integral > HUMIDITY_PANIC_THRESHOLD) {
          g_system_state.consecutive_panic_humidity_count++;
        } else {
          g_system_state.consecutive_panic_humidity_count = 0;
        }

        if (g_system_state.consecutive_panic_temp_count >= CONSECUTIVE_MEASUREMENTS_FOR_PANIC ||
            g_system_state.consecutive_panic_humidity_count >= CONSECUTIVE_MEASUREMENTS_FOR_PANIC) {
          uart_print("\r\nPANIC MODE\r\n");
          // Note: NVIC_SystemReset() is a hard reset. UART transmission might not complete.
          // For critical applications, ensuring message delivery might require more complex handling.
          NVIC_SystemReset(); 
        }
      } else {
        sprintf(msg_buffer, "Sensor: Error reading DHT11 - Status %d\r\n", g_system_state.last_dht_status);
        uart_print(msg_buffer);
      }
    }

    // --- UART Command Handling ---
    if (!queue_is_empty(&rx_queue)) {
      uart_print("> "); 
      char command_buffer[INPUT_LINE_MAX_LEN];
      get_line_from_uart(command_buffer, INPUT_LINE_MAX_LEN);

      if (strlen(command_buffer) > 0) {
        if (strcmp(command_buffer, "status") == 0) {
          sprintf(msg_buffer, "--- System Status ---\r\n"); uart_print(msg_buffer);
          sprintf(msg_buffer, "Mode: %s\r\n", g_system_state.current_mode == MODE_A_NORMAL ? "A (Normal)" : "B (Alert)"); uart_print(msg_buffer);
          sprintf(msg_buffer, "Display: %s\r\n", 
            g_system_state.display_preference == DISPLAY_TEMP ? "Temperature" : 
            (g_system_state.display_preference == DISPLAY_HUMIDITY ? "Humidity" : "Both")); uart_print(msg_buffer);
          sprintf(msg_buffer, "Read Interval: %u ms\r\n", g_system_state.read_interval_ms); uart_print(msg_buffer);
          sprintf(msg_buffer, "Last Temp: %d.%d C, Last Hum: %d.%d %% (Checksum: %02X, Status: %d)\r\n", 
            g_system_state.last_temp_integral, g_system_state.last_temp_decimal, 
            g_system_state.last_rh_integral, g_system_state.last_rh_decimal, 
            g_system_state.last_checksum, g_system_state.last_dht_status);
          uart_print(msg_buffer);
          sprintf(msg_buffer, "Profile Changes: %d\r\n", g_system_state.profile_changes_count); uart_print(msg_buffer);
          sprintf(msg_buffer, "LED State: %s\r\n", g_system_state.led_state ? "ON" : "OFF"); uart_print(msg_buffer);
          uart_print("---------------------\r\n");
        } else {
          char cmd_char = command_buffer[0];
          switch (cmd_char) {
            case 'a': { // Decrease interval
              uint32_t current_interval_s = g_system_state.read_interval_ms / 1000;
              if (current_interval_s > MIN_READ_INTERVAL_S) {
                current_interval_s--;
                g_system_state.read_interval_ms = current_interval_s * 1000;
                g_system_state.profile_changes_count++;
                sprintf(msg_buffer, "Read interval decreased to %u s\r\n", current_interval_s);
              } else {
                sprintf(msg_buffer, "Read interval already at minimum (%d s)\r\n", MIN_READ_INTERVAL_S);
              }
              uart_print(msg_buffer);
              break;
            }
            case 'b': { // Increase interval
              uint32_t current_interval_s = g_system_state.read_interval_ms / 1000;
              if (current_interval_s < MAX_READ_INTERVAL_S) {
                current_interval_s++;
                g_system_state.read_interval_ms = current_interval_s * 1000;
                g_system_state.profile_changes_count++;
                sprintf(msg_buffer, "Read interval increased to %u s\r\n", current_interval_s);
              } else {
                sprintf(msg_buffer, "Read interval already at maximum (%d s)\r\n", MAX_READ_INTERVAL_S);
              }
              uart_print(msg_buffer);
              break;
            }
            case 'c': {
              g_system_state.display_preference = (DisplayPreference)(((int)g_system_state.display_preference + 1) % 3);
              g_system_state.profile_changes_count++;
              sprintf(msg_buffer, "Display preference changed to: %s\r\n", 
                g_system_state.display_preference == DISPLAY_TEMP ? "Temperature Only" :
                (g_system_state.display_preference == DISPLAY_HUMIDITY ? "Humidity Only" : "Temperature & Humidity"));
              uart_print(msg_buffer);
              break;
            }
            case 'd': {
              sprintf(msg_buffer, "--- Latest Values & State ---\r\n"); uart_print(msg_buffer);
              sprintf(msg_buffer, "Mode: %s\r\n", g_system_state.current_mode == MODE_A_NORMAL ? "A (Normal)" : "B (Alert)"); uart_print(msg_buffer);
              sprintf(msg_buffer, "Last Temp: %d.%d C, Last Hum: %d.%d %% (Checksum: %02X, Status: %d)\r\n", 
                g_system_state.last_temp_integral, g_system_state.last_temp_decimal, 
                g_system_state.last_rh_integral, g_system_state.last_rh_decimal, 
                g_system_state.last_checksum, g_system_state.last_dht_status);
              uart_print(msg_buffer);
              sprintf(msg_buffer, "Read Interval: %u ms\r\n", g_system_state.read_interval_ms); uart_print(msg_buffer);
              sprintf(msg_buffer, "Display Pref: %s\r\n", 
                g_system_state.display_preference == DISPLAY_TEMP ? "Temp" : 
                (g_system_state.display_preference == DISPLAY_HUMIDITY ? "Hum" : "Both")); uart_print(msg_buffer);
              uart_print("---------------------------\r\n");
              break;
            }
            default: {
              sprintf(msg_buffer, "Unknown command: '%s'\r\n", command_buffer);
              uart_print(msg_buffer);
              break;
            }
          } // End of switch
        } // End of else (not 'status' command)
      } // End of if (strlen(command_buffer) > 0)
    } // End of if (!queue_is_empty(&rx_queue))

    // --- Other Periodic Tasks (e.g., touch sensor, LED updates, panic checks) --- 
    // To be implemented in future steps

    delay_ms(10); // Small delay to yield CPU if no pressing tasks
  } // End of while(1)
}
