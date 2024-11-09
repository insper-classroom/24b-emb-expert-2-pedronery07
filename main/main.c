#include <stdio.h>
#include "btstack.h" //funcionalidades de Bluetooth.
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "server_common.h"

#define HEARTBEAT_PERIOD_MS 1000

static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static void heartbeat_handler(struct btstack_timer_source *ts) {
    static uint32_t counter = 0;
    counter++;

    // Update the temp every 10s
    if (counter % 10 == 0) {
        poll_temp();
        if (le_notification_enabled) {
            att_server_request_can_send_now_event(con_handle);
        }
    }

    // Invert the led
    static int led_on = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    // Restart timer
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

int main() {
    stdio_init_all();

    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    // Inicializa a leitura do sensor de temperatura, 
    // esse sensor já vem embutido na placa, não precisamos adicionar nenhum sensor na placa.
    adc_init();
    adc_select_input(ADC_CHANNEL_TEMPSENSOR);
    adc_set_temp_sensor_enabled(true);

    l2cap_init(); // Inicializa o L2CAP (Logical Link Control and Adaptation Protocol) 
    sm_init(); // Inicializa o SM (Security Manager) do Bluetooth.

    // Inicializar o servidor ATT (Attribute Protocol)
    // com o perfil GATT definido (profile_data) e os callbacks de leitura e escrita.
    att_server_init(profile_data, att_read_callback, att_write_callback);

    // Registrar callbacks BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(packet_handler);

    // set one-shot btstack timer
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);

    // turn on bluetooth!
    hci_power_control(HCI_POWER_ON);

    while(1) {
        tight_loop_contents();

    }
    return 0;
}