#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// LCD I2C + RTC DS1307

#define PIN_I2C_DATOS 21
#define PIN_I2C_RELOJ 22
#define BUS_I2C_PRINCIPAL I2C_NUM_0

#define DIR_LCD_I2C    0x27
#define DIR_RTC_DS1307 0x68

#define BIT_LCD_LUZ 0x08
#define BIT_LCD_ENABLE    0x04
#define BIT_LCD_RS        0x01

// RC522 SPI
#define PIN_SPI_ENTRADA 19
#define PIN_SPI_SALIDA 13
#define PIN_SPI_RELOJ  18
#define PIN_SPI_CHIP_SELECT   5
#define PIN_RFID_RESET  27

#define BUS_SPI_RFID SPI2_HOST

static spi_device_handle_t manejador_spi_rfid;

// LEDs y buzzer
#define SALIDA_LED_VERDE GPIO_NUM_16
#define SALIDA_LED_ROJO  GPIO_NUM_17
#define SALIDA_LED_AZUL  GPIO_NUM_2
#define SALIDA_BUZZER    GPIO_NUM_23

// Estado del sistema
volatile int estado_panel_habilitado = 0;
volatile int bandera_refrescar_lcd = 0;

char texto_lcd_actual[17] = "Sin mensajes";
char texto_hora_actual[9] = "00:00:00";

int64_t tiempo_ultimo_rfid = 0;
int64_t tiempo_ultimo_reloj = 0;

#define TIEMPO_ESPERA_RFID_MS 4000

// UID autorizado: TARJETA PERMITIDA
uint8_t uid_permitido[4] = {0x9C, 0x18, 0x8C, 0xBB};

// Registros RC522
#define CommandReg      0x01
#define ComIrqReg       0x04
#define ErrorReg        0x06
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define ControlReg      0x0C
#define BitFramingReg   0x0D
#define ModeReg         0x11
#define TxControlReg    0x14
#define TxASKReg        0x15
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D
#define VersionReg      0x37

#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C
#define PCD_SOFTRESET   0x0F

#define PICC_REQALL     0x52
#define PICC_ANTICOLL   0x93

// BLE NUS
#define NOMBRE_DISPOSITIVO_BLE "PanelHmi"

static uint8_t tipo_direccion_ble;

static const ble_uuid128_t uuid_servicio_nus =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

static const ble_uuid128_t uuid_caracteristica_rx =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

static const ble_uuid128_t uuid_caracteristica_tx =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static int ble_gap_event(struct ble_gap_event *evento_ble, void *argumento_ble);

// LCD
void i2c_init(void)
{
    i2c_config_t config_i2c = {
        .modo_lcd = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_DATOS,
        .scl_io_num = PIN_I2C_RELOJ,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };

    i2c_param_config(BUS_I2C_PRINCIPAL, &config_i2c);
    i2c_driver_install(BUS_I2C_PRINCIPAL, config_i2c.modo_lcd, 0, 0, 0);
}

void lcd_write_byte(uint8_t dato_i2c)
{
    i2c_master_write_to_device(BUS_I2C_PRINCIPAL, DIR_LCD_I2C, &dato_i2c, 1, pdMS_TO_TICKS(100));
}

void lcd_pulse_enable(uint8_t dato_i2c)
{
    lcd_write_byte(dato_i2c | BIT_LCD_ENABLE | BIT_LCD_LUZ);
    esp_rom_delay_us(1000);

    lcd_write_byte((dato_i2c & ~BIT_LCD_ENABLE) | BIT_LCD_LUZ);
    esp_rom_delay_us(1000);
}

void lcd_send_nibble(uint8_t medio_byte_lcd, uint8_t modo_lcd)
{
    uint8_t dato_i2c = (medio_byte_lcd & 0xF0) | modo_lcd | BIT_LCD_LUZ;
    lcd_pulse_enable(dato_i2c);
}

void lcd_send_byte(uint8_t valor_lcd, uint8_t modo_lcd)
{
    lcd_send_nibble(valor_lcd & 0xF0, modo_lcd);
    lcd_send_nibble((valor_lcd << 4) & 0xF0, modo_lcd);
}

void lcd_cmd(uint8_t comando_lcd)
{
    lcd_send_byte(comando_lcd, 0);
}

void lcd_data(uint8_t dato_i2c)
{
    lcd_send_byte(dato_i2c, BIT_LCD_RS);
}

void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_send_nibble(0x20, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_cmd(0x28);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_cmd(0x08);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_cmd(0x06);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_cmd(0x0C);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void lcd_clear(void)
{
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void lcd_set_cursor(uint8_t fila_lcd, uint8_t columna_lcd)
{
    if (fila_lcd == 0)
        lcd_cmd(0x80 + columna_lcd);
    else
        lcd_cmd(0xC0 + columna_lcd);
}

void lcd_print(const char *cadena_lcd)
{
    while (*cadena_lcd)
    {
        lcd_data(*cadena_lcd++);
    }
}

void lcd_print_16(const char *cadena_lcd)
{
    char buffer_local[17];

    for (int indice = 0; indice < 16; indice++)
    {
        if (cadena_lcd[indice] != '\0')
            buffer_local[indice] = cadena_lcd[indice];
        else
            buffer_local[indice] = ' ';
    }

    buffer_local[16] = '\0';
    lcd_print(buffer_local);
}

// RTC DS1307
int bcd_to_dec(uint8_t valor_bcd)
{
    return ((valor_bcd >> 4) * 10) + (valor_bcd & 0x0F);
}

uint8_t dec_to_bcd(int valor_bcd)
{
    return ((valor_bcd / 10) << 4) | (valor_bcd % 10);
}

void poner_hora_default(char *buffer_local)
{
    buffer_local[0] = '0';
    buffer_local[1] = '0';
    buffer_local[2] = ':';
    buffer_local[3] = '0';
    buffer_local[4] = '0';
    buffer_local[5] = ':';
    buffer_local[6] = '0';
    buffer_local[7] = '0';
    buffer_local[8] = '\0';
}

void ds1307_set_time(int hora_rtc, int minuto_rtc, int segundo_rtc)
{
    uint8_t dato_i2c[4];

    dato_i2c[0] = 0x00;
    dato_i2c[1] = dec_to_bcd(segundo_rtc);
    dato_i2c[2] = dec_to_bcd(minuto_rtc);
    dato_i2c[3] = dec_to_bcd(hora_rtc);

    i2c_master_write_to_device(BUS_I2C_PRINCIPAL, DIR_RTC_DS1307, dato_i2c, 4, pdMS_TO_TICKS(100));
}

void ds1307_get_time(char *buffer_local)
{
    uint8_t registro_rfid = 0x00;
    uint8_t dato_i2c[3] = {0};

    esp_err_t resultado_operacion = i2c_master_write_read_device(
        BUS_I2C_PRINCIPAL,
        DIR_RTC_DS1307,
        &registro_rfid,
        1,
        dato_i2c,
        3,
        pdMS_TO_TICKS(100)
    );

    if (resultado_operacion != ESP_OK)
    {
        poner_hora_default(buffer_local);
        return;
    }

    int segundo_rtc  = bcd_to_dec(dato_i2c[0] & 0x7F);
    int minuto_rtc  = bcd_to_dec(dato_i2c[1]);
    int hora_rtc = bcd_to_dec(dato_i2c[2] & 0x3F);

    if (hora_rtc > 23 || minuto_rtc > 59 || segundo_rtc > 59)
    {
        poner_hora_default(buffer_local);
        return;
    }

    buffer_local[0] = '0' + (hora_rtc / 10);
    buffer_local[1] = '0' + (hora_rtc % 10);
    buffer_local[2] = ':';
    buffer_local[3] = '0' + (minuto_rtc / 10);
    buffer_local[4] = '0' + (minuto_rtc % 10);
    buffer_local[5] = ':';
    buffer_local[6] = '0' + (segundo_rtc / 10);
    buffer_local[7] = '0' + (segundo_rtc % 10);
    buffer_local[8] = '\0';
}

// Buzzer PWM
void buzzer_init(void)
{
    ledc_timer_config_t temporizador_pwm = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&temporizador_pwm);

    ledc_channel_config_t canal_pwm = {
        .gpio_num = SALIDA_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .canal_pwm = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&canal_pwm);
}

void buzzer_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_beep(int tiempo_ms)
{
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(tiempo_ms));
    buzzer_off();
}

// Pantallas LCD
void lcd_mostrar_bloqueado(void)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Panel bloqueado");

    lcd_set_cursor(1, 0);
    lcd_print("Acerque cred.");
}

void lcd_mostrar_concedido(void)
{
    ds1307_get_time(texto_hora_actual);

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Acceso concedido");

    lcd_set_cursor(1, 0);
    lcd_print(texto_hora_actual);
}

void lcd_mostrar_denegado(void)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Acceso denegado");

    lcd_set_cursor(1, 0);
    lcd_print("UID no regist.");
}

void lcd_mostrar_estado_activo(void)
{
    ds1307_get_time(texto_hora_actual);

    lcd_clear();

    lcd_set_cursor(0, 0);
    lcd_print_16(texto_lcd_actual);

    lcd_set_cursor(1, 0);
    lcd_print(texto_hora_actual);
}

// RC522
static void rc522_write_reg(uint8_t registro_rfid, uint8_t valor_lcd)
{
    uint8_t trama_tx[2] = {
        (registro_rfid << 1) & 0x7E,
        valor_lcd
    };

    spi_transaction_t transaccion_spi = {
        .length = 16,
        .tx_buffer = trama_tx
    };

    spi_device_transmit(manejador_spi_rfid, &transaccion_spi);
}

static uint8_t rc522_read_reg(uint8_t registro_rfid)
{
    uint8_t trama_tx[2] = {
        ((registro_rfid << 1) & 0x7E) | 0x80,
        0x00
    };

    uint8_t trama_rx[2] = {0};

    spi_transaction_t transaccion_spi = {
        .length = 16,
        .tx_buffer = trama_tx,
        .rx_buffer = trama_rx
    };

    spi_device_transmit(manejador_spi_rfid, &transaccion_spi);

    return trama_rx[1];
}

static void rc522_set_bit_mask(uint8_t registro_rfid, uint8_t mascara_bits)
{
    uint8_t valor_temporal = rc522_read_reg(registro_rfid);
    rc522_write_reg(registro_rfid, valor_temporal | mascara_bits);
}

static void rc522_clear_bit_mask(uint8_t registro_rfid, uint8_t mascara_bits)
{
    uint8_t valor_temporal = rc522_read_reg(registro_rfid);
    rc522_write_reg(registro_rfid, valor_temporal & (~mascara_bits));
}

static void rc522_reset(void)
{
    gpio_set_direction(PIN_RFID_RESET, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_RFID_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(PIN_RFID_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    rc522_write_reg(CommandReg, PCD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void rc522_antenna_on(void)
{
    uint8_t valor_temporal = rc522_read_reg(TxControlReg);

    if (!(valor_temporal & 0x03))
    {
        rc522_set_bit_mask(TxControlReg, 0x03);
    }
}

static void rc522_init(void)
{
    rc522_reset();

    rc522_write_reg(TModeReg, 0x8D);
    rc522_write_reg(TPrescalerReg, 0x3E);
    rc522_write_reg(TReloadRegL, 30);
    rc522_write_reg(TReloadRegH, 0);

    rc522_write_reg(TxASKReg, 0x40);
    rc522_write_reg(ModeReg, 0x3D);

    rc522_antenna_on();
}

static int rc522_to_card(uint8_t comando_rfid, uint8_t *datos_salida_rfid, uint8_t longitud_salida_rfid,
                         uint8_t *datos_respuesta_rfid, uint16_t *longitud_respuesta_rfid)
{
    uint8_t interrupcion_esperada = 0x00;
    uint8_t cantidad_bytes_fifo;
    uint16_t indice;

    if (comando_rfid == PCD_TRANSCEIVE)
    {
        interrupcion_esperada = 0x30;
    }

    rc522_write_reg(ComIrqReg, 0x7F);
    rc522_set_bit_mask(FIFOLevelReg, 0x80);
    rc522_write_reg(CommandReg, PCD_IDLE);

    for (indice = 0; indice < longitud_salida_rfid; indice++)
    {
        rc522_write_reg(FIFODataReg, datos_salida_rfid[indice]);
    }

    rc522_write_reg(CommandReg, comando_rfid);

    if (comando_rfid == PCD_TRANSCEIVE)
    {
        rc522_set_bit_mask(BitFramingReg, 0x80);
    }

    indice = 5000;

    do
    {
        cantidad_bytes_fifo = rc522_read_reg(ComIrqReg);
        indice--;
    }
    while ((indice != 0) && !(cantidad_bytes_fifo & 0x01) && !(cantidad_bytes_fifo & interrupcion_esperada));

    rc522_clear_bit_mask(BitFramingReg, 0x80);

    if (indice == 0)
    {
        return 0;
    }

    if (rc522_read_reg(ErrorReg) & 0x1B)
    {
        return 0;
    }

    if (comando_rfid == PCD_TRANSCEIVE)
    {
        cantidad_bytes_fifo = rc522_read_reg(FIFOLevelReg);
        uint8_t bits_finales = rc522_read_reg(ControlReg) & 0x07;

        if (bits_finales)
            *longitud_respuesta_rfid = (cantidad_bytes_fifo - 1) * 8 + bits_finales;
        else
            *longitud_respuesta_rfid = cantidad_bytes_fifo * 8;

        if (cantidad_bytes_fifo > 16)
            cantidad_bytes_fifo = 16;

        for (indice = 0; indice < cantidad_bytes_fifo; indice++)
        {
            datos_respuesta_rfid[indice] = rc522_read_reg(FIFODataReg);
        }
    }

    return 1;
}

static int rc522_request(uint8_t modo_peticion, uint8_t *tipo_tarjeta)
{
    uint16_t bits_respuesta;
    uint8_t buffer_local[1];

    buffer_local[0] = modo_peticion;

    rc522_write_reg(BitFramingReg, 0x07);

    int estado_operacion = rc522_to_card(PCD_TRANSCEIVE, buffer_local, 1, tipo_tarjeta, &bits_respuesta);

    if ((estado_operacion != 1) || (bits_respuesta != 0x10))
    {
        return 0;
    }

    return 1;
}

static int rc522_anticoll(uint8_t *serie_tarjeta)
{
    uint16_t longitud_uid;
    uint8_t buffer_local[2];

    buffer_local[0] = PICC_ANTICOLL;
    buffer_local[1] = 0x20;

    rc522_write_reg(BitFramingReg, 0x00);

    int estado_operacion = rc522_to_card(PCD_TRANSCEIVE, buffer_local, 2, serie_tarjeta, &longitud_uid);

    if (estado_operacion)
    {
        uint8_t xor_verificacion = 0;

        for (int indice = 0; indice < 4; indice++)
        {
            xor_verificacion ^= serie_tarjeta[indice];
        }

        if (xor_verificacion != serie_tarjeta[4])
        {
            return 0;
        }

        return 1;
    }

    return 0;
}

// Accesos
int uid_es_autorizado(uint8_t *uid_leido)
{
    for (int indice = 0; indice < 4; indice++)
    {
        if (uid_leido[indice] != uid_permitido[indice])
        {
            return 0;
        }
    }

    return 1;
}

void acceso_concedido(void)
{
    printf("Acceso concedido\cantidad_bytes_fifo");

    lcd_mostrar_concedido();

    gpio_set_level(SALIDA_LED_ROJO, 0);
    gpio_set_level(SALIDA_LED_AZUL, 0);
    gpio_set_level(SALIDA_LED_VERDE, 1);

    buzzer_beep(500);

    vTaskDelay(pdMS_TO_TICKS(1000));

    gpio_set_level(SALIDA_LED_VERDE, 0);
    gpio_set_level(SALIDA_LED_AZUL, 1);

    estado_panel_habilitado = 1;

    strcpy(texto_lcd_actual, "Sin mensajes");
    bandera_refrescar_lcd = 1;
}

void acceso_denegado(void)
{
    printf("Acceso denegado\cantidad_bytes_fifo");

    lcd_mostrar_denegado();

    gpio_set_level(SALIDA_LED_VERDE, 0);
    gpio_set_level(SALIDA_LED_AZUL, 0);

    buzzer_on();

    for (int indice = 0; indice < 3; indice++)
    {
        gpio_set_level(SALIDA_LED_ROJO, 1);
        vTaskDelay(pdMS_TO_TICKS(300));

        gpio_set_level(SALIDA_LED_ROJO, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_off();

    gpio_set_level(SALIDA_LED_ROJO, 1);

    estado_panel_habilitado = 0;

    vTaskDelay(pdMS_TO_TICKS(1000));
    lcd_mostrar_bloqueado();
}

void cerrar_sesion(void)
{
    printf("Cerrando sesion\cantidad_bytes_fifo");

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Cerrando sesion");

    lcd_set_cursor(1, 0);
    lcd_print("Espere...");

    buzzer_beep(500);

    gpio_set_level(SALIDA_LED_AZUL, 0);
    gpio_set_level(SALIDA_LED_VERDE, 0);
    gpio_set_level(SALIDA_LED_ROJO, 1);

    estado_panel_habilitado = 0;

    strcpy(texto_lcd_actual, "Sin mensajes");
    bandera_refrescar_lcd = 0;

    vTaskDelay(pdMS_TO_TICKS(500));
    lcd_mostrar_bloqueado();
}

void configurar_salidas(void)
{
    gpio_config_t config_salidas_gpio = {
        .pin_bit_mask = (1ULL << SALIDA_LED_ROJO) |
                        (1ULL << SALIDA_LED_VERDE) |
                        (1ULL << SALIDA_LED_AZUL),
        .modo_lcd = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&config_salidas_gpio);

    gpio_set_level(SALIDA_LED_ROJO, 1);
    gpio_set_level(SALIDA_LED_VERDE, 0);
    gpio_set_level(SALIDA_LED_AZUL, 0);
}

// BLE
static int nus_rx_access_cb(uint16_t conexion_ble,
                            uint16_t atributo_ble,
                            struct ble_gatt_access_ctxt *contexto_ble,
                            void *argumento_ble)
{
    char buffer_local[64];
    uint16_t longitud_mensaje_ble = OS_MBUF_PKTLEN(contexto_ble->om);

    if (longitud_mensaje_ble >= sizeof(buffer_local))
        longitud_mensaje_ble = sizeof(buffer_local) - 1;

    memset(buffer_local, 0, sizeof(buffer_local));
    os_mbuf_copydata(contexto_ble->om, 0, longitud_mensaje_ble, buffer_local);
    buffer_local[longitud_mensaje_ble] = '\0';

    if (estado_panel_habilitado == 1)
    {
        memset(texto_lcd_actual, 0, sizeof(texto_lcd_actual));

        int longitud_texto = strlen(buffer_local);
        if (longitud_texto > 16)
            longitud_texto = 16;

        strncpy(texto_lcd_actual, buffer_local, longitud_texto);
        texto_lcd_actual[longitud_texto] = '\0';

        bandera_refrescar_lcd = 1;

        printf("Mensaje BLE recibido: %s\cantidad_bytes_fifo", texto_lcd_actual);
    }
    else
    {
        printf("Mensaje BLE ignorado porque el sistema esta bloqueado\cantidad_bytes_fifo");
    }

    return 0;
}

static int nus_tx_access_cb(uint16_t conexion_ble,
                            uint16_t atributo_ble,
                            struct ble_gatt_access_ctxt *contexto_ble,
                            void *argumento_ble)
{
    return 0;
}

static const struct ble_gatt_svc_def servicios_gatt[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_servicio_nus.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &uuid_caracteristica_rx.u,
                .access_cb = nus_rx_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &uuid_caracteristica_tx.u,
                .access_cb = nus_tx_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0}
        },
    },
    {0}
};

static void ble_advertise(void)
{
    struct ble_gap_adv_params parametros_anuncio;
    struct ble_hs_adv_fields campos_anuncio;

    memset(&campos_anuncio, 0, sizeof(campos_anuncio));

    campos_anuncio.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    campos_anuncio.name = (uint8_t *)NOMBRE_DISPOSITIVO_BLE;
    campos_anuncio.name_len = strlen(NOMBRE_DISPOSITIVO_BLE);
    campos_anuncio.name_is_complete = 1;

    ble_gap_adv_set_fields(&campos_anuncio);

    memset(&parametros_anuncio, 0, sizeof(parametros_anuncio));

    parametros_anuncio.conn_mode = BLE_GAP_CONN_MODE_UND;
    parametros_anuncio.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(tipo_direccion_ble, NULL, BLE_HS_FOREVER,
                      &parametros_anuncio, ble_gap_event, NULL);
}

static int ble_gap_event(struct ble_gap_event *evento_ble, void *argumento_ble)
{
    switch (evento_ble->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            if (evento_ble->connect.estado_operacion == 0)
            {
                printf("BLE conectado\cantidad_bytes_fifo");
            }
            else
            {
                printf("Fallo conexion BLE\cantidad_bytes_fifo");
                ble_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf("BLE desconectado\cantidad_bytes_fifo");
            ble_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_advertise();
            break;

        default:
            break;
    }

    return 0;
}

static void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &tipo_direccion_ble);
    ble_advertise();
}

void ble_host_task(void *parametro_tarea)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_init(void)
{
    esp_err_t resultado_operacion = nvs_flash_init();

    if (resultado_operacion == ESP_ERR_NVS_NO_FREE_PAGES || resultado_operacion == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    nimble_port_init();

    ble_svc_gap_device_name_set(NOMBRE_DISPOSITIVO_BLE);

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(servicios_gatt);
    ble_gatts_add_svcs(servicios_gatt);

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);

    printf("BLE iniciado como %s\cantidad_bytes_fifo", NOMBRE_DISPOSITIVO_BLE);
}

// Main
void app_main(void)
{
    configurar_salidas();
    buzzer_init();

    i2c_init();
    lcd_init();

    // AJUSTE DE HORA:
    ds1307_set_time(10, 0, 0);

    estado_panel_habilitado = 0;
    lcd_mostrar_bloqueado();

    ble_init();

    spi_bus_config_t config_bus_spi = {
        .miso_io_num = PIN_SPI_ENTRADA,
        .mosi_io_num = PIN_SPI_SALIDA,
        .sclk_io_num = PIN_SPI_RELOJ,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t config_dispositivo_spi = {
        .clock_speed_hz = 500000,
        .modo_lcd = 0,
        .spics_io_num = PIN_SPI_CHIP_SELECT,
        .queue_size = 7
    };

    spi_bus_initialize(BUS_SPI_RFID, &config_bus_spi, SPI_DMA_CH_AUTO);
    spi_bus_add_device(BUS_SPI_RFID, &config_dispositivo_spi, &manejador_spi_rfid);

    rc522_init();

    uint8_t version_rfid = rc522_read_reg(VersionReg);

    printf("Version RC522: 0x%02X\cantidad_bytes_fifo", version_rfid);

    if (version_rfid == 0x00 || version_rfid == 0xFF)
    {
        printf("No se detecta el RC522. Revisa conexiones SPI.\cantidad_bytes_fifo");

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("Error RC522");

        lcd_set_cursor(1, 0);
        lcd_print("Revise cables");
    }
    else
    {
        printf("RC522 detectado correctamente.\cantidad_bytes_fifo");
        printf("Acerque una tarjeta...\cantidad_bytes_fifo");
    }

    while (1)
    {
        int64_t tiempo_actual_ms = esp_timer_get_time() / 1000;

        if (estado_panel_habilitado == 1)
        {
            gpio_set_level(SALIDA_LED_AZUL, 1);

            if (bandera_refrescar_lcd == 1)
            {
                bandera_refrescar_lcd = 0;
                lcd_mostrar_estado_activo();
                tiempo_ultimo_reloj = tiempo_actual_ms;
            }

            if ((tiempo_actual_ms - tiempo_ultimo_reloj) >= 1000)
            {
                tiempo_ultimo_reloj = tiempo_actual_ms;
                lcd_mostrar_estado_activo();
            }
        }

        uint8_t tipo_tag_rfid[2];
        uint8_t uid_leido[5];

        if ((tiempo_actual_ms - tiempo_ultimo_rfid) > TIEMPO_ESPERA_RFID_MS)
        {
            if (rc522_request(PICC_REQALL, tipo_tag_rfid))
            {
                if (rc522_anticoll(uid_leido))
                {
                    tiempo_ultimo_rfid = tiempo_actual_ms;

                    printf("Tarjeta detectada. UID: ");

                    for (int indice = 0; indice < 4; indice++)
                    {
                        printf("%02X ", uid_leido[indice]);
                    }

                    printf("\cantidad_bytes_fifo");

                    lcd_clear();
                    lcd_set_cursor(0, 0);
                    lcd_print("UID detectado");

                    lcd_set_cursor(1, 0);

                    char texto_uid[17];
                    const char tabla_hexadecimal[] = "0123456789ABCDEF";

                    texto_uid[0]  = tabla_hexadecimal[(uid_leido[0] >> 4) & 0x0F];
                    texto_uid[1]  = tabla_hexadecimal[uid_leido[0] & 0x0F];
                    texto_uid[2]  = ' ';
                    texto_uid[3]  = tabla_hexadecimal[(uid_leido[1] >> 4) & 0x0F];
                    texto_uid[4]  = tabla_hexadecimal[uid_leido[1] & 0x0F];
                    texto_uid[5]  = ' ';
                    texto_uid[6]  = tabla_hexadecimal[(uid_leido[2] >> 4) & 0x0F];
                    texto_uid[7]  = tabla_hexadecimal[uid_leido[2] & 0x0F];
                    texto_uid[8]  = ' ';
                    texto_uid[9]  = tabla_hexadecimal[(uid_leido[3] >> 4) & 0x0F];
                    texto_uid[10] = tabla_hexadecimal[uid_leido[3] & 0x0F];
                    texto_uid[11] = '\0';

                    lcd_print(texto_uid);

                    vTaskDelay(pdMS_TO_TICKS(700));

                    if (uid_es_autorizado(uid_leido))
                    {
                        if (estado_panel_habilitado == 0)
                        {
                            acceso_concedido();
                        }
                        else
                        {
                            cerrar_sesion();
                        }
                    }
                    else
                    {
                        if (estado_panel_habilitado == 0)
                        {
                            acceso_denegado();
                        }
                        else
                        {
                            printf("UID no autorizado ignorado en estado activo\cantidad_bytes_fifo");
                            lcd_mostrar_estado_activo();
                            gpio_set_level(SALIDA_LED_AZUL, 1);
                        }
                    }

                    vTaskDelay(pdMS_TO_TICKS(1500));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
