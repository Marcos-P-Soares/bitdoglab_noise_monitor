#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "ws2818b.pio.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include <string.h>

#define ADC_VREF 3.3f
// -------------------- Configuração do Microfone --------------------
#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)

// Parâmetros e macros do ADC.
#define ADC_CLOCK_DIV 96.f
#define SAMPLES 200  
#define ADC_ADJUST(x) (x * 3.3f / (1 << 12u) - 1.65f) 
#define ADC_MAX 3.3f
#define ADC_STEP (3.3f / 5.f)

#define abs(x) ((x < 0) ? (-x) : (x))

// -------------------- Configuração dos LEDs --------------------
#define LED_PIN 7
#define LED_COUNT 25

// -------------------- Configuração do Buzzer --------------------
#define BUZZER_PIN 21
#define BUZZER_FREQUENCY 1000  // Hz

// Definição dos pinos do I2C
const uint I2C_SDA = 14;
const uint I2C_SCL = 15;


// -------------------- Estrutura do LED --------------------
struct pixel_t {
    uint8_t G, R, B; // Formato GRB
};
typedef struct pixel_t npLED_t;

// -------------------- Variáveis Globais --------------------
static npLED_t leds[LED_COUNT]; // Buffer dos LEDs
static PIO np_pio;
static uint np_sm;
uint dma_channel;
dma_channel_config dma_cfg;
uint16_t adc_buffer[SAMPLES];

uint8_t ssd[ssd1306_buffer_length];

// -------------------- Declaração de Funções --------------------
// Controle dos LEDs
void npInit(uint pin, uint amount);
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npClear();
void npWrite();

// Controle do Buzzer
void pwm_init_buzzer(uint pin);
void beep(uint pin, uint duration_ms);

// Leitura do Microfone
void sample_mic();
float mic_power();
uint8_t get_intensity(float v);

void display_message(const char *message);
// -------------------- Implementação das Funções --------------------


void display_message(const char *message) {
    memset(ssd, 0, ssd1306_buffer_length); // Limpa o display

    char *text_lines[2];  
    text_lines[0] = strtok((char *)message, "\n");  
    text_lines[1] = strtok(NULL, "\n"); 

    int y = 0;
    for (uint i = 0; i < 2; i++) { 
        if (text_lines[i] != NULL) {
            ssd1306_draw_string(ssd, 5, y, text_lines[i]);
            y += 8;
        }
    }

    struct render_area frame_area = {
        .start_column = 0,
        .end_column = ssd1306_width - 1,
        .start_page = 0,
        .end_page = ssd1306_n_pages - 1
    };
    calculate_render_area_buffer_length(&frame_area);
    render_on_display(ssd, &frame_area);
}
/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin, uint amount) {
    printf("[LOG] Inicializando matriz de LEDs...\n");
    np_pio = pio0;
    
    uint offset = pio_add_program(np_pio, &ws2818b_program);
    
    // Toma posse de uma máquina PIO.
    np_sm = pio_claim_unused_sm(np_pio, true);

    // Inicia programa na máquina PIO obtida.
    ws2818b_program_init(np_pio, np_sm, offset, pin, 800000.f);

    // Limpa buffer de pixels.
    for (uint i = 0; i < amount; i++) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

/**
 * Atribui uma cor RGB a um LED específico.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    if (index < LED_COUNT) {
        leds[index].R = r;
        leds[index].G = g;
        leds[index].B = b;
    }
}

/**
 * Limpa o buffer de pixels (desliga todos os LEDs).
 */
void npClear() {
    for (uint i = 0; i < LED_COUNT; i++) {
        npSetLED(i, 0, 0, 0);
    }
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite() {
    for (uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, np_sm, leds[i].G);
        pio_sm_put_blocking(np_pio, np_sm, leds[i].R);
        pio_sm_put_blocking(np_pio, np_sm, leds[i].B);
    }
    sleep_us(100); // Tempo de RESET dos LEDs (conforme datasheet)
}

/**
 * Inicializa o buzzer utilizando PWM.
 */
void pwm_init_buzzer(uint pin) {
    printf("[LOG] Inicializando PWM para o buzzer...\n");
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096));
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
}

/**
 * Emite um beep no buzzer por um determinado tempo.
 */
void beep(uint pin, uint duration_ms) {
    printf("[LOG] Emitindo beep de %d ms...\n", duration_ms);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_gpio_level(pin, 2048);
    sleep_ms(duration_ms);
    pwm_set_gpio_level(pin, 0);
    sleep_ms(100);
}

/**
 * Realiza as leituras do ADC e armazena os valores no buffer.
 */
void sample_mic() {
    adc_fifo_drain();  // Limpa o FIFO do ADC.
    adc_run(false);     // Desliga o ADC antes de configurar o DMA.
    
    dma_channel_configure(dma_channel, &dma_cfg,
        adc_buffer,       // Escreve no buffer.
        &(adc_hw->fifo),  // Lê do ADC.
        SAMPLES,          // Faz SAMPLES amostras.
        true              // Liga o DMA.
    );

    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);
    adc_run(false);
}


/**
 * Calcula a potência média das leituras do ADC. (Valor RMS)
 */
float mic_power() {
    float avg = 0.f;

    // Converte os valores do ADC para tensão e calcula a potência média corretamente.
    for (uint i = 0; i < SAMPLES; ++i) {
        float voltage = (adc_buffer[i] * ADC_VREF) / (1 << 12);  // Converte para tensão (0 a 3.3V)
        float adjusted = voltage - 1.65f;  // Centraliza em torno de 0V (-1.65V a +1.65V)
        avg += adjusted * adjusted;  // Potência RMS
    }

    avg /= SAMPLES;  // Média das potências
    return sqrt(avg);  // Raiz quadrada para obter valor RMS correto
}

/**
 * Calcula a intensidade do volume registrado no microfone, de 0 a 2, usando a tensão.
 */
uint8_t get_intensity(float v) {
    if (v < 0.1f) {
      return 0;  // Baixa intensidade
    } else if (v < 0.5f) {
      return 1;  // Moderada
    } else {
      return 2;  // Alta
    }
  }
// -------------------- Função Principal --------------------

int main() {
    stdio_init_all();
    sleep_ms(5000);  // Pequeno atraso para abrir o monitor serial

    printf("[LOG] Inicializando sistema...\n");
    // Inicialização do i2c
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();
     // Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
    struct render_area frame_area = {
        .start_column = 0,
        .end_column = ssd1306_width - 1,
        .start_page = 0,
        .end_page = ssd1306_n_pages - 1
    };

    calculate_render_area_buffer_length(&frame_area);

    // zera o display inteiro
    
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    display_message("Detector \nde Ruido");

    // Inicializa Matriz de LEDs
    npInit(LED_PIN, LED_COUNT);

    // -------------------- Sinalização Inicial --------------------
    printf("[LOG] Sinalizando inicialização (LED Verde)...\n");
    for (int i = 0; i < LED_COUNT; i++) {
        npSetLED(i, 0, 255, 0);  // Verde
    }
    npWrite();
    sleep_ms(1000);

    // Apagar os LEDs antes de iniciar o monitoramento
    printf("[LOG] Apagando LEDs e iniciando monitoramento...\n");
    npClear();
    npWrite();

    // Preparação do ADC.
    printf("Preparando ADC...\n");

    adc_gpio_init(MIC_PIN);
    adc_init();
    adc_select_input(MIC_CHANNEL);
    adc_fifo_setup(true, true, 1, false, false);
    adc_set_clkdiv(ADC_CLOCK_DIV);

    printf("ADC Configurado!\n\n");

    printf("Preparando DMA...");


    dma_channel = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16); 
    channel_config_set_read_increment(&dma_cfg, false); 
    channel_config_set_write_increment(&dma_cfg, true); 
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);


    sample_mic();

    // Inicializa Buzzer
    pwm_init_buzzer(BUZZER_PIN);
    
    // -------------------- Loop Principal --------------------
    printf("iniciando loop\n");
    while (true) {
        sample_mic();
        float intensity = mic_power();
        uint8_t level = get_intensity(intensity);
        printf("[LOG] Intensidade: %d | Voltagem: %.4fV\n", level, intensity);

        npClear();
        for (int i = 0; i < LED_COUNT; i++) {
            if (level == 2) npSetLED(i, 255, 0, 0);
            else if (level == 1) npSetLED(i, 255, 255, 0);
            else npSetLED(i, 0, 0, 255);
        }
        npWrite();

        if (level == 2) {
            display_message("Ruido\nExcessivo");
            beep(BUZZER_PIN, 500);
            sleep_ms(2000);
            display_message("Detector \nde Ruido");
        }
        sleep_ms(500);
    }
}
