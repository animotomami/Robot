

/// ENCODER PINS
#define HSPI 2  // 2 para S2 y S3, 1 para S1.
#define VSPI 3
#define HSPI_MISO 13
#define HSPI_MOSI 11
#define HSPI_SCLK 12
#define HSPI_CS0 14
#define HSPI_CS1 10
#define HSPI_CS2 9

/// Fns leer tres encoders
float encoder0_read ();
float encoder1_read ();
float encoder2_read ();

/// Fns set a 0 tres encoders
void set_zero_enc_0 ();
void set_zero_enc_1 ();
void set_zero_enc_2 ();

/// Fn inicialización de SPI
void encoder_init();