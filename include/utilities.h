#define MODEM_BAUDRATE                      (115200)
#define MODEM_DTR_PIN                       (45) //9
#define MODEM_TX_PIN                        (18) //11
#define MODEM_RX_PIN                        (17) //10
// The modem boot pin needs to follow the startup sequence.
#define BOARD_PWRKEY_PIN                    (33) // maybe 33? //18 
#define BOARD_LED_PIN                       (-1) // 12
// There is no modem power control, the LED Pin is used as a power indicator here.
#define BOARD_POWERON_PIN                   (BOARD_LED_PIN)
#define MODEM_RING_PIN                      (40) // 3
#define MODEM_RESET_PIN                     (-1)
#define MODEM_RESET_LEVEL                   LOW
#define SerialAT                            Serial1

#define NEOPIXEL                            (38)

#define BOARD_BAT_ADC_PIN                   (4)
#define BOARD_SOLAR_ADC_PIN                 (5)
#define BOARD_MISO_PIN                      (47)
#define BOARD_MOSI_PIN                      (14)
#define BOARD_SCK_PIN                       (21)
#define BOARD_SD_CS_PIN                     (13)

#ifndef TINY_GSM_MODEM_SIM7672
#define TINY_GSM_MODEM_SIM7672
#endif

#define MODEM_GPS_ENABLE_GPIO               (-1) //4