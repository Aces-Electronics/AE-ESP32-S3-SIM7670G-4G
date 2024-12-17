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
#define SerialAT                            Serial1

#define NEOPIXEL                            (38)

#ifndef TINY_GSM_MODEM_SIM7672
#define TINY_GSM_MODEM_SIM7672
#endif

#define MODEM_GPS_ENABLE_GPIO               (-1) //4

// https://www.keyestudio.com/products/keyestudio-mma8452q-module-triaxial-digital-acceleration-tilt-sensor-for-arduino
#define ACCEL_INT1                          (11)
#define ACCEL_INT2                          (12)
#define ACCEL_SCL                           (13)
#define ACCEL_SDA                           (14)  