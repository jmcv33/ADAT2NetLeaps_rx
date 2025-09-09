//https://chummersone.github.io/qformat.html#arithmetic
//https://electronics.stackexchange.com/questions/440908/interpretation-of-i2s-data

// GPIO1 : 24.476 MHz
// Sorties I2S:
// GPIO2 : MCLK
// GPIO4 : BCLK
// GPIO5 : LRCLK
// GPIO6 : DATA

// GPIO6 : BP1
// GPIO7 : BP2

// Sur platine 1:
// LED D1:GPIO20/GPIO21:Rouge:1 sur GPIO20
// LED D2:GPIO18/GPIO19:Rouge:1 sur GPIO18
// LED D3:GPIO16/GPIO17:Rouge:1 sur GPIO17

// Sur platine 2:
// LED D1:GPIO20/GPIO21:Rouge:1 sur GPIO21
// LED D2:GPIO18/GPIO19:Rouge:1 sur GPIO19
// LED D3:GPIO16/GPIO17:Rouge:1 sur GPIO16

//openocd -f scripts/interface/cmsis-dap.cfg -c "adapter speed 5000" -f scripts/target/rp2040.cfg -c "program D:/sono/Broadionet/RP2040/RX_ETH/build/rx_eth.elf verify reset"
// Tire 260 mA sous 12 V, charge asymetrique sur ampli HIFI.

/*
Donnees de configuration en flash data:
Rang de l'octet     signification
0                   0xAA, si absent, on fixe les valeurs pas defaut
1                   0x55, si absent, on fixe les valeurs pas defaut
2                   Canal en cours sur sortie analogique L, 0 par defaut
3                   Canal en cours sur sortie analogique R, 3 par defaut
4                   Si 0x00, pas de somme des canaux sur sortie analogique L, par defaut
                    Si 0xFF, somme des canaux sur sortie analigique L en cours
5                   Si 0x00, pas de mute sur la sortie analogique L
                    Si 0xFF, mute en cours sur la sortie analogique L, par defaut
6                   Si 0x00, pas de mute sur la sortie analogique R
                    Si 0xFF, mute en cours sur la sortie analogique R, par defaut
*/
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "rx_eth_pio0_sm0.pio.h"
#include "rx_eth_pio0_sm1.pio.h"
//#include "rx_eth_pio0_sm2.pio.h"
//#include "rx_eth_pio1_sm0.pio.h"
#include "rx_eth_pio1_sm1.pio.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "pico/multicore.h"

#include "Routines_W5500.h"
//#include "pio_spi.h"

#include "nec_receive.h"
// import the assembled PIO state machine program
#include "hardware/clocks.h"
#include "nec_receive.pio.h"

// MAC module 1
//uint8_t adresse_MAC[6] = {0x00, 0x00, 0x00, 0x12, 0x09, 0x92};
// MAC module 2
uint8_t adresse_MAC[6] = {0x00, 0x00, 0x00, 0x01, 0x07, 0x15};
#define PORT_TCP 0x1012 // Ecoute TCP en 4114 

/* LEDs Module 1 */
/*
#define D1_ROUGE gpio_put_masked(0x00300000, 0x00200000)
#define D1_VERTE gpio_put_masked(0x00300000, 0x00100000)
#define D2_VERTE gpio_put_masked(0x00000030, 0x00000020)
#define D2_ROUGE gpio_put_masked(0x00000030, 0x00000010)
#define D3_ROUGE gpio_put_masked(0x00000003, 0x00000001)
#define D3_VERTE gpio_put_masked(0x00000003, 0x00000002)
#define D1_OFF gpio_put_masked(0x00300000, 0x00000000)
#define D2_OFF gpio_put_masked(0x00000030, 0x00000000)
#define D3_OFF gpio_put_masked(0x00000003, 0x00000000)
*/
/* LEDs Module 2 */
#define D1_ROUGE gpio_put_masked(0x00300000, 0x00200000)
#define D1_VERTE gpio_put_masked(0x00300000, 0x00100000)
#define D2_ROUGE gpio_put_masked(0x00000030, 0x00000020)
#define D2_VERTE gpio_put_masked(0x00000030, 0x00000010)
#define D3_VERTE gpio_put_masked(0x00000003, 0x00000001)
#define D3_ROUGE gpio_put_masked(0x00000003, 0x00000002)
#define D1_OFF gpio_put_masked(0x00300000, 0x00000000)
#define D2_OFF gpio_put_masked(0x00000030, 0x00000000)
#define D3_OFF gpio_put_masked(0x00000003, 0x00000000)

const uint LED_PIN = PICO_DEFAULT_LED_PIN;    
uint8_t tampon_tx[640], tampon_rx[640];
int i, i_core1, j, i_tmp, j_tmp;
uint8_t uint8_tmp, uint8_tmp_core1;
uint8_t buffer_4_i2s[384]; // 24 echantillons, 1 pour voie L, 1 pour voie R, le tout x 2
int dma_chan_4_i2s_1, dma_chan_4_i2s_2;
int32_t echant1_int32, echant2_int32, somme_echant_int32;

uint sm_pio1_sm0;

bool flag_it, flag_it_prec;

uint16_t PTR_S0_RX_READ, PTR_S0_TX_WR;
bool reception_ok;
uint8_t compteur_48_octets;

#define FLASH_TARGET_OFFSET (256 * 1024)
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
uint8_t  flash_data[FLASH_PAGE_SIZE];
uint32_t ints; // Sauvegarde des ITs
bool flg_maj_flash_data; 

uint8_t niveau_L_core1, niveau_R_core1;
bool stop_sortie_car_clip_L, stop_sortie_car_clip_R;
uint64_t t_debut_clip_L, t_debut_clip_R, t_debut_bascule_leds;
bool clip_L, clip_R;
bool bascule_leds;

uint64_t t_debut_data_invalides;
bool data_non_valides; // Au cas ou des donnees "bidons" sont recues et
                       // causent des artefacts audio. Cela peut etre
                       // cause par une mauvaise synchro ADAT sur l'emetteur.

int rx_sm;

// canal_ADAT_pour_voie_L et canal_ADAT_pour_voie_R contiennent les offsets
// dans le tampon de reception W5500,
// soit 0 pour le canal ADAT 1, 3 pour le canal ADAT 2, 6 pour le canal ADAT 3, ...
uint8_t canal_ADAT_pour_voie_L, canal_ADAT_pour_voie_R;

uint8_t touche1_tcde, touche2_tcde;
bool somme_des_canaux;
uint64_t t_debut_touche_1, t_debut_led_telec;
bool mute_l, mute_r;
uint64_t t_debut_wd_lien_eth;
bool liaison_eth_ok;

bool flg_dbg;

void maj_flash_data(void);
void core1_entry(void);

void maj_flash_data()
    {
        flash_data[0] = 0xAA;
        flash_data[1] = 0x55;
        flash_data[2] = canal_ADAT_pour_voie_L;
        flash_data[3] = canal_ADAT_pour_voie_R;
        flash_data[4] = somme_des_canaux ? 0xFF : 0x00;
        flash_data[5] = mute_l ? 0xFF : 0x00;
        flash_data[6] = mute_r ? 0xFF : 0x00;        
    }

void core1_entry()
{
    //int rx_sm = nec_rx_init(pio1, 13);
    uint8_t rx_address, rx_data;

    touche1_tcde = TOUCHE_AUCUNE;
    t_debut_bascule_leds = 0;
    
    while (1)
      {
        if ((time_us_64() - t_debut_wd_lien_eth) > 1000)
            {
                // Perte de liaison ETH
                for(i_core1=0; i_core1<256; i_core1++) buffer_4_i2s[i_core1] = 0x00;
                data_non_valides = true;
                t_debut_data_invalides = time_us_64();
                if ((time_us_64() - t_debut_led_telec) > 1.5E5)
                    {
                        liaison_eth_ok = false;
                        D1_ROUGE;
                        D2_OFF;
                        D3_OFF;                        
                    }
            }
        else
            {
                // Si pas d'appui recent sur la telecommande
                if ((time_us_64() - t_debut_led_telec) > 1.5E5)
                    {
                        D1_VERTE;
                        liaison_eth_ok = true;
                    }
                if ((time_us_64() - t_debut_data_invalides) > 5.0E6)
                    {
                        data_non_valides = false;
                    }
            }

        if ((time_us_64() - t_debut_touche_1) > 1.5E6) touche1_tcde = TOUCHE_AUCUNE;
        if (!pio_sm_is_rx_fifo_empty(pio1, rx_sm))
            { 
                uint32_t rx_frame = pio_sm_get(pio1, rx_sm);

                if (nec_decode_frame(rx_frame, &rx_address, &rx_data))
                    {            
                        D1_OFF;
                        t_debut_led_telec = time_us_64();
                        if (    (rx_data == TOUCHE_CH_MOINS)
                            ||  (rx_data == TOUCHE_CH)
                            ||  (rx_data == TOUCHE_CH_PLUS))
                                {   
                                    touche1_tcde = rx_data;
                                    t_debut_touche_1 = time_us_64();
                                    touche2_tcde = TOUCHE_AUCUNE;
                                }
                        else
                                {
                                    touche2_tcde = rx_data;
                                }

                        if (touche1_tcde == TOUCHE_CH_MOINS)
                            {
                                switch(touche2_tcde)
                                    {
                                        case TOUCHE_1 : canal_ADAT_pour_voie_L = 0;
                                                        somme_des_canaux = false;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_2 : canal_ADAT_pour_voie_L = 3;
                                                        somme_des_canaux = false;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_3 : canal_ADAT_pour_voie_L = 6;
                                                        somme_des_canaux = false;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_4 : canal_ADAT_pour_voie_L = 9;
                                                        somme_des_canaux = false;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_5 : canal_ADAT_pour_voie_L = 12;
                                                        somme_des_canaux = false;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_6 : canal_ADAT_pour_voie_L = 15;
                                                        somme_des_canaux = false;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_7 : canal_ADAT_pour_voie_L = 18;
                                                        somme_des_canaux = false;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_8 : canal_ADAT_pour_voie_L = 21;
                                                        somme_des_canaux = false;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_9 : somme_des_canaux = true;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_MOINS : mute_l = true;
                                                            flg_maj_flash_data = true;
                                                            break;
                                        case TOUCHE_PLUS :  mute_l = false;
                                                            flg_maj_flash_data = true;
                                                            break;
                                    }
                            }
                            
                        if (touche1_tcde == TOUCHE_CH_PLUS)
                            {
                                switch(touche2_tcde)
                                    {
                                        case TOUCHE_1 : canal_ADAT_pour_voie_R = 0;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_2 : canal_ADAT_pour_voie_R = 3;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_3 : canal_ADAT_pour_voie_R = 6;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_4 : canal_ADAT_pour_voie_R = 9;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_5 : canal_ADAT_pour_voie_R = 12;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_6 : canal_ADAT_pour_voie_R = 15;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_7 : canal_ADAT_pour_voie_R = 18;
                                                        flg_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_8 : canal_ADAT_pour_voie_R = 21;
                                                        flg_maj_flash_data = true;
                                                        break;                                    
                                        case TOUCHE_MOINS : mute_r = true;
                                                            flg_maj_flash_data = true;
                                                            break;
                                        case TOUCHE_PLUS :  mute_r = false;
                                                            flg_maj_flash_data = true;
                                                            break;
                                    }
                            }
                    
                    if (touche1_tcde == TOUCHE_CH)
                            {
                                switch(touche2_tcde)
                                    {
                                                                           
                                        case TOUCHE_MOINS : mute_l = true;
                                                            mute_r = true;
                                                            flg_maj_flash_data = true;
                                                            break;
                                        case TOUCHE_PLUS :  mute_l = false;
                                                            mute_r = false;
                                                            flg_maj_flash_data = true;
                                                            break;
                                    }
                            }
                    //if (flg_maj_flash_data) pio_sm_set_enabled(pio1, rx_sm, false);                                                     
                    }
            }

            if (!liaison_eth_ok) continue;

// Traitement des leds CLIPs
// Clignotement vert/blanc si le canal est en mute;
// Clignotement rouge/vert de la led du canal R si somme des canaux sur la voie L.               
            if ((time_us_64() - t_debut_bascule_leds) > 3E5)
                {
                    t_debut_bascule_leds = time_us_64();
                    if (bascule_leds)
                            {
                            bascule_leds = false;
                            if (mute_l)
                                {
                                    D2_VERTE;
                                }
                            if (somme_des_canaux)
                                {
                                    D3_ROUGE;
                                }                        
                            if ((!somme_des_canaux) && mute_r)
                                {
                                    D3_VERTE;
                                }                           
                            }
                    else
                        {
                            bascule_leds = true;
                            if (mute_l)
                                {
                                    D2_OFF;
                                }
                            if (somme_des_canaux && (!mute_r))
                                {
                                    D3_VERTE;
                                }
                            if (mute_r)
                                {
                                    D3_OFF;
                                }                            
                        }                        
                }                        
               
            if (!somme_des_canaux)
                {
                    niveau_L_core1 = echant1_int32 >> 16;
                    niveau_R_core1 = echant2_int32 >> 16;
                }
            else
                {
                    niveau_L_core1 = somme_echant_int32 >> 16;
                    niveau_R_core1 = 0x00;                    
                }
            if (!mute_l)
                {
                    if ((time_us_64() - t_debut_clip_L) > 1E6)
                        {
                            stop_sortie_car_clip_L = false;
                        }
                    if ((time_us_64() - t_debut_clip_L) > 1E5)
                        {
                            clip_L = false;
                        }                                  
                    if ((niveau_L_core1 & 0x80))
                                {   
                                    if ((niveau_L_core1 & 0x7F) < 15)
                                        {   
                                            D2_ROUGE;
                                            stop_sortie_car_clip_L = true;
                                            t_debut_clip_L = time_us_64();
                                            clip_L = true;
                                        }   
                                    else
                                        {
                                            if (!clip_L)
                                                {
                                                    if ((niveau_L_core1 & 0x7F) < 127)
                                                        {
                                                            D2_VERTE;
                                                        }
                                                    else
                                                        {
                                                            D2_OFF;
                                                        }
                                                }                                                                        
                                        }                             
                                }
                                else
                                    { 
                                        if (niveau_L_core1 > 112)
                                            {   
                                                D2_ROUGE;
                                                stop_sortie_car_clip_L = true;
                                                t_debut_clip_L = time_us_64();
                                                clip_L = true;                    
                                            }   
                                        else
                                            {   
                                                if (!clip_L)
                                                    {
                                                        if ((niveau_L_core1 & 0x7F) > 1)
                                                            {
                                                                D2_VERTE;
                                                            }
                                                        else
                                                            {
                                                                D2_OFF;
                                                            }
                                                    }                                        
                                            }                               
                                    }
                }

            if ((!mute_r) && (!somme_des_canaux))
                {
                    if ((time_us_64() - t_debut_clip_R) > 1E6)
                        {
                            stop_sortie_car_clip_R = false;
                        }            
                    if ((time_us_64() - t_debut_clip_R) > 1E5)
                        {
                            clip_R = false;
                        }
                    if ((niveau_R_core1 & 0x80))
                                {   
                                    if ((niveau_R_core1 & 0x7F) < 15)
                                        {   
                                            D3_ROUGE;
                                            stop_sortie_car_clip_R = true;
                                            t_debut_clip_R = time_us_64();
                                            clip_R = true;
                                        }   
                                    else
                                        {
                                            if (!clip_R)
                                                {
                                                    if ((niveau_R_core1 & 0x7F) < 127)
                                                        {
                                                            D3_VERTE;
                                                        }
                                                    else
                                                        {
                                                            D3_OFF;
                                                        }
                                                }                                                                        
                                        }                             
                                }
                                else
                                    { 
                                        if (niveau_R_core1 > 112)
                                            {   
                                                D3_ROUGE;
                                                stop_sortie_car_clip_R = true;
                                                t_debut_clip_R = time_us_64();
                                                clip_R = true;                    
                                            }   
                                        else
                                            {
                                                if (!clip_R)
                                                    {
                                                        if (niveau_R_core1 > 1)
                                                            {
                                                                D3_VERTE;
                                                            }
                                                        else
                                                            {
                                                                D3_OFF;
                                                            }
                                                    }                                        
                                            }                               
                                    }
                }      
        }
}

// Version pour 8 echantillons vers DAC. 2 (stereo) x 4 octets x 8 = 64 octets
// envoyes par un canal DMA.
void dma_handler() {
    if (!flag_it)
        {
            flag_it = true;
            dma_hw->ints0 = 1u << dma_chan_4_i2s_1;
            dma_channel_set_read_addr(dma_chan_4_i2s_1, &buffer_4_i2s[0], false);
        }
    else
        {
            flag_it = false;
            dma_hw->ints0 = 1u << dma_chan_4_i2s_2;
            dma_channel_set_read_addr(dma_chan_4_i2s_2, &buffer_4_i2s[128], false);
        }              
}


int main() {
    
    flg_dbg = false;

    stdio_init_all();

    //sleep_ms(20);
    sleep_ms(500);

    // LED D1 Etats sys
    gpio_init(20);
    gpio_set_dir(20, GPIO_OUT);
    gpio_init(21);
    gpio_set_dir(21, GPIO_OUT);
    D1_ROUGE;

    // LED D2 LEFT
    gpio_init(4);
    gpio_set_dir(4, GPIO_OUT);
    gpio_init(5);
    gpio_set_dir(5, GPIO_OUT);
    D2_ROUGE;

    // LED D3 RIGHT
    gpio_init(0);
    gpio_set_dir(0, GPIO_OUT);
    gpio_init(1);
    gpio_set_dir(1, GPIO_OUT);
    D3_ROUGE;

    gpio_init(14); // Activation de l'oscillateur a 24.576 MHz
    gpio_set_dir(14, GPIO_OUT);
    gpio_put(14, 1);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    gpio_init(15); // Pour l'oscillateur a 24.576 MHz
    gpio_set_dir(15, GPIO_IN);

    gpio_init(6); // Pour BP1
    gpio_set_dir(6, GPIO_IN);

    gpio_init(7); // Pour BP2
    gpio_set_dir(7, GPIO_IN);

    //somme_des_canaux = false;
    flg_maj_flash_data = false;
    //mute_l = true;
    //mute_r = true;


//core1_entry();

    //sleep_ms(1000); // Laisser le temps au TDA7418 de s'initialiser.

    for(i=0; i<0x07; i++) flash_data[i] = flash_target_contents[i];
    if ((flash_data[0] != 0xAA) && (flash_data[1] != 0x55)) // Pas de configuration memorisee ?
    //if (1)
        {   // Si non, on memorise une configuration. 
            flash_data[0] = 0xAA;
            flash_data[1] = 0x55;
            flash_data[2] = 0;
            flash_data[3] = 3;
            flash_data[4] = 0x00;
            flash_data[5] = 0xFF;
            flash_data[6] = 0xFF;            
            ints = save_and_disable_interrupts();
            flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
            flash_range_program(FLASH_TARGET_OFFSET, flash_data, FLASH_PAGE_SIZE);
            restore_interrupts (ints);
            canal_ADAT_pour_voie_L = 0;
            canal_ADAT_pour_voie_R = 3;
            somme_des_canaux = false;
            mute_l = true;
            mute_r = true;
        }
    else
        {   
            canal_ADAT_pour_voie_L = flash_data[2];
            canal_ADAT_pour_voie_R = flash_data[3];
            somme_des_canaux = flash_data[4] == 0x00 ? false : true;
            mute_l = flash_data[5] == 0x00 ? false : true;
            mute_r = flash_data[6] == 0x00 ? false : true;
        }

    // PIO 0, machine 0 : Generation de MCLK et BCLK
    uint offset_pio0_sm0 = pio_add_program(pio0, &rx_eth_pio0_sm0_program);
    uint sm_pio0_sm0 = pio_claim_unused_sm(pio0, true);
    pio0_sm0_program_init(pio0, sm_pio0_sm0, offset_pio0_sm0, 8);

    // PIO 0, machine 1 : Generation de LRCLK
    uint offset_pio0_sm1 = pio_add_program(pio0, &rx_eth_pio0_sm1_program);
    uint sm_pio0_sm1 = pio_claim_unused_sm(pio0, true);
    rx_eth_pio0_sm1_program_init(pio0, sm_pio0_sm1, offset_pio0_sm1, 11);

    // PIO 0, machine 2 : Generation de la sortie de donnees I2S    
    uint offset_pio0_sm2 = pio_add_program(pio0, &rx_eth_pio1_sm1_program);
    uint sm_pio0_sm2 = pio_claim_unused_sm(pio0, true);
    rx_eth_pio1_sm1_program_init(pio0, sm_pio0_sm2, offset_pio0_sm2, 12);

    // Installation de deux canaux DMA chaines qui alimentent la sortie I2S
    // via la machine 2 du PIO0.
    flag_it = false;
    dma_chan_4_i2s_1 = dma_claim_unused_channel(true);
    dma_chan_4_i2s_2 = dma_claim_unused_channel(true);    
    // DMA 1, envoi de la parie basse du buffer.
    dma_channel_config c1 = dma_channel_get_default_config(dma_chan_4_i2s_1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_chain_to(&c1, dma_chan_4_i2s_2);
    channel_config_set_dreq(&c1, DREQ_PIO0_TX2);
    dma_channel_set_irq0_enabled(dma_chan_4_i2s_1, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_configure(
        dma_chan_4_i2s_1,
        &c1,
        &pio0_hw->txf[2], // Write address (only need to set this once)
        &buffer_4_i2s[0], // A lire a la base du buffer,
        32,            // 16 mots de 4 octets.
        false             // Don't start yet
    );

    // DMA 2, envoi de la parie haute du buffer.
    dma_channel_config c2 = dma_channel_get_default_config(dma_chan_4_i2s_2);
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);
    channel_config_set_read_increment(&c2, true);
    channel_config_set_write_increment(&c2, false);
    channel_config_set_chain_to(&c2, dma_chan_4_i2s_1);
    channel_config_set_dreq(&c2, DREQ_PIO0_TX2);
    dma_channel_set_irq0_enabled(dma_chan_4_i2s_2, true);

    dma_channel_configure(
        dma_chan_4_i2s_2,
        &c2,
        &pio0_hw->txf[2], // Write address (only need to set this once)
        //&buffer_4_i2s[768],
        &buffer_4_i2s[128], // A lire dans la seconde partie du buffer,
        32,            // 16 mots de 4 octets.
        false             // Don't start yet
    );
   
    //channel_config_set_chain_to(&c1, dma_chan_4_i2s_2);
    //channel_config_set_chain_to(&c2, dma_chan_4_i2s_1);

    for(i=0; i<8; i++) // Pour que la machine ne soit pas bloquee
        {              // le temps du demarrage du DMA.
            pio_sm_put_blocking(pio0, sm_pio0_sm2, 0x00000000);
        }
    //pio_sm_set_enabled(pio0, sm_pio0_sm2, true);    
    pio_enable_sm_mask_in_sync(pio0, (1ULL << sm_pio0_sm0) | (1ULL << sm_pio0_sm1)  | (1ULL << sm_pio0_sm2));
   dma_start_channel_mask(1u << dma_chan_4_i2s_1);

    spi_init(spi1, 50000000);
    gpio_set_function(28, GPIO_FUNC_SPI);
    gpio_set_function(26, GPIO_FUNC_SPI);
    gpio_set_function(27, GPIO_FUNC_SPI);

    spi_init(spi0, 50000000); // W5500 de recopie.
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);
    // Pour Picotool
    //bi_decl(bi_4pins_with_func(16, 19, 18, 22, GPIO_FUNC_SPI));

    gpio_init(2); // /INT du W5500
    gpio_set_dir(2, GPIO_IN);

    gpio_init(22); // /CS du W5500
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);    

    /*
    gpio_init(21); // /RESET du W5500
    gpio_set_dir(21, GPIO_OUT);
    gpio_put(21, 1);
    sleep_ms(100);
    gpio_put(21, 0);
    sleep_ms(10);
    gpio_put(21, 1);
    sleep_ms(100);
    */

    gpio_init(17); // /CS du W5500 de recopie
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, 1);

    gpio_init(3); // /RESET des W5500
    gpio_set_dir(3, GPIO_OUT);
    gpio_put(3, 1);
    sleep_ms(100);
    gpio_put(3, 0);
    sleep_ms(10);
    gpio_put(3, 1);
    sleep_ms(100);

    //sleep_ms(2000);

    // Configuration generale du W5500
    tampon_tx[0] = 0x00;
    tampon_tx[1] = 0x00;
    tampon_tx[2] = 0x04;
    tampon_tx[3] = 0x00;
    tampon_tx[4] = flash_data[1];
    tampon_tx[5] = flash_data[2];
    tampon_tx[6] = flash_data[3];
    tampon_tx[7] = flash_data[4];
    tampon_tx[8] = flash_data[5];
    tampon_tx[9] = flash_data[6];
    tampon_tx[10] = flash_data[7];
    tampon_tx[11] = flash_data[8];
    tampon_tx[12] = adresse_MAC[0];
    tampon_tx[13] = adresse_MAC[1];
    tampon_tx[14] = adresse_MAC[2];
    tampon_tx[15] = adresse_MAC[3];
    tampon_tx[16] = adresse_MAC[4];
    tampon_tx[17] = adresse_MAC[5];
    tampon_tx[18] = flash_data[9];
    tampon_tx[19] = flash_data[10];
    tampon_tx[20] = flash_data[11];
    tampon_tx[21] = flash_data[12];   
    gpio_put(22, 0);
    spi_write_read_blocking(spi1, tampon_tx, tampon_rx, 22);       
    gpio_put(22, 1);

    // L'ordre d'ouverture des sockets est important pour laisser une
    // taille maximale (16 Ko) au buffer de reception du W5500 de la
    // socket 0. La taille de 1 Ko du buffer de reception de la socket 2
    // est ramenee a 0 lors de l'affectation d'une taille de 16 Ko a la
    // socket 0. Neanmoins, la socket 2 genere quand meme une IT sur reception.

    // Configuration de la socket 1 (TCP de controle)
    /*
    W5500_ecrt_Sn_MR(1, 0x01); // TCP
    W5500_ecrt_Sn_SRC_PORT(1, PORT_TCP); // Port 4114
    tampon_tx[3] = 0x02; // Validation IT sur socket 1
    W5500_SPI_ecrt_Frame(0x0018, 0x00, 1);
    W5500_ecrt_Sn_IMR(1, 0x04); // IT sur reception TCP.
    W5500_ecrt_Sn_RXBUF_SIZE(1, 1);
    W5500_ecrt_Sn_TXBUF_SIZE(1, 1);
    W5500_ecrt_Sn_CR(1, 0x01);// Ouverture de la socket 1
    W5500_ecrt_Sn_CR(1, 0x02);// Listen sur la socket 1

    // Configuration de la socket 2 (UDP multicast de controle)
    W5500_ecrt_Sn_MR(2, 0x82); // Ecriture dans S2_MR : UDP multicast
    //W5500_ecrt_Sn_SRC_PORT(2, PORT_TCP);
    W5500_ecrt_Sn_SRC_PORT(2, 4114); // Port 4114
    tampon_tx[3] = 0x06; // Validation IT sur les sockets 1 et 2
    W5500_SPI_ecrt_Frame(0x0018, 0x00, 1);
    W5500_ecrt_Sn_IMR(2, 0x04); // Validation IT sur reception sur la socket 2
    W5500_ecrt_Sn_DEST_IP(2, flash_data[14], flash_data[15], flash_data[16], flash_data[17]);
    //W5500_ecrt_Sn_DEST_PORT(2, PORT_TCP);// PORT de destination dans S2_DPORT
    W5500_ecrt_Sn_DEST_PORT(2, 4114);// PORT de destination dans S2_DPORT
    W5500_ecrt_Sn_RXBUF_SIZE(2, 1);
    W5500_ecrt_Sn_TXBUF_SIZE(2, 1);
    W5500_ecrt_Sn_CR(2, 0x01);// Ouverture de la socket 2
    */

     // Configuration de la socket 0 - UDP pour reception du stream audio
    //W5500_ecrt_Sn_MR(0, flash_data[13]);// UDP, multicast, IGMP V2
    W5500_ecrt_Sn_MR(spi1, 22, 0, 0xF4);// MAC RAW avec filtrage
    //W5500_ecrt_Sn_SRC_PORT(0, 0x1DE6);// PORT source (7654) sur la socket 0
    // IP de destination dans S0_DIPR
    //W5500_ecrt_Sn_DEST_IP(0, flash_data[14], flash_data[15], flash_data[16], flash_data[17]);
    //W5500_ecrt_Sn_DEST_PORT(0, 0x1DE6);// PORT de destination dans S0_DPORT : 7654
    W5500_ecrt_Sn_RXBUF_SIZE(spi1, 22, 0, 16);
    W5500_ecrt_Sn_TXBUF_SIZE(spi1, 22, 0, 1);
    tampon_tx[3] = 0x01; // Validation IT sur la socket 0
    W5500_SPI_ecrt_Frame(spi1, 22, 0x0018, 0x00, 1);
    W5500_ecrt_Sn_IMR(spi1, 22, 0, 0x04); // Validation IT sur reception sur la socket 0
    W5500_ecrt_Sn_CR(spi1, 22, 0, 0x01);// Ouverture de la socket 0

    // Configuration du W5500 de recopie
    W5500_ecrt_Sn_MR(spi0, 17, 0, 0xF4);// MAC RAW avec filtrage
    W5500_ecrt_Sn_RXBUF_SIZE(spi0, 17, 0, 1);
    W5500_ecrt_Sn_TXBUF_SIZE(spi0, 17, 0, 16);
    W5500_ecrt_Sn_CR(spi0, 17, 0, 0x01);// Ouverture de la socket 0

    PTR_S0_RX_READ = 0x0000;
    PTR_S0_TX_WR = 0x0000;
    compteur_48_octets = 0;
    flag_it_prec = flag_it;
    memset(buffer_4_i2s, 0x00, sizeof (buffer_4_i2s));
    reception_ok = false;

    t_debut_clip_L = 0;
    t_debut_clip_L = 0;
    rx_sm = nec_rx_init(pio1, 13);
    multicore_launch_core1(core1_entry);
    //sleep_ms(2000);

    t_debut_wd_lien_eth = 0;
    liaison_eth_ok = false;

    data_non_valides = true;
    t_debut_data_invalides = time_us_64();

    // Boucle principale
    for(;;)
        {   
            if (!gpio_get(2))
                {
                    t_debut_wd_lien_eth = time_us_64();                    
                    // On passe ici toutes les 500 µs
                    //gpio_put(LED_PIN, gpio_get(LED_PIN)==1?0:1);  
                    W5500_ecrt_Sn_IR(spi1, 22, 0, 0x04); // Remonte de /INT
                    // Point A
                    // 200 µs du point A au point B,
                    // Pour une période de 500 µs.   
//gpio_put(LED_PIN, 1);                 
                    tampon_tx[0] = PTR_S0_RX_READ>>8; // Lecture des donnees recues sur la socket 0
                    tampon_tx[1] = PTR_S0_RX_READ;
                    tampon_tx[2] = 0x18;                    
                    gpio_put(22, 0);
                    // Indice 3 : pF du nombre d'octets recus
                    // Indice 4 : pf du nombre d'octets recus
                    // Indice 17 : Debut des datas.
                    //spi_write_read_blocking(spi1, tampon_tx, tampon_rx, 593);
                    // 209 = 192 de data audio + nombre d'octets recus + 12 pour
                    // les @ MACs + 3 pour le protocole W5500.
                    spi_write_read_blocking(spi1, tampon_tx, tampon_rx, 401); // 401 pour 16 trames ADAT, 209 pour 8 trames ADAT
                    gpio_put(22, 1);
                    // Point B
                    // 4.5 µs du point B au point C,
                    // Pour une période de 500 µs.
                    //PTR_S0_RX_READ+=590;
                    // 206 octets recus:192 octets Data audio + 12 octets @MACs + numbre d'octets recus.                    
                    PTR_S0_RX_READ+=398; // 398 pour 16 trames ADAT, 206 pour 8 trames ADAT
                    tampon_tx[0] = 0x00; // Offset:Mise a jour de S0_RX_RD
                    tampon_tx[1] = 0x28;
                    tampon_tx[2] = 0x0C;
                    tampon_tx[3] = PTR_S0_RX_READ>>8;
                    tampon_tx[4] = PTR_S0_RX_READ;
                    gpio_put(22, 0);
                    spi_write_read_blocking(spi1, tampon_tx, tampon_rx, 5);
                    gpio_put(22, 1);
                    tampon_tx[0] = 0x00; // Offset: Validation de la lecture dans S0_CR
                    tampon_tx[1] = 0x01;
                    tampon_tx[2] = 0x0C;
                    tampon_tx[3] = 0x40;
                    gpio_put(22, 0);
                    spi_write_read_blocking(spi1, tampon_tx, tampon_rx, 4);
                    gpio_put(22, 1);

                    // Reemission vers le W5500 de reopie.               
                    tampon_rx[2] = PTR_S0_TX_WR>>8; // Ecriture sur la FIFO d'emission du W5500.
                    tampon_rx[3] = PTR_S0_TX_WR;
                    tampon_rx[4] = 0x14;
                    gpio_put(17, 0);
                    //spi_write_read_blocking(spi0, &tampon_rx[2], tampon_tx, 591);
                    // 207 = 192 de data audio + 12 pour les @ MACs 
                    // + 3 pour le protocole W5500.
                    // A partir de tampon_rx[2] pour ne pas envoyer le nombre
                    // d'octets recus.
                    spi_write_read_blocking(spi0, &tampon_rx[2], tampon_tx, 399); // 399 pour 16 trames ADAT, // 207 pour 8 trames ADAT
                    gpio_put(17, 1);

                    //PTR_S0_TX_WR+=588;
                    // 204 : 192 de donnees audio + 12 octets pour les @ MACs.
                    // 396 : 384 de donnees audio + 12 octets pour les @ MACs.
                    PTR_S0_TX_WR+=396;

                    tampon_tx[0] = 0x00; // Offset:Mise a jour de PTR_S0_TX_WR
                    tampon_tx[1] = 0x24;
                    tampon_tx[2] = 0x0C;
                    tampon_tx[3] = PTR_S0_TX_WR>>8;
                    tampon_tx[4] = PTR_S0_TX_WR;
                    gpio_put(17, 0);
                    spi_write_read_blocking(spi0, tampon_tx, tampon_rx, 5);
                    gpio_put(17, 1);

                    tampon_tx[0] = 0x00; // Offset: Validation de l'ecriture dans S0_CR
                    tampon_tx[1] = 0x01;
                    tampon_tx[2] = 0x0C;
                    tampon_tx[3] = 0x20;
                    gpio_put(17, 0);
                    spi_write_read_blocking(spi0, tampon_tx, tampon_rx, 4);
                    gpio_put(17, 1);

                    if (data_non_valides) continue;
 
                    while (flag_it_prec == flag_it);
                    flag_it_prec = flag_it;
                    
                    if (!flag_it)
                        {   
                            for(j = 17, i = 128; j<380; j+=24)                            
                                {
                                    echant1_int32 = tampon_rx[j + canal_ADAT_pour_voie_L + 2] << 16;
                                    echant1_int32+= tampon_rx[j + canal_ADAT_pour_voie_L + 1] << 8;
                                    echant1_int32+= tampon_rx[j + canal_ADAT_pour_voie_L];
                                    if (echant1_int32 & 0x800000)
                                        echant1_int32|=0xFF000000;

                                    echant2_int32 = tampon_rx[j + canal_ADAT_pour_voie_R + 2] << 16;
                                    echant2_int32+= tampon_rx[j + canal_ADAT_pour_voie_R + 1] << 8;
                                    echant2_int32+= tampon_rx[j + canal_ADAT_pour_voie_R];
                                    if (echant2_int32 & 0x800000)
                                        echant2_int32|=0xFF000000;

                                    if (somme_des_canaux)
                                        {
                                            if (mute_l) echant1_int32 = 0L;
                                            if (mute_r) echant2_int32 = 0L;
                                            if (stop_sortie_car_clip_L)
                                                {
                                                    echant1_int32 = 0L;
                                                    echant2_int32 = 0L;
                                                }
                                        }
                                    else
                                        {
                                            if (stop_sortie_car_clip_L || mute_l) echant1_int32 = 0L;
                                            if (stop_sortie_car_clip_R || mute_r) echant2_int32 = 0L;
                                        }                                    

                                    if (!somme_des_canaux)
                                        {
                                            buffer_4_i2s[i+3] = echant1_int32 >> 16;
                                            buffer_4_i2s[i+2] = echant1_int32 >> 8;
                                            buffer_4_i2s[i+1] = echant1_int32;
                                            i+=4;
                                            buffer_4_i2s[i+3] = echant2_int32 >> 16;
                                            buffer_4_i2s[i+2] = echant2_int32 >> 8;
                                            buffer_4_i2s[i+1] = echant2_int32;
                                            i+=4;
                                        }
                                    else
                                        {
                                            echant1_int32>>=1;                                    
                                            echant2_int32>>=1;                                           
                                            
                                            somme_echant_int32 = echant1_int32 + echant2_int32;
                                            buffer_4_i2s[i+3] = somme_echant_int32 >> 16;
                                            buffer_4_i2s[i+2] = somme_echant_int32 >> 8;
                                            buffer_4_i2s[i+1] = somme_echant_int32;
                                            i+=4;                               
                                            buffer_4_i2s[i+3] = 0x00;
                                            buffer_4_i2s[i+2] = 0x00;
                                            buffer_4_i2s[i+1] = 0x00;                                                                            
                                            i+=4;
                                        }                                                                       
                                }
                        }                            
                    else
                        {   
                            for(j = 17, i = 0; j<380; j+=24)
                                {
                                    echant1_int32 = tampon_rx[j + canal_ADAT_pour_voie_L + 2] << 16;
                                    echant1_int32+= tampon_rx[j + canal_ADAT_pour_voie_L + 1] << 8;
                                    echant1_int32+= tampon_rx[j + canal_ADAT_pour_voie_L];
                                    if (echant1_int32 & 0x800000)
                                        echant1_int32|=0xFF000000;

                                    echant2_int32 = tampon_rx[j + canal_ADAT_pour_voie_R + 2] << 16;
                                    echant2_int32+= tampon_rx[j + canal_ADAT_pour_voie_R + 1] << 8;
                                    echant2_int32+= tampon_rx[j + canal_ADAT_pour_voie_R];
                                    if (echant2_int32 & 0x800000)
                                        echant2_int32|=0xFF000000;

                                    if (somme_des_canaux)
                                        {
                                            if (mute_l) echant1_int32 = 0L;
                                            if (mute_r) echant2_int32 = 0L;
                                            if (stop_sortie_car_clip_L)
                                                {
                                                    echant1_int32 = 0L;
                                                    echant2_int32 = 0L;
                                                }
                                        }
                                    else
                                        {
                                            if (stop_sortie_car_clip_L || mute_l) echant1_int32 = 0L;
                                            if (stop_sortie_car_clip_R || mute_r) echant2_int32 = 0L;
                                        }

                                    if (!somme_des_canaux)
                                        {
                                            buffer_4_i2s[i+3] = echant1_int32 >> 16;
                                            buffer_4_i2s[i+2] = echant1_int32 >> 8;
                                            buffer_4_i2s[i+1] = echant1_int32;
                                            i+=4;
                                            buffer_4_i2s[i+3] = echant2_int32 >> 16;
                                            buffer_4_i2s[i+2] = echant2_int32 >> 8;
                                            buffer_4_i2s[i+1] = echant2_int32;
                                            i+=4;
                                        }
                                    else
                                        {
                                            echant1_int32>>=1;                                    
                                            echant2_int32>>=1;                                           
                                            
                                            somme_echant_int32 = echant1_int32 + echant2_int32;
                                            buffer_4_i2s[i+3] = somme_echant_int32 >> 16;
                                            buffer_4_i2s[i+2] = somme_echant_int32 >> 8;
                                            buffer_4_i2s[i+1] = somme_echant_int32;
                                            i+=4;                               
                                            buffer_4_i2s[i+3] = 0x00;
                                            buffer_4_i2s[i+2] = 0x00;
                                            buffer_4_i2s[i+1] = 0x00;                                                                            
                                            i+=4;
                                        }                                                                       
                                }                                                            
                        }                                                       
                }
        if (flg_maj_flash_data)
                        {                            
                            maj_flash_data();
                            pio_sm_set_enabled(pio0, sm_pio0_sm0, false);
                            multicore_reset_core1();
                            ints = save_and_disable_interrupts();
                            flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
                            flash_range_program(FLASH_TARGET_OFFSET, flash_data, FLASH_PAGE_SIZE);
                            restore_interrupts (ints);
                            flg_maj_flash_data = false; 
                            multicore_launch_core1(core1_entry);
                            pio_sm_set_enabled(pio0, sm_pio0_sm0, true);                                                                                
                        } 
        }    
    return 0;
}
