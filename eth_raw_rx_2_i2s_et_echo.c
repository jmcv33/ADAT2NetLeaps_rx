// eth_raw_rx_2_i2s_et_echo.c
// Raspberry PICO 2, RP2350
// Auteur : jeanmarc.villers@wanadoo.fr
// Licence Creative Commons CC BY-NC-SA
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "i2s_mclk_bclk_pio0_sm0.pio.h"
#include "i2s_lrclk_pio0_sm1.pio.h"
#include "i2s_data_pio0_sm2.pio.h"
#include "w5500.h"
#include "nec_receive.h"
#include "nec_receive.pio.h"

// MAC source. Tous les recepteurs ont cette MAC en MAC source.
uint8_t adresse_MAC_srce[6] = {0x00, 0x00, 0x00, 0x01, 0x07, 0x15};

// Commande des LEDs selon le boitier. Depend du sens des broches impossibles
// a reperer... Important pour le rouge et le vert.
// D1 : LED d'etat du systeme
// D2 : LED d'etat de la sortie gauche
// D3 : LED d'etat de la sortie droite
// LEDs Module 1
#define D1_ROUGE gpio_put_masked(0x00300000, 0x00200000)
#define D1_VERTE gpio_put_masked(0x00300000, 0x00100000)
#define D2_VERTE gpio_put_masked(0x00000030, 0x00000020)
#define D2_ROUGE gpio_put_masked(0x00000030, 0x00000010)
#define D3_ROUGE gpio_put_masked(0x00000003, 0x00000001)
#define D3_VERTE gpio_put_masked(0x00000003, 0x00000002)
#define D1_OFF gpio_put_masked(0x00300000, 0x00000000)
#define D2_OFF gpio_put_masked(0x00000030, 0x00000000)
#define D3_OFF gpio_put_masked(0x00000003, 0x00000000)

// LEDs Module 2
/*
#define D1_ROUGE gpio_put_masked(0x00300000, 0x00200000)
#define D1_VERTE gpio_put_masked(0x00300000, 0x00100000)
#define D2_ROUGE gpio_put_masked(0x00000030, 0x00000020)
#define D2_VERTE gpio_put_masked(0x00000030, 0x00000010)
#define D3_VERTE gpio_put_masked(0x00000003, 0x00000001)
#define D3_ROUGE gpio_put_masked(0x00000003, 0x00000002)
#define D1_OFF gpio_put_masked(0x00300000, 0x00000000)
#define D2_OFF gpio_put_masked(0x00000030, 0x00000000)
#define D3_OFF gpio_put_masked(0x00000003, 0x00000000)
*/

uint8_t buffer_tx_W5500[512], buffer_rx_W5500[512];
uint32_t i, i_core1, j;
uint8_t buffer_4_i2s[256]; // 32 echantillons par voies
uint32_t dma_chan_4_i2s_1, dma_chan_4_i2s_2;

// Variables intermediaures pour les echantillons audio envoyes sur les sorties gauche et droite.
// somme_echant_int32 utilise pour la sortie gauche en "MONO", mix des deux voies selectionnees.
int32_t echant1_int32, echant2_int32, somme_echant_int32;

bool flag_it, flag_it_prec; // Selection partie basse 
uint16_t PTR_S0_RX_READ, PTR_S0_TX_WR;

// Toute configuration depuis la telecommande est memorisee en FLASH
// afin d'etre recuperee apres une remise sous tension.
// L'ecriture / lecture en flash (Circuit-integre a cote du RP2350 en liaison QSPI)
// est faite selon l'exemple:
// https://github.com/raspberrypi/pico-examples/tree/master/flash/program
// Voir fichier notes.txt 
#define FLASH_TARGET_OFFSET (256 * 1024)
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
uint8_t  flash_data[FLASH_PAGE_SIZE];
uint32_t ints; // Sauvegarde des ITs
bool flag_maj_flash_data; 

int32_t echant1_4_LED_int32, echant2_4_LED_int32, somme_echant_4_LED_int32;
uint8_t niveau_L_core1, niveau_R_core1;
bool flag_stop_sortie_L, flag_stop_sortie_R;
uint64_t t_debut_clip_L, t_debut_clip_R, t_debut_bascule_leds;
bool flag_clip_L, flag_clip_R;
bool flag_bascule_leds;

uint64_t t_debut_data_invalides;
bool flag_data_bruit; // Au cas ou des donnees "bidons" sont recues et
                       // causent des artefacts audio. Cela peut etre
                       // cause par une mauvaise synchro ADAT sur l'emetteur
                       // provoque par exemple par un flux ADAT a 44.1 KHz au lieu de 48 KHz
                       // Affecte a true aussi si pas de reception Ethernet.
uint32_t machine_pio_telec;

// canal_ADAT_pour_voie_L et canal_ADAT_pour_voie_R contiennent les offsets
// dans le tampon du W5500 en reception,
// soit 0 pour le canal ADAT 1, 3 pour le canal ADAT 2, 6 pour le canal ADAT 3, ...
uint8_t canal_ADAT_pour_voie_L, canal_ADAT_pour_voie_R;

uint8_t touche1_tcde, touche2_tcde;
bool flag_somme_canaux;
uint64_t t_debut_touche_1, t_debut_led_telec;
bool flag_mute_L, flag_mute_R;
uint64_t t_debut_wd_lien_eth;
bool flag_liaison_eth_ok;

void core1_entry(void);

// Code pour le core 1 : Supervision des operations
// Traitement:
// - des ordres de la telecommande - Voir notes.txt;
// - des clips;
// - des LEDs.
void core1_entry()
{
    uint8_t rx_address, rx_data;

    touche1_tcde = TOUCHE_AUCUNE;
    t_debut_bascule_leds = 0;
    
    while (1)
      {
        // Si absence de reception Ethernet depuis plus de 1 ms
        if ((time_us_64() - t_debut_wd_lien_eth) > 1000)
            {
                // En vu d'envoyer des 0x00000 sur l'I2S.
                for(i_core1=0; i_core1<256; i_core1++) buffer_4_i2s[i_core1] = 0x00;
                flag_data_bruit = true;
                t_debut_data_invalides = time_us_64();
                // Si pas de flash pour telecommande en cours sur D1
                if ((time_us_64() - t_debut_led_telec) > 1.5E5)
                    {
                        flag_liaison_eth_ok = false;
                        D1_ROUGE;
                        D2_OFF;
                        D3_OFF;                        
                    }
            }
        else // Reception Ethernet OK
            {
                // Si pas de flash dur D1 pour telecommande en cours
                if ((time_us_64() - t_debut_led_telec) > 1.5E5)
                    {
                        D1_VERTE;
                        flag_liaison_eth_ok = true;
                    }
                if ((time_us_64() - t_debut_data_invalides) > 5.0E6)
                    {
                        flag_data_bruit = false;
                    }
            }

        // Si absence de sequence recue de la telecommande depuis 1.5 s
        if ((time_us_64() - t_debut_touche_1) > 1.5E6) touche1_tcde = TOUCHE_AUCUNE;
        // Si reception d'une touche de la telecommande
        if (!pio_sm_is_rx_fifo_empty(pio1, machine_pio_telec))
            { 
                uint32_t rx_frame = pio_sm_get(pio1, machine_pio_telec);

                if (nec_decode_frame(rx_frame, &rx_address, &rx_data))
                    {            
                        D1_OFF; // flash par OFF de D1
                        t_debut_led_telec = time_us_64();
                        // Si premiere touche tapee OK
                        if (    (rx_data == TOUCHE_CH_MOINS)
                            ||  (rx_data == TOUCHE_CH)
                            ||  (rx_data == TOUCHE_CH_PLUS))
                                {   
                                    touche1_tcde = rx_data;
                                    t_debut_touche_1 = time_us_64();
                                    touche2_tcde = TOUCHE_AUCUNE;
                                }
                        else // Si seconde touche tapee (avec permiere touche OK)
                                {
                                    touche2_tcde = rx_data;
                                }

                        if (touche1_tcde == TOUCHE_CH_MOINS)
                            {
                                switch(touche2_tcde)
                                    {
                                        case TOUCHE_1 : canal_ADAT_pour_voie_L = 0;
                                                        flag_somme_canaux = false;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_2 : canal_ADAT_pour_voie_L = 3;
                                                        flag_somme_canaux = false;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_3 : canal_ADAT_pour_voie_L = 6;
                                                        flag_somme_canaux = false;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_4 : canal_ADAT_pour_voie_L = 9;
                                                        flag_somme_canaux = false;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_5 : canal_ADAT_pour_voie_L = 12;
                                                        flag_somme_canaux = false;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_6 : canal_ADAT_pour_voie_L = 15;
                                                        flag_somme_canaux = false;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_7 : canal_ADAT_pour_voie_L = 18;
                                                        flag_somme_canaux = false;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_8 : canal_ADAT_pour_voie_L = 21;
                                                        flag_somme_canaux = false;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_9 : flag_somme_canaux = true;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_MOINS : flag_mute_L = true;
                                                            flag_maj_flash_data = true;
                                                            break;
                                        case TOUCHE_PLUS :  flag_mute_L = false;
                                                            flag_maj_flash_data = true;
                                                            break;
                                    }
                            }
                            
                        if (touche1_tcde == TOUCHE_CH_PLUS)
                            {
                                switch(touche2_tcde)
                                    {
                                        case TOUCHE_1 : canal_ADAT_pour_voie_R = 0;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_2 : canal_ADAT_pour_voie_R = 3;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_3 : canal_ADAT_pour_voie_R = 6;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_4 : canal_ADAT_pour_voie_R = 9;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_5 : canal_ADAT_pour_voie_R = 12;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_6 : canal_ADAT_pour_voie_R = 15;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_7 : canal_ADAT_pour_voie_R = 18;
                                                        flag_maj_flash_data = true;
                                                        break;
                                        case TOUCHE_8 : canal_ADAT_pour_voie_R = 21;
                                                        flag_maj_flash_data = true;
                                                        break;                                    
                                        case TOUCHE_MOINS : flag_mute_R = true;
                                                            flag_maj_flash_data = true;
                                                            break;
                                        case TOUCHE_PLUS :  flag_mute_R = false;
                                                            flag_maj_flash_data = true;
                                                            break;
                                    }
                            }
                    
                    if (touche1_tcde == TOUCHE_CH)
                            {
                                switch(touche2_tcde)
                                    {
                                                                           
                                        case TOUCHE_MOINS : flag_mute_L = true;
                                                            flag_mute_R = true;
                                                            flag_maj_flash_data = true;
                                                            break;
                                        case TOUCHE_PLUS :  flag_mute_L = false;
                                                            flag_mute_R = false;
                                                            flag_maj_flash_data = true;
                                                            break;
                                    }
                            }
                    }
            }

            if (!flag_liaison_eth_ok) continue; // On ne va pas au-dela si absence de reception Ethernet

// Traitement des LEDs d'etat des sortie:
// Clignotement vert/blanc si le canal est en mute;
// Clignotement rouge/vert de la LED du canal R si somme des canaux sur la sortie L;
// Sinon vert selon la modulation et rouge (100 ms) en cas de clip.      
// Les variables bascule_leds et t_debut_bascule_leds sont destinees a permettre le clignotement
// (changement d'etat tous les 300 ms) des LEDs d'etats des sorties en blanc/vert (mute) ou
// en vert/rouge (somme des canaux sur la sortie gauche).
            if ((time_us_64() - t_debut_bascule_leds) > 3E5)
                {
                    t_debut_bascule_leds = time_us_64();
                    if (flag_bascule_leds)
                            {
                            flag_bascule_leds = false;
                            if (flag_mute_L)
                                {
                                    D2_VERTE;
                                }
                            if (flag_somme_canaux)
                                {
                                    D3_ROUGE;
                                }                        
                            if (flag_mute_R && (!flag_somme_canaux))
                                {
                                    D3_VERTE;
                                }                           
                            }
                    else
                        {
                            flag_bascule_leds = true;
                            if (flag_mute_L)
                                {
                                    D2_OFF;
                                }
                            if (flag_somme_canaux)
                                {
                                    D3_VERTE;
                                }
                            if (flag_mute_R  && (!flag_somme_canaux))
                                {
                                    D3_OFF;
                                }                            
                        }                        
                }                        
            // Evaluaion des niveaux audio
            // En cas de clip sur une sortie, la LED correspondante est rouge pendant 100 ms
            // et la sortie est OFF pendant 1 s.
            if (!flag_somme_canaux)
                {
                    // Evaluation du niveau audio sur les 8 bits I2S de poid Fort uniquement.
                    niveau_L_core1 = echant1_4_LED_int32 >> 16;
                    niveau_R_core1 = echant2_4_LED_int32 >> 16;
                }
            else
                {
                    niveau_L_core1 = somme_echant_4_LED_int32 >> 16;
                    niveau_R_core1 = 0x00;                    
                }
            if (!flag_mute_L)
                {
                    // Reactivation de la sortie gauche au bout d'1 s
                    if ((time_us_64() - t_debut_clip_L) > 1E6)
                        {
                            flag_stop_sortie_L = false;
                        }
                    // Extinction du rouge sur la LED de la sortie gauche au bout de 100 ms
                    if ((time_us_64() - t_debut_clip_L) > 1E5)
                        {
                            flag_clip_L = false;
                        }                       
                        
                    // I2S encode en complement a 2.
                    // Cas de la sortie gauche.
                    // Si la valeur est negative
                    if (niveau_L_core1 & 0x80)
                                {   
                                    // Suppression du bit de signe
                                    // et si valeur < 15, alors CLIP
                                    if ((niveau_L_core1 & 0x7F) < 15)
                                        {   
                                            D2_ROUGE;
                                            flag_stop_sortie_L = true;
                                            t_debut_clip_L = time_us_64();
                                            flag_clip_L = true;
                                        }   
                                    else
                                        {
                                            if (!flag_clip_L)
                                                {
                                                    // Si presence modulation raisonnable
                                                    if ((niveau_L_core1 & 0x7F) < 127)
                                                        {
                                                            D2_VERTE;
                                                        }
                                                    else
                                                        {   // Si absence de modulation
                                                            D2_OFF;
                                                        }
                                                }                                                                        
                                        }                             
                                }
                                else // Idem mais dans le cas d'une valeur I2S positive.
                                    { 
                                        if (niveau_L_core1 > 112)
                                            {   
                                                D2_ROUGE;
                                                flag_stop_sortie_L = true;
                                                t_debut_clip_L = time_us_64();
                                                flag_clip_L = true;                    
                                            }   
                                        else
                                            {   
                                                if (!flag_clip_L)
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
            // Cas de la sortie droite
            if ((!flag_mute_R) && (!flag_somme_canaux))
                {
                    if ((time_us_64() - t_debut_clip_R) > 1E6)
                        {
                            flag_stop_sortie_R = false;
                        }            
                    if ((time_us_64() - t_debut_clip_R) > 1E5)
                        {
                            flag_clip_R = false;
                        }
                    if ((niveau_R_core1 & 0x80))
                                {   
                                    if ((niveau_R_core1 & 0x7F) < 15)
                                        {   
                                            D3_ROUGE;
                                            flag_stop_sortie_R = true;
                                            t_debut_clip_R = time_us_64();
                                            flag_clip_R = true;
                                        }   
                                    else
                                        {
                                            if (!flag_clip_R)
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
                                                flag_stop_sortie_R = true;
                                                t_debut_clip_R = time_us_64();
                                                flag_clip_R = true;                    
                                            }   
                                        else
                                            {
                                                if (!flag_clip_R)
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

// Interruption declenchee a chaque fin de transfert d'un canal DMA.
// Version pour 8 echantillons vers DAC. 2 (stereo) x 4 octets x 8 = 64 octets
// envoyes par un canal DMA.
void dma_handler() {
    if (!flag_it)
        {
            // IT pour fin de transfert avec le DMA 1 qui lit la partie
            // basse du buffer buffer_4_i2s.
            flag_it = true;
            // Reinitialisation de la source d'IT.
            dma_hw->ints0 = 1u << dma_chan_4_i2s_1;
            // Reinitialisation du pointeur a la base de la partie basse du buffer
            // pour le prochain transfert par DMA1.
            dma_channel_set_read_addr(dma_chan_4_i2s_1, &buffer_4_i2s[0], false);
        }
    else
        {
            // IT pour fin de transfert avec le DMA 2 qui lit la partie
            // haute du buffer buffer_4_i2s.
            flag_it = false;
            dma_hw->ints0 = 1u << dma_chan_4_i2s_2;
            dma_channel_set_read_addr(dma_chan_4_i2s_2, &buffer_4_i2s[128], false);
        }              
}

int main() {

    // LED integree au PICO 2 utilisee pour DEBUG 
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;

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

    gpio_init(6); // Pour BP1, inutilise (Bouton Poussoir)
    gpio_set_dir(6, GPIO_IN);

    gpio_init(7); // Pour BP2, inutilise
    gpio_set_dir(7, GPIO_IN);

    flag_maj_flash_data = false;

    // Recuperation des donnees de configuration en flash
    for(i=0; i<0x07; i++) flash_data[i] = flash_target_contents[i];
    if ((flash_data[0] != 0xAA) && (flash_data[1] != 0x55)) // Pas de configuration memorisee ?
        {   // Alors, on memorise une configuration par defaut
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
            flag_somme_canaux = false;
            flag_mute_L = true;
            flag_mute_R = true;
        }
    else
        {   // Recuperation de la configuration en cours avant l'arret
            canal_ADAT_pour_voie_L = flash_data[2];
            canal_ADAT_pour_voie_R = flash_data[3];
            flag_somme_canaux = flash_data[4] == 0x00 ? false : true;
            flag_mute_L = flash_data[5] == 0x00 ? false : true;
            flag_mute_R = flash_data[6] == 0x00 ? false : true;
        }

    // PIO 0, machine 0 : Generation de MCLK et BCLK
    uint offset_pio0_sm0 = pio_add_program(pio0, &i2s_mclk_bclk_pio0_sm0_program);
    uint sm_pio0_sm0 = pio_claim_unused_sm(pio0, true);
    pio0_sm0_program_init(pio0, sm_pio0_sm0, offset_pio0_sm0);

    // PIO 0, machine 1 : Generation de LRCLK
    uint offset_pio0_sm1 = pio_add_program(pio0, &i2s_lrclk_pio0_sm1_program);
    uint sm_pio0_sm1 = pio_claim_unused_sm(pio0, true);
    i2s_lrclk_pio0_sm1_program_init(pio0, sm_pio0_sm1, offset_pio0_sm1);

    // PIO 0, machine 2 : Generation de la sortie de donnees I2S    
    uint offset_pio0_sm2 = pio_add_program(pio0, &i2s_data_pio0_sm2_program);
    uint sm_pio0_sm2 = pio_claim_unused_sm(pio0, true);
    i2s_data_pio0_sm2_program_init(pio0, sm_pio0_sm2, offset_pio0_sm2);

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
        &pio0_hw->txf[2], // Ecriture sur la machine 2 du PIO 0 via la FIFO correspondante
        &buffer_4_i2s[0], // Lecture sur la base du buffer,
        32,               // 32 mots de 4 octets, soit 2 x 16 echantillons a 48 KHz d'echantillonnage
        false             // Ne pas demarrer tout de suite
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
        &pio0_hw->txf[2],  // Ecriture sur la machine 2 du PIO 0 via la FIFO correspondante
        &buffer_4_i2s[128],// Lecture sur la seconde moitie du buffer,
        32,                // 32 mots de 4 octets, soit 2 x 16 echantillons a 48 KHz d'echantillonnage
        false              // Ne pas demarrer tout de suite
    );
   
    // Lancement synchronis des 3 machines du PIO0 pour generer les signaux I2S
    pio_enable_sm_mask_in_sync(pio0, (1ULL << sm_pio0_sm0) | (1ULL << sm_pio0_sm1)  | (1ULL << sm_pio0_sm2));
    dma_start_channel_mask(1u << dma_chan_4_i2s_1);

    // Installation du SPI1 a 50 MHz pour le W5500 U1 du schema en reception du flux audio.
    spi_init(spi1, 50000000);
    gpio_set_function(28, GPIO_FUNC_SPI);
    gpio_set_function(26, GPIO_FUNC_SPI);
    gpio_set_function(27, GPIO_FUNC_SPI);

    // Installation du SPI0 a 50 MHz pour le W5500 U2 du schema pour la reemission du flux audio.
    spi_init(spi0, 50000000); // W5500 de recopie.
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);

    gpio_init(2); // /INT du W5500
    gpio_set_dir(2, GPIO_IN);

    gpio_init(22); // /CS du W5500
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);    

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

    // Configuration generale du W5500 de reception. Seule l'adresse MAC est importante.
    buffer_tx_W5500[0] = 0x00;
    buffer_tx_W5500[1] = 0x09;
    buffer_tx_W5500[2] = 0x04;
    buffer_tx_W5500[3] = adresse_MAC_srce[0];
    buffer_tx_W5500[4] = adresse_MAC_srce[1];
    buffer_tx_W5500[5] = adresse_MAC_srce[2];
    buffer_tx_W5500[6] = adresse_MAC_srce[3];
    buffer_tx_W5500[7] = adresse_MAC_srce[4];
    buffer_tx_W5500[8] = adresse_MAC_srce[5];
    gpio_put(22, 0);
    spi_write_read_blocking(spi1, buffer_tx_W5500, buffer_rx_W5500, 9);       
    gpio_put(22, 1);
    
     // Configuration du W5500 en reception du flux audio
    W5500_ecrt_Sn_MR(spi1, 22, 0, 0xF4);// MAC RAW avec filtrage
    W5500_ecrt_Sn_RXBUF_SIZE(spi1, 22, 0, 16);
    W5500_ecrt_Sn_TXBUF_SIZE(spi1, 22, 0, 1);
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
    flag_it_prec = flag_it;
    memset(buffer_4_i2s, 0x00, sizeof (buffer_4_i2s));

    t_debut_clip_L = 0;
    t_debut_clip_L = 0;

    // Lancement de la machine du PIO1 pour la reception des trames de la telecommande
    machine_pio_telec = nec_rx_init(pio1, 13);

    multicore_launch_core1(core1_entry);
   
    t_debut_wd_lien_eth = 0;
    flag_liaison_eth_ok = false;

    flag_data_bruit = true;
    t_debut_data_invalides = time_us_64();

    // Boucle principale
    for(;;)
        {   
            if (!gpio_get(2)) // Si reception sur le W5500, indique par sa ligne /INT
                {
                    t_debut_wd_lien_eth = time_us_64();                    
                    W5500_ecrt_Sn_IR(spi1, 22, 0, 0x04); // Remonte de /INT

                    // Lecture des donnees recues
                    buffer_tx_W5500[0] = PTR_S0_RX_READ>>8; // Lecture des donnees recues sur la socket 0
                    buffer_tx_W5500[1] = PTR_S0_RX_READ;
                    buffer_tx_W5500[2] = 0x18;                                        
                    // Indice 3 : pF du nombre d'octets recus
                    // Indice 4 : pf du nombre d'octets recus
                    // Puis, selon la trame Ethernet :
                    // Indice 5 : 6 octets d'adresse MAC destination
                    // Indice 11 : 6 octets d'adresse MAC source
                    // Indice 17 : Debut des 384 octets (16 trames ADAT) de donnees audio. 
                    // Les deux octets du champ "type" d'une trame Ethernet (inutile car absence
                    // d'usage des couches DoD superieures) contiennent les deux premiers octet
                    // de donnees audio.
                    // Un total de 398 octets sont donc lus dans le buffer de reception de la
                    // socket 0 du W5500.
                    gpio_put(22, 0);
                    spi_write_read_blocking(spi1, buffer_tx_W5500, buffer_rx_W5500, 401); // 401 pour 16 trames ADAT, 209 pour 8 trames ADAT
                    gpio_put(22, 1);
                    // Les 396 octets de donnees recues dans buffer_rx_W5500 vont etre renvoyees a
                    // partir de l'indice 5 par le W5500 d'emission.
                    // Les deux spi_write_read_blocking() suivants ne touchent pas a ces octets.
                    
                    // Mise a jour du pointeur de lecture dans le buffer.
                    PTR_S0_RX_READ+=398;
                    buffer_tx_W5500[0] = 0x00;
                    buffer_tx_W5500[1] = 0x28;
                    buffer_tx_W5500[2] = 0x0C;
                    buffer_tx_W5500[3] = PTR_S0_RX_READ>>8;
                    buffer_tx_W5500[4] = PTR_S0_RX_READ;
                    gpio_put(22, 0);
                    spi_write_read_blocking(spi1, buffer_tx_W5500, buffer_rx_W5500, 5);
                    gpio_put(22, 1);

                    // Validation de la lecture dans S0_CR
                    buffer_tx_W5500[0] = 0x00;
                    buffer_tx_W5500[1] = 0x01;
                    buffer_tx_W5500[2] = 0x0C;
                    buffer_tx_W5500[3] = 0x40;
                    gpio_put(22, 0);
                    spi_write_read_blocking(spi1, buffer_tx_W5500, buffer_rx_W5500, 4);
                    gpio_put(22, 1);

                    // Reemission vers le W5500 de recopie.               
                    buffer_rx_W5500[2] = PTR_S0_TX_WR>>8; // Ecriture sur la FIFO d'emission du W5500.
                    buffer_rx_W5500[3] = PTR_S0_TX_WR;
                    buffer_rx_W5500[4] = 0x14;                    
                    // Comme indique ci-dessus, utilisation de buffer_rx_W5500[] pour
                    // envoyer les donnees (@MAC source, @MAC destination, 384 octets de 16 trames
                    // ADAT) par le W5500 d'emission.
                    gpio_put(17, 0);
                    spi_write_read_blocking(spi0, &buffer_rx_W5500[2], buffer_tx_W5500, 399); // 399 pour 16 trames ADAT, // 207 pour 8 trames ADAT
                    gpio_put(17, 1);

                    // Mise a jour du pointeur du buffer d'emission du W5500
                    PTR_S0_TX_WR+=396;

                    buffer_tx_W5500[0] = 0x00;
                    buffer_tx_W5500[1] = 0x24;
                    buffer_tx_W5500[2] = 0x0C;
                    buffer_tx_W5500[3] = PTR_S0_TX_WR>>8;
                    buffer_tx_W5500[4] = PTR_S0_TX_WR;
                    gpio_put(17, 0);
                    spi_write_read_blocking(spi0, buffer_tx_W5500, buffer_rx_W5500, 5);
                    gpio_put(17, 1);

                    // Validation de l'ecriture dans S0_CR
                    buffer_tx_W5500[0] = 0x00;
                    buffer_tx_W5500[1] = 0x01;
                    buffer_tx_W5500[2] = 0x0C;
                    buffer_tx_W5500[3] = 0x20;
                    gpio_put(17, 0);
                    spi_write_read_blocking(spi0, buffer_tx_W5500, buffer_rx_W5500, 4);
                    gpio_put(17, 1);

                    if (flag_data_bruit) continue;
                    
                    // Attente de passage a l'envoi de l'autre moitie du buffer buffer_4_i2s[]
                    // vers l'I2S via le les canaux DMA et les machines du PIO0.
                    while (flag_it_prec == flag_it);
                    flag_it_prec = flag_it;
                    
                    if (!flag_it)
                        {   
                            // Remplissage de la partie haute du buffer buffer_4_i2s[] depuis
                            // les données recues sur le W5500.
                            // j designe la base de 8 echantillons audio de 3 octets (24 bits) chacun
                            // recus des 8 voies ADAT.
                            // Rappels:
                            // - canal_ADAT_pour_voie_L et canal_ADAT_pour_voie_R affectees dans core 1
                            //   a partir de la telecommande;
                            // - buffer_rx_W5500[] contient les echantillons audio de 16 trames ADAT.
                            for(j = 17, i = 128; j<380; j+=24)                            
                                {
                                    echant1_int32 = buffer_rx_W5500[j + canal_ADAT_pour_voie_L + 2] << 16;
                                    echant1_int32+= buffer_rx_W5500[j + canal_ADAT_pour_voie_L + 1] << 8;
                                    echant1_int32+= buffer_rx_W5500[j + canal_ADAT_pour_voie_L];
                                    // Propagation du bit de signe pour les echantillons negatifs
                                    if (echant1_int32 & 0x800000)
                                        echant1_int32|=0xFF000000;

                                    echant2_int32 = buffer_rx_W5500[j + canal_ADAT_pour_voie_R + 2] << 16;
                                    echant2_int32+= buffer_rx_W5500[j + canal_ADAT_pour_voie_R + 1] << 8;
                                    echant2_int32+= buffer_rx_W5500[j + canal_ADAT_pour_voie_R];
                                    if (echant2_int32 & 0x800000)
                                        echant2_int32|=0xFF000000;

                                    // Affectations des echantillons pour la supervision par le core 1
                                    echant1_4_LED_int32 = echant1_int32;
                                    echant2_4_LED_int32 = echant2_int32;
                                    somme_echant_4_LED_int32 = (echant1_int32>>1) + (echant2_int32>>1);

                                    if (flag_somme_canaux)
                                        {
                                            if (flag_stop_sortie_L || flag_mute_L)
                                                {
                                                    echant1_int32 = 0L;
                                                    echant2_int32 = 0L;
                                                }
                                        }
                                    else
                                        {
                                            if (flag_stop_sortie_L || flag_mute_L) echant1_int32 = 0L;
                                            if (flag_stop_sortie_R || flag_mute_R) echant2_int32 = 0L;
                                        }                                    

                                    if (!flag_somme_canaux)
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
                                            // Attenuation de 6 dB en cas de mode "MONO",
                                            // somme (mix) des deux canaux selectionnes pour la sortie
                                            // gauche
                                            echant1_int32>>=1;                                    
                                            echant2_int32>>=1;                                           
                                            
                                            somme_echant_int32 = echant1_int32 + echant2_int32;
                                            buffer_4_i2s[i+3] = somme_echant_int32 >> 16;
                                            buffer_4_i2s[i+2] = somme_echant_int32 >> 8;
                                            buffer_4_i2s[i+1] = somme_echant_int32;
                                            i+=4;                               
                                            buffer_4_i2s[i+3] = 0x00; // Chut! sur la sortie droite
                                            buffer_4_i2s[i+2] = 0x00;
                                            buffer_4_i2s[i+1] = 0x00;                                                                            
                                            i+=4;
                                        }                                                                       
                                }
                        }                            
                    else
                        {   
                            // Remplissage de la partie basse du buffer buffer_4_i2s[] depuis
                            // les données recues sur le W5500.
                            // Memes commentaires que pour le remplissage de la partie haute
                            for(j = 17, i = 0; j<380; j+=24)
                                {
                                    echant1_int32 = buffer_rx_W5500[j + canal_ADAT_pour_voie_L + 2] << 16;
                                    echant1_int32+= buffer_rx_W5500[j + canal_ADAT_pour_voie_L + 1] << 8;
                                    echant1_int32+= buffer_rx_W5500[j + canal_ADAT_pour_voie_L];
                                    if (echant1_int32 & 0x800000)
                                        echant1_int32|=0xFF000000;

                                    echant2_int32 = buffer_rx_W5500[j + canal_ADAT_pour_voie_R + 2] << 16;
                                    echant2_int32+= buffer_rx_W5500[j + canal_ADAT_pour_voie_R + 1] << 8;
                                    echant2_int32+= buffer_rx_W5500[j + canal_ADAT_pour_voie_R];
                                    if (echant2_int32 & 0x800000)
                                        echant2_int32|=0xFF000000;
                                    echant1_4_LED_int32 = echant1_int32;
                                    echant2_4_LED_int32 = echant2_int32;
                                    somme_echant_4_LED_int32 = (echant1_int32>>1) + (echant2_int32>>1);

                                    if (flag_somme_canaux)
                                        {
                                            if (flag_stop_sortie_L || flag_mute_L)
                                                {
                                                    echant1_int32 = 0L;
                                                    echant2_int32 = 0L;
                                                }
                                        }
                                    else
                                        {
                                            if (flag_stop_sortie_L || flag_mute_L) echant1_int32 = 0L;
                                            if (flag_stop_sortie_R || flag_mute_R) echant2_int32 = 0L;
                                        }

                                    if (!flag_somme_canaux)
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
        // Si une demande de modification de configuration a ete demandee au sein du core 1
        // depuis la telecommande, memorisation en flash de la nouvelle configuration.
        // Effectuer cette memorisation au sein du code sous core 1 provoque un plantage
        // general (?). 
        if (flag_maj_flash_data)
                        {                            
                            flash_data[0] = 0xAA;
                            flash_data[1] = 0x55;
                            flash_data[2] = canal_ADAT_pour_voie_L;
                            flash_data[3] = canal_ADAT_pour_voie_R;
                            flash_data[4] = flag_somme_canaux ? 0xFF : 0x00;
                            flash_data[5] = flag_mute_L ? 0xFF : 0x00;
                            flash_data[6] = flag_mute_R ? 0xFF : 0x00;
                            pio_sm_set_enabled(pio0, sm_pio0_sm0, false);
                            multicore_reset_core1();
                            ints = save_and_disable_interrupts();
                            flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
                            flash_range_program(FLASH_TARGET_OFFSET, flash_data, FLASH_PAGE_SIZE);
                            restore_interrupts (ints);
                            flag_maj_flash_data = false; 
                            multicore_launch_core1(core1_entry);
                            pio_sm_set_enabled(pio0, sm_pio0_sm0, true);                                                                                
                        } 
        }    
    return 0;
}
