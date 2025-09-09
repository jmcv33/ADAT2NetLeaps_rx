#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/spi.h"
//#include "pio_spi.h"
#include "Routines_W5500.h"

extern uint8_t tampon_tx[], tampon_rx[];

// L'appel de cette fonction impose d'affecter les octets de la phase de
// donnees (voir doc du W5500) a partir de tampon_tx[3]. 
void W5500_SPI_ecrt_Frame(spi_inst_t *spi, uint cs, uint16_t adresse, uint8_t controle, uint8_t nb_octet)
	{
	 tampon_tx[0] = adresse>>8; 
	 tampon_tx[1] = adresse;
	 tampon_tx[2] = controle;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, nb_octet + 3);
	 gpio_put(cs, 1);
	}

uint8_t W5500_lect_SIR(spi_inst_t *spi, uint cs)
	{tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x17;
	 tampon_tx[2] = 0x00;
	 tampon_tx[3] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);
	 return (tampon_rx[3]);
	 }	

uint16_t W5500_lect_Sn_TX_WR_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x24;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;	 
	 tampon_tx[3] = 0x00;
	 tampon_tx[4] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 5);
	 gpio_put(cs, 1);
	 m = (tampon_rx[3] << 8);
	 m&= 0xFF00;
	 m+= tampon_rx[4];
	return(m);
	}
	
void W5500_ecrt_Sn_TX_WR_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t adresse)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x24;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = adresse >> 8;
	 tampon_tx[4] = adresse;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 5);
	 gpio_put(cs, 1); 	
	}	
	
uint16_t W5500_lect_Sn_RX_RD_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x28;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[3] = 0x00;
	 tampon_tx[4] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 5);	
	 gpio_put(cs, 1); 
	 m = (tampon_rx[3] << 8);
	 m&= 0xFF00;
	 m+= tampon_rx[4];
	 return(m);
	}
	
void W5500_ecrt_Sn_RX_RD_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t adresse)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x28;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;	 
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = adresse >> 8;
	 tampon_tx[4] = adresse;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 5);
	 gpio_put(cs, 1);	 
	}	
	
uint16_t W5500_lect_Sn_TX_FREE_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x20;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[3] = 0x00;
	 tampon_tx[4] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 5);
	 gpio_put(cs, 1);	 
	 m = (tampon_rx[3] << 8);
	 m&= 0xFF00;
	 m+= tampon_rx[4];
	 return(m);
	}
	
uint16_t W5500_lect_Sn_RX_RECEIVED_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x26;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[3] = 0x00;
	 tampon_tx[4] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 5);
	 gpio_put(cs, 1);	 
	 m = (tampon_rx[3] << 8);
	 m&= 0xFF00;
	 m+= tampon_rx[4];
	return(m);
	}	

void W5500_ecrt_Sn_MR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x00;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;		 
	 tampon_tx[3] = commande;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);	 
	}	
	
void W5500_ecrt_Sn_CR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x01;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;		 
	 tampon_tx[3] = commande;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);	 
	 do {
		 tampon_tx[0] = 0x00; 
		 tampon_tx[1] = 0x01;
		 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
		 tampon_tx[2]&= 0xF8;
		 tampon_tx[3] = 0x00;
		 gpio_put(cs, 0);
         spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
		 gpio_put(cs, 1);		 		 
		} while(tampon_rx[3]);
	}

void W5500_ecrt_no_wait_Sn_CR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x01;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;		 
	 tampon_tx[3] = commande;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);	 
	}	
	
uint8_t W5500_lect_Sn_SR(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x03;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[3] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);	 
	 return(tampon_rx[3]);
	}

void W5500_ecrt_Sn_RXBUF_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t taille)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x1E;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = taille;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);	
	}

void W5500_ecrt_Sn_TXBUF_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t taille)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x1F;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = taille;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);	
	}
	
void W5500_ecrt_Sn_SRC_PORT(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t numero_port)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x04;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = numero_port >> 8;
	 tampon_tx[4] = numero_port;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 5);
	 gpio_put(cs, 1);	
	}
	
void W5500_ecrt_Sn_DEST_PORT(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t numero_port)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x10;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = numero_port >> 8;
	 tampon_tx[4] = numero_port;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 5);
	 gpio_put(cs, 1);	 
	}
	
void W5500_ecrt_Sn_DEST_IP(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t ipFF, uint8_t ipFL, uint8_t ipLF, uint8_t ipLL)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x0C;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = ipFF;
	 tampon_tx[4] = ipFL;
	 tampon_tx[5] = ipLF;
	 tampon_tx[6] = ipLL;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 7);
	 gpio_put(cs, 1);	
	}
	
void W5500_lect_Sn_DEST_IP(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *ipFF, uint8_t *ipFL, uint8_t *ipLF, uint8_t *ipLL)
	{uint16_t m;
	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x0C;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[3] = 0x00;
	 tampon_tx[4] = 0x00;
	 tampon_tx[5] = 0x00;
	 tampon_tx[6] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 7);
	 gpio_put(cs, 1);
	 *ipFF = tampon_rx[3];
	 *ipFL = tampon_rx[4];
	 *ipLF = tampon_rx[5];
	 *ipLL = tampon_rx[6];	
	}
	
void W5500_ecrt_Sn_IMR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t masques)
	{	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x2C;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = masques;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);	
	}	
	
void W5500_ecrt_Sn_IR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t flags)	
	{	
	 tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x02;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[2]|= 0x04;	 
	 tampon_tx[3] = flags;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);	
	}

uint8_t W5500_lect_Sn_IR(spi_inst_t *spi, uint cs, uint8_t numero_socket)	
	{tampon_tx[0] = 0x00; 
	 tampon_tx[1] = 0x02;
	 tampon_tx[2] = ((numero_socket * 4) + 1) << 3;
	 tampon_tx[2]&= 0xF8;
	 tampon_tx[3] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 4);
	 gpio_put(cs, 1);
	 return(tampon_rx[3]);
	}		
			
void W5500_ecrt_Sn_TX_Buffer(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *buf, uint16_t nbre_octets)
	{uint16_t adresse_base;
	 while (W5500_lect_Sn_TX_FREE_SIZE(spi, cs, numero_socket) < nbre_octets);
	 adresse_base = W5500_lect_Sn_TX_WR_PTR(spi, cs, numero_socket);
	 
	 tampon_tx[0] = adresse_base >> 8; 
	 tampon_tx[1] = adresse_base;
	 tampon_tx[2] = ((numero_socket * 4) + 2) << 3;
	 tampon_tx[2]&= 0xF8;	 
	 tampon_tx[2]|= 0x04;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 3);
     spi_write_blocking(spi, buf, nbre_octets);
	 gpio_put(cs, 1);
	 W5500_ecrt_Sn_TX_WR_PTR(spi, cs, numero_socket, adresse_base + nbre_octets);
	 W5500_ecrt_Sn_CR(spi, cs, numero_socket, 0x20); // Validation de l'ecriture sur la socket
	}
	
void W5500_lect_Sn_RX_Buffer(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *buf, uint16_t nbre_octets)
	{uint16_t adresse_base;
	 adresse_base = W5500_lect_Sn_RX_RD_PTR(spi, cs, numero_socket);	 
	 tampon_tx[0] = adresse_base >> 8; 
	 tampon_tx[1] = adresse_base;
	 tampon_tx[2] = ((numero_socket * 4) + 3) << 3;
	 tampon_tx[2]&= 0xF8;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, tampon_tx, tampon_rx, 3);
     spi_read_blocking(spi, 0x00, buf, nbre_octets);
	 gpio_put(cs, 1);	 
	 W5500_ecrt_Sn_RX_RD_PTR(spi, cs, numero_socket, adresse_base + nbre_octets);
	 W5500_ecrt_Sn_CR(spi, cs, numero_socket, 0x40); // Validation de la lecture sur la socket
	}
