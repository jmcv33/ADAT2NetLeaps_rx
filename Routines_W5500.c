#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/spi.h"
//#include "pio_spi.h"
#include "Routines_W5500.h"

extern uint8_t buffer_tx_W5500[], buffer_rx_W5500[];

// L'appel de cette fonction impose d'affecter les octets de la phase de
// donnees (voir doc du W5500) a partir de tampon_tx[3]. 
void W5500_SPI_ecrt_Frame(spi_inst_t *spi, uint cs, uint16_t adresse, uint8_t controle, uint8_t nb_octet)
	{
	 buffer_tx_W5500[0] = adresse>>8; 
	 buffer_tx_W5500[1] = adresse;
	 buffer_tx_W5500[2] = controle;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, nb_octet + 3);
	 gpio_put(cs, 1);
	}

uint8_t W5500_lect_SIR(spi_inst_t *spi, uint cs)
	{buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x17;
	 buffer_tx_W5500[2] = 0x00;
	 buffer_tx_W5500[3] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);
	 return (buffer_rx_W5500[3]);
	 }	

uint16_t W5500_lect_Sn_TX_WR_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x24;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;	 
	 buffer_tx_W5500[3] = 0x00;
	 buffer_tx_W5500[4] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 5);
	 gpio_put(cs, 1);
	 m = (buffer_rx_W5500[3] << 8);
	 m&= 0xFF00;
	 m+= buffer_rx_W5500[4];
	return(m);
	}
	
void W5500_ecrt_Sn_TX_WR_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t adresse)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x24;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = adresse >> 8;
	 buffer_tx_W5500[4] = adresse;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 5);
	 gpio_put(cs, 1); 	
	}	
	
uint16_t W5500_lect_Sn_RX_RD_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x28;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[3] = 0x00;
	 buffer_tx_W5500[4] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 5);	
	 gpio_put(cs, 1); 
	 m = (buffer_rx_W5500[3] << 8);
	 m&= 0xFF00;
	 m+= buffer_rx_W5500[4];
	 return(m);
	}
	
void W5500_ecrt_Sn_RX_RD_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t adresse)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x28;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;	 
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = adresse >> 8;
	 buffer_tx_W5500[4] = adresse;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 5);
	 gpio_put(cs, 1);	 
	}	
	
uint16_t W5500_lect_Sn_TX_FREE_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x20;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[3] = 0x00;
	 buffer_tx_W5500[4] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 5);
	 gpio_put(cs, 1);	 
	 m = (buffer_rx_W5500[3] << 8);
	 m&= 0xFF00;
	 m+= buffer_rx_W5500[4];
	 return(m);
	}
	
uint16_t W5500_lect_Sn_RX_RECEIVED_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x26;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[3] = 0x00;
	 buffer_tx_W5500[4] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 5);
	 gpio_put(cs, 1);	 
	 m = (buffer_rx_W5500[3] << 8);
	 m&= 0xFF00;
	 m+= buffer_rx_W5500[4];
	return(m);
	}	

void W5500_ecrt_Sn_MR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x00;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;		 
	 buffer_tx_W5500[3] = commande;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);	 
	}	
	
void W5500_ecrt_Sn_CR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x01;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;		 
	 buffer_tx_W5500[3] = commande;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);	 
	 do {
		 buffer_tx_W5500[0] = 0x00; 
		 buffer_tx_W5500[1] = 0x01;
		 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
		 buffer_tx_W5500[2]&= 0xF8;
		 buffer_tx_W5500[3] = 0x00;
		 gpio_put(cs, 0);
         spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
		 gpio_put(cs, 1);		 		 
		} while(buffer_rx_W5500[3]);
	}

void W5500_ecrt_no_wait_Sn_CR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x01;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;		 
	 buffer_tx_W5500[3] = commande;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);	 
	}	
	
uint8_t W5500_lect_Sn_SR(spi_inst_t *spi, uint cs, uint8_t numero_socket)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x03;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[3] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);	 
	 return(buffer_rx_W5500[3]);
	}

void W5500_ecrt_Sn_RXBUF_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t taille)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x1E;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = taille;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);	
	}

void W5500_ecrt_Sn_TXBUF_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t taille)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x1F;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = taille;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);	
	}
	
void W5500_ecrt_Sn_SRC_PORT(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t numero_port)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x04;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = numero_port >> 8;
	 buffer_tx_W5500[4] = numero_port;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 5);
	 gpio_put(cs, 1);	
	}
	
void W5500_ecrt_Sn_DEST_PORT(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t numero_port)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x10;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = numero_port >> 8;
	 buffer_tx_W5500[4] = numero_port;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 5);
	 gpio_put(cs, 1);	 
	}
	
void W5500_ecrt_Sn_DEST_IP(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t ipFF, uint8_t ipFL, uint8_t ipLF, uint8_t ipLL)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x0C;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = ipFF;
	 buffer_tx_W5500[4] = ipFL;
	 buffer_tx_W5500[5] = ipLF;
	 buffer_tx_W5500[6] = ipLL;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 7);
	 gpio_put(cs, 1);	
	}
	
void W5500_lect_Sn_DEST_IP(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *ipFF, uint8_t *ipFL, uint8_t *ipLF, uint8_t *ipLL)
	{uint16_t m;
	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x0C;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[3] = 0x00;
	 buffer_tx_W5500[4] = 0x00;
	 buffer_tx_W5500[5] = 0x00;
	 buffer_tx_W5500[6] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 7);
	 gpio_put(cs, 1);
	 *ipFF = buffer_rx_W5500[3];
	 *ipFL = buffer_rx_W5500[4];
	 *ipLF = buffer_rx_W5500[5];
	 *ipLL = buffer_rx_W5500[6];	
	}
	
void W5500_ecrt_Sn_IMR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t masques)
	{	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x2C;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = masques;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);	
	}	
	
void W5500_ecrt_Sn_IR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t flags)	
	{	
	 buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x02;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[2]|= 0x04;	 
	 buffer_tx_W5500[3] = flags;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);	
	}

uint8_t W5500_lect_Sn_IR(spi_inst_t *spi, uint cs, uint8_t numero_socket)	
	{buffer_tx_W5500[0] = 0x00; 
	 buffer_tx_W5500[1] = 0x02;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 1) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 buffer_tx_W5500[3] = 0x00;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 4);
	 gpio_put(cs, 1);
	 return(buffer_rx_W5500[3]);
	}		
			
void W5500_ecrt_Sn_TX_Buffer(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *buf, uint16_t nbre_octets)
	{uint16_t adresse_base;
	 while (W5500_lect_Sn_TX_FREE_SIZE(spi, cs, numero_socket) < nbre_octets);
	 adresse_base = W5500_lect_Sn_TX_WR_PTR(spi, cs, numero_socket);
	 
	 buffer_tx_W5500[0] = adresse_base >> 8; 
	 buffer_tx_W5500[1] = adresse_base;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 2) << 3;
	 buffer_tx_W5500[2]&= 0xF8;	 
	 buffer_tx_W5500[2]|= 0x04;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 3);
     spi_write_blocking(spi, buf, nbre_octets);
	 gpio_put(cs, 1);
	 W5500_ecrt_Sn_TX_WR_PTR(spi, cs, numero_socket, adresse_base + nbre_octets);
	 W5500_ecrt_Sn_CR(spi, cs, numero_socket, 0x20); // Validation de l'ecriture sur la socket
	}
	
void W5500_lect_Sn_RX_Buffer(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *buf, uint16_t nbre_octets)
	{uint16_t adresse_base;
	 adresse_base = W5500_lect_Sn_RX_RD_PTR(spi, cs, numero_socket);	 
	 buffer_tx_W5500[0] = adresse_base >> 8; 
	 buffer_tx_W5500[1] = adresse_base;
	 buffer_tx_W5500[2] = ((numero_socket * 4) + 3) << 3;
	 buffer_tx_W5500[2]&= 0xF8;
	 gpio_put(cs, 0);
     spi_write_read_blocking(spi, buffer_tx_W5500, buffer_rx_W5500, 3);
     spi_read_blocking(spi, 0x00, buf, nbre_octets);
	 gpio_put(cs, 1);	 
	 W5500_ecrt_Sn_RX_RD_PTR(spi, cs, numero_socket, adresse_base + nbre_octets);
	 W5500_ecrt_Sn_CR(spi, cs, numero_socket, 0x40); // Validation de la lecture sur la socket
	}
