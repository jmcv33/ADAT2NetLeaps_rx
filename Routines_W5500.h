#include "pico/stdlib.h"

void W5500_SPI_ecrt_Frame(spi_inst_t *spi, uint cs, uint16_t adresse, uint8_t controle, uint8_t nb_octet);
uint8_t W5500_lect_SIR(spi_inst_t *spi, uint cs);
uint16_t W5500_lect_Sn_TX_WR_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket);
void W5500_ecrt_Sn_TX_WR_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t adresse);
uint16_t W5500_lect_Sn_RX_RD_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket);
void W5500_ecrt_Sn_RX_RD_PTR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t adresse);
uint16_t W5500_lect_Sn_TX_FREE_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket);
uint16_t W5500_lect_Sn_RX_RECEIVED_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket);
void W5500_ecrt_Sn_MR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande);
void W5500_ecrt_Sn_CR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande);
void W5500_ecrt_no_wait_Sn_CR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t commande);
uint8_t W5500_lect_Sn_SR(spi_inst_t *spi, uint cs, uint8_t numero_socket);
void W5500_ecrt_Sn_RXBUF_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t taille);
void W5500_ecrt_Sn_TXBUF_SIZE(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t taille);
void W5500_ecrt_Sn_SRC_PORT(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t numero_port);
void W5500_ecrt_Sn_DEST_PORT(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint16_t numero_port);
void W5500_ecrt_Sn_DEST_IP(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t ipFF, uint8_t ipFL, uint8_t ipLF, uint8_t ipLL);
void W5500_lect_Sn_DEST_IP(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *ipFF, uint8_t *ipFL, uint8_t *ipLF, uint8_t *ipLL);
void W5500_ecrt_Sn_IMR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t masques);
void W5500_ecrt_Sn_IR(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t flags);
uint8_t W5500_lect_Sn_IR(spi_inst_t *spi, uint cs, uint8_t numero_socket);
void W5500_ecrt_Sn_TX_Buffer(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *buf, uint16_t nbre_octets);
void W5500_lect_Sn_RX_Buffer(spi_inst_t *spi, uint cs, uint8_t numero_socket, uint8_t *buf, uint16_t nbre_octets);
