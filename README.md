ADAT2NetLeaps
Audio sur ADAT vers une chaine d'un nombre quelconque de récepteurs chainés en Ethernet. Chaque récepteur possède deux sorties analogiques,
chacune affectable sur un des 8 canaux ADAT depuis une télécommande infrarouge.
Au coeur de l'émetteur et d'un récepteur : un Raspberry PICO2 à base de RP2350
Transferts réseaux en MAC brut (MAC RAW). Pas d'IP et couches supérieures.
Des switchs peuvent-être insérés.

Latence inférieure à 500 µs entre l'émetteur et les sorties du
premier récepteur; puis ajouter 2 ms par saut sur un récepteur.


L'ensemble est présenté ici : https://youtu.be/pYdALltDKGU
Contact : jeanmarc.villers@wanadoo.fr
- Dépot pour l'émetteur : ADAT2NetLeaps_tx
- Dépot pour un recepteur : ADAT2NetLeaps_rx
