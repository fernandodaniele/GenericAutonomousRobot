# Formatear tarjeta SD para usar en este proyecto

## 1. Identificar dispositivo

Primero debemos identificar el nombre del dispositivo en nuestro sistema. Con el siguiente comando inspeccionamos todos los dispositivos de almacenamiento conectados al pc.

```
$ lsblk
```

```
```

Ejemplo salida:
```
$ lsblk
NAME        MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
mmcblk0     179:0    0  14,5G  0 disk
└─mmcblk0p1 179:1    0  14,5G  0 part
nvme0n1     259:0    0 931,5G  0 disk
├─nvme0n1p1 259:1    0   100M  0 part /boot/efi
├─nvme0n1p2 259:2    0    16M  0 part
├─nvme0n1p3 259:3    0   465G  0 part
├─nvme0n1p4 259:4    0   653M  0 part
└─nvme0n1p5 259:5    0 465,8G  0 part /var/log
                                      /var/cache
                                      /home
                                      /
```

En este caso `mmcblk0` es la tarjeta sd y `mmcblk0p1` es la particion que tiene creada. Una vez identificada la tarjeta sd, desmontamos la partición para poder formatearla:

```
$ sudo unmount /dev/mmcblk0p1
```



## 2. Formatear

Para este paso utilizamos el comando `mkfs.vfat` del paquete `dosfstools` para crear el sistema de archivos FAT32 optimizado para microcontroladores.

```
$ sudo mkfs.vfat -F 32 -S 512 -s 1 -n "STM32_SD" /dev/mmcblk0p1
```

Explicación de los flags:
- `-F 32` Fuerza el uso de FAT32
- `-S 512` Establece el tamaño del sector lógico en 512 bytes (Requisito de MAX_SS = 512 en la configuración del FAT_FS del stm32)
- `-s 1` Define un sector por cluster. Esto minimiza el uso de memoria RAM al leer y escribir.
- `-n "NOMBRE"` Asigna una etiqueta al volumen. Para mayor compatibilidad usar un máximo de 8 caracteres.



## 3. Verificar la geometría del disco

Es fundamental confirmar que el sector y la alineación sean correctos para evitar errores de montaje.

```
$ sudo fdisk -l /dev/mmcblk0
```

Ejemplo:

```
$ sudo fdisk -l /dev/mmcblk0
Disco /dev/mmcblk0: 14,48 GiB, 15548284928 bytes, 30367744 sectores
Unidades: sectores de 1 * 512 = 512 bytes
Tamaño de sector (lógico/físico): 512 bytes / 512 bytes
Tamaño de E/S (mínimo/óptimo): 512 bytes / 512 bytes
Tipo de etiqueta de disco: dos
Identificador del disco: 0xe910de44

Disposit.      Inicio Comienzo    Final Sectores Tamaño Id Tipo
/dev/mmcblk0p1            8192 30367743 30359552  14,5G  c W95 FAT32 (LBA)
```

Qué debo verificar:
- `Sector size (logical/physical)` Debe indicar `512 / 512 bytes`.
- `Tipo de etiqueta de disco` debe ser `dos`. Esto indica tabla de particiones MBR.
- `Comienzo (Start)` Lo ideal es que el sector de inicio esté alineado (ej. 8192 o 2048) para mejorar la vida útil de la flash.
- `Tipo` Debe aparecer como `W95 FAT32 (LBA)`.

Con esto verificado ya podemos usar la tarjeta SD en nuestro proyecto con stm32. 
