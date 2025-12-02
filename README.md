# PROYECTO_IOT_G2

Grupo - Proyecto - Sensor de ruido

Integrantes:
TACURI FLORES, BRANDON RAFAEL - 20215433

ALVARADO PERALTA, JESUS ANTONIO - 20211688

LLANOS ROMERO, DAVID ANTONIO - 20201638

AGREDA VARGAS, OSCAR ANTONIO - 20193315

LUJAN FERNANDEZ, ANDRES RODRIGO - 20191450

ROJAS ZARATE, DIEGO MAURICIO - 20191267


Instrucciones para correr la app:

• En un servidor linux ya sea una vm de aws o cualquier vm
• Asegurarse que en integraciones de chirpstack se añada el endopoint http: http://<IP>:9000/chirpstack
1) Clonar el repositorio y ubicarse dentro del fichero ruido-monitor
2) docker-compose up -d --build (Con este comando se levantan los servicios en los 3 contenedores)
3) Con esto ya se estarían enviando datos del sensor y guardandolo en la base de datos influx
4) Luego ya podrás graficar en grafana mediante querys accediendo a http://<IP>:3000/
5) Adjunto el dashboard que puede ser importado en grafana para que se pueda visualziar distintos tipos de gráficos
6) LISTO 



