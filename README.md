
# Proyecto Final: Balancín con Bola

## 1. Descripción
Desarrollo de un sistema de control de posición de una bola
sobre una viga, utilizando un ESP32, un sensor infrarrojo
Sharp GP2Y0A21 y un servomotor MG996R.

## 2. Hardware
- ESP32
- Sensor Sharp GP2Y0A21
- Servomotor MG996R
- Estructura mecánica del balancín

## 3. Conexiones
- Sensor Sharp: GPIO 34
- Servomotor MG996R: GPIO 27

## 4. Software
- Visual Studio Code
- PlatformIO
- Arduino Framework
- Teleplot para visualización de datos

## 5. Modelo matemático
Modelo identificado de primer orden:

G(s) = 43 / (4s + 1)

## 6. Controlador
Controlador PI diseñado mediante el método
de Truxal-Ragazzini.

## 7. Ejecución
1. Abrir el proyecto en Visual Studio Code.
2. Instalar la extensión PlatformIO.
3. Conectar el ESP32.
4. Compilar y cargar el programa.
5. Abrir el puerto serial y utilizar Teleplot
   para visualizar las variables del sistema.
