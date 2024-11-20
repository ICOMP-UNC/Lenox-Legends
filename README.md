# Lenox-Legends

En este repositorio se encuentran los archivos del proyecto utilizando 
la placa STM32 - BluePill con y sin FreeRTOS.

Se implemento la actividad propuesta por el profesor.

## Integrantes del Grupo:
1. Lenox Graham
2. Franco Mamaní
3. Mauricio Mugni

## Esquemático del circuito final
![alt text](<img/Esquematico.SVG>)

## Diagrama de flujos para el código sin FreeRTOS
```mermaid
sequenceDiagram
    participant Sistema as Sistema
    participant SensorMovimiento as Sensor de Movimiento
    participant ADC as ADC
    participant LCD as LCD
    participant Alarma as Alarma
    participant Puerta as Puerta
    participant UART as UART
    participant Buzzer as Buzzer
    participant BotonModo as Botón de Cambio de Modo

    Sistema->>Sistema: Inicia el sistema
    Sistema->>LCD: Inicializa la pantalla LCD
    Sistema->>SensorMovimiento: Configura el sensor de movimiento
    Sistema->>ADC: Configura el ADC para temperatura y batería
    Sistema->>Puerta: Configura los pines de control de la puerta
    Sistema->>Alarma: Configura la alarma y el buzzer
    Sistema->>BotonModo: Configura el botón de cambio de modo

    loop Cada 500ms
        Sistema->>ADC: Lee valores de temperatura y batería
        ADC->>Sistema: Devuelve los valores de temperatura y batería
        Sistema->>LCD: Actualiza la pantalla LCD con los valores de temperatura y batería
        Sistema->>Puerta: Verifica el estado de la puerta (abierta/cerrada)
        Puerta->>Sistema: Devuelve el estado actual de la puerta
        Sistema->>LCD: Muestra el estado de la puerta en la LCD

        alt Movimiento detectado
            Sistema->>Alarma: Activa alarma
            Alarma->>Buzzer: Activa el buzzer
            Buzzer->>Alarma: Mantiene el buzzer activo hasta que no haya movimiento
        else No hay movimiento
            Sistema->>Alarma: Desactiva alarma
            Alarma->>Buzzer: Desactiva el buzzer
        end

        alt Cambio de modo detectado
            BotonModo->>Sistema: Detecta presión del botón
            Sistema->>Sistema: Cambia el modo entre normal y alarma
            Sistema->>LCD: Muestra mensaje indicando el modo actual
        end
    end

    alt Transmisión UART
        Sistema->>UART: Prepara los datos (temperatura, batería, estado puerta)
        UART->>Sistema: Envía datos por UART
        Sistema->>UART: Espera confirmación de transmisión exitosa
        UART->>Sistema: Confirmación de datos enviados
    end

    Sistema->>SensorMovimiento: Revisa si hay cambios en el sensor de movimiento
    SensorMovimiento->>Sistema: Interrupción (si hay movimiento)
    Sistema->>SensorMovimiento: Reacciona a la interrupción (puede activar alarma)

```

## Diagrama de secuencias para el código con FreeRTOS
![alt text](<img/Diagrama_Secuencia.jpg>)

## Imagen del circuito 
![alt text](<img/Circuito_Funcionando.jpeg>)


