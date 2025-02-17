# c-atmega328p-ds18b20-usart

**Project Description - English**

This project demonstrates reading temperature from multiple DS18B20 sensors connected to the One-Wire bus on ATmega328P. The data is transmitted via USART to a computer, allowing real-time monitoring and analysis.

Features:

- One-Wire bus communication implemented in pure C (no Arduino libraries)

- Detection and temperature reading from multiple DS18B20 sensors

- Verification of readings using CRC

- Overtemperature alarm (indicated by LED and UART message)

- Data transmission to a computer via USART

Hardware Requirements:

- ATmega328P microcontroller (e.g., Arduino Nano or standalone AVR)

- One or more DS18B20 temperature sensors

- 4.7kΩ pull-up resistor for the One-Wire bus

- USB-UART converter (e.g., CP2102, FT232RL) to connect to a computer

- Connecting wires

- 5V power supply

Usage Instructions:

- Compile the code in an AVR C-compatible environment (e.g., Atmel Studio, PlatformIO, AVR-GCC).

- Connect the DS18B20 sensors to the One-Wire bus (PD2) with a 4.7kΩ pull-up resistor.

- Connect the ATmega328P to the computer via a USB-UART converter (TX – RX, RX – TX, GND – GND).

- Open a serial terminal (e.g., PuTTY, Tera Term) set to 9600 baud.

- Temperature data from each sensor will be sent to the terminal every second.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

**Opis projektu - Polski**

Ten projekt demonstruje odczyt temperatury z wielu czujników DS18B20 podłączonych do magistrali One-Wire na ATmega328P. Dane są przesyłane przez USART do komputera, co pozwala na ich monitorowanie i analizę w czasie rzeczywistym.

Funkcjonalność:

- Obsługa magistrali One-Wire w czystym C (bez bibliotek Arduino)

- Wykrywanie i odczyt temperatury z wielu czujników DS18B20

- Weryfikacja poprawności odczytów poprzez CRC

- Alarm przekroczenia progu temperatury (sygnalizowany diodą LED i przez UART)

- Przesyłanie danych do komputera przez USART

Wymagania sprzętowe:

- Mikrokontroler ATmega328P (np. Arduino Nano lub standalone AVR)

- Jeden lub więcej czujników DS18B20

- Rezystor pull-up 4.7kΩ dla magistrali One-Wire

- Konwerter USB-UART (np. CP2102, FT232RL) do podłączenia komputera

- Przewody połączeniowe

- Zasilanie 5V

Instrukcja użytkowania:

- Kompiluj kod w środowisku obsługującym AVR C (np. Atmel Studio, PlatformIO, AVR-GCC).

- Podłącz czujniki DS18B20 do magistrali One-Wire (PD2) z rezystorem 4.7kΩ pull-up.

- Podłącz ATmega328P do komputera przez konwerter USB-UART (TX – RX, RX – TX, GND – GND).

- Otwórz terminal szeregowy (np. PuTTY, Tera Term) ustawiony na 9600 baud.

- Dane o temperaturze z każdego czujnika będą wysyłane co sekundę do terminala.
